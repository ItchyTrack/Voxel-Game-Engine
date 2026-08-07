use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use std::time::Duration;

use bevy::prelude::*;
use lightyear::prelude::PeerId;
use voxel_data::compressed_voxels::{CompressVoxelsError, CompressedVoxels};
use voxel_data::grid::GridId;
use voxel_data::voxels::{VoxelTypeId, Voxels};
use voxel_sources::{CancellationToken, TileVoxelCancellation, TileVoxelLoadRequest, VoxelSourcesRequestHandle};

use super::super::{
	VoxelLoadFinished,
	VoxelLoadId,
	VoxelLoadOutcome,
	VoxelLoadRequest,
	VoxelLoadRequestKind,
	VoxelLoadResponse,
	VoxelLoadResponseKind,
};

const RESPONSE_RETRY_INTERVAL: Duration = Duration::from_millis(500);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct PendingChunkKey {
	grid: GridId,
	chunk: IVec3,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct PendingTileVoxelKey {
	grid: GridId,
	key: voxel_sources::TileVoxelKey,
	voxel_type: VoxelTypeId,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct VoxelLoadOwner {
	peer: PeerId,
	id: VoxelLoadId,
}

struct ChunkPayload {
	generation: u64,
	voxels: Option<CompressedVoxels>,
}

struct TileVoxelPayload {
	generation: u64,
	voxel_type: VoxelTypeId,
	voxels: Option<CompressedVoxels>,
}

enum VoxelLoadStatus<T> {
	Loading,
	Ready { payload: Arc<T>, next_send_at: f64 },
}

struct ChunkVoxelLoad {
	key: PendingChunkKey,
	status: VoxelLoadStatus<ChunkPayload>,
}

struct TileVoxelLoad {
	key: PendingTileVoxelKey,
	status: VoxelLoadStatus<TileVoxelPayload>,
}

enum ServerVoxelLoad {
	Chunk(ChunkVoxelLoad),
	TileVoxels(TileVoxelLoad),
}

struct PendingChunkLoad {
	subscribers: HashSet<VoxelLoadOwner>,
	cancellation: CancellationToken,
}

struct PendingTileVoxelsLoad {
	subscribers: HashSet<VoxelLoadOwner>,
	cancellation: TileVoxelCancellation,
}

#[derive(Resource, Default)]
pub(crate) struct PendingVoxelLoads {
	clients: HashMap<PeerId, HashMap<VoxelLoadId, ServerVoxelLoad>>,
	cancelled: HashMap<PeerId, HashSet<VoxelLoadId>>,
	chunk_loads: HashMap<PendingChunkKey, PendingChunkLoad>,
	tile_voxel_loads: HashMap<PendingTileVoxelKey, PendingTileVoxelsLoad>,
}

impl PendingVoxelLoads {
	pub(super) fn request(&mut self, peer: PeerId, request: VoxelLoadRequest, sources: &VoxelSourcesRequestHandle) {
		if self.take_cancellation(peer, request.id) {
			return;
		}
		self.remove(peer, request.id);
		let owner = VoxelLoadOwner { peer, id: request.id };
		match request.kind {
			VoxelLoadRequestKind::Chunk { grid, chunk } => {
				let key = PendingChunkKey { grid, chunk };
				self.clients.entry(peer).or_default().insert(request.id, ServerVoxelLoad::Chunk(ChunkVoxelLoad {
					key,
					status: VoxelLoadStatus::Loading,
				}));
				if let Some(load) = self.chunk_loads.get_mut(&key) {
					load.subscribers.insert(owner);
				} else {
					let cancellation = sources.request_chunk(voxel_sources::ChunkLoadRequest { grid, chunk });
					self.chunk_loads.insert(key, PendingChunkLoad { subscribers: HashSet::from([owner]), cancellation });
				}
			}
			VoxelLoadRequestKind::TileVoxels { grid, key, voxel_type, priority } => {
				let pending_key = PendingTileVoxelKey { grid, key, voxel_type };
				self.clients.entry(peer).or_default().insert(request.id, ServerVoxelLoad::TileVoxels(TileVoxelLoad {
					key: pending_key,
					status: VoxelLoadStatus::Loading,
				}));
				if let Some(load) = self.tile_voxel_loads.get_mut(&pending_key) {
					load.subscribers.insert(owner);
				} else {
					let cancellation = sources.request_tile_voxels(TileVoxelLoadRequest::raw(
						grid, Entity::PLACEHOLDER, key, voxel_type, 0, priority,
					));
					self.tile_voxel_loads.insert(pending_key, PendingTileVoxelsLoad { subscribers: HashSet::from([owner]), cancellation });
				}
			}
		}
	}

	pub(super) fn finish(&mut self, peer: PeerId, event: VoxelLoadFinished) {
		if self.remove(peer, event.id) {
			return;
		}
		if event.outcome == VoxelLoadOutcome::Cancelled {
			self.cancelled.entry(peer).or_default().insert(event.id);
		}
	}

	pub(super) fn complete_chunk(
		&mut self,
		grid: GridId,
		chunk: IVec3,
		generation: u64,
		voxels: Option<&Voxels>,
	) -> Result<(), CompressVoxelsError> {
		let key = PendingChunkKey { grid, chunk };
		let Some(load) = self.chunk_loads.remove(&key) else { return Ok(()) };
		let voxels = match compress(voxels) {
			Ok(voxels) => voxels,
			Err(err) => {
				for owner in load.subscribers {
					self.remove(owner.peer, owner.id);
				}
				return Err(err);
			}
		};
		let payload = Arc::new(ChunkPayload { generation, voxels });
		for owner in load.subscribers {
			self.set_chunk_ready(owner, key, payload.clone());
		}
		Ok(())
	}

	pub(super) fn complete_tile_voxels(
		&mut self,
		grid: GridId,
		key: voxel_sources::TileVoxelKey,
		generation: u64,
		voxel_type: VoxelTypeId,
		voxels: Option<&Voxels>,
	) -> Result<(), CompressVoxelsError> {
		let pending_key = PendingTileVoxelKey { grid, key, voxel_type };
		let Some(load) = self.tile_voxel_loads.remove(&pending_key) else { return Ok(()) };
		let voxels = match compress(voxels) {
			Ok(voxels) => voxels,
			Err(err) => {
				for owner in load.subscribers {
					self.remove(owner.peer, owner.id);
				}
				return Err(err);
			}
		};
		let payload = Arc::new(TileVoxelPayload { generation, voxel_type, voxels });
		for owner in load.subscribers {
			self.set_tile_voxels_ready(owner, pending_key, payload.clone());
		}
		Ok(())
	}

	pub(super) fn send_due(&mut self, now: f64, mut send: impl FnMut(PeerId, VoxelLoadResponse)) {
		for (&peer, voxel_loads) in &mut self.clients {
			for (&id, voxel_load) in voxel_loads {
				let response = match voxel_load {
					ServerVoxelLoad::Chunk(ChunkVoxelLoad { key, status: VoxelLoadStatus::Ready { payload, next_send_at } }) if now >= *next_send_at => {
						*next_send_at = now + RESPONSE_RETRY_INTERVAL.as_secs_f64();
						VoxelLoadResponse {
							id,
							kind: VoxelLoadResponseKind::Chunk {
								grid: key.grid,
								chunk: key.chunk,
								generation: payload.generation,
								voxels: payload.voxels.clone(),
							},
						}
					}
					ServerVoxelLoad::TileVoxels(TileVoxelLoad { key, status: VoxelLoadStatus::Ready { payload, next_send_at } }) if now >= *next_send_at => {
						*next_send_at = now + RESPONSE_RETRY_INTERVAL.as_secs_f64();
						VoxelLoadResponse {
							id,
							kind: VoxelLoadResponseKind::TileVoxels {
								grid: key.grid,
								key: key.key,
								generation: payload.generation,
								voxel_type: payload.voxel_type,
								voxels: payload.voxels.clone(),
							},
						}
					}
					_ => continue,
				};
				send(peer, response);
			}
		}
	}

	pub(super) fn remove_disconnected(&mut self, mut connected: impl FnMut(PeerId) -> bool) {
		let disconnected: Vec<_> = self.clients.keys().filter(|&&peer| !connected(peer)).copied().collect();
		for peer in disconnected {
			self.remove_client(peer);
		}
		self.cancelled.retain(|peer, _| connected(*peer));
	}

	fn remove(&mut self, peer: PeerId, id: VoxelLoadId) -> bool {
		let Some(voxel_load) = self.clients.get_mut(&peer).and_then(|voxel_loads| voxel_loads.remove(&id)) else { return false };
		if self.clients.get(&peer).is_some_and(HashMap::is_empty) {
			self.clients.remove(&peer);
		}

		let owner = VoxelLoadOwner { peer, id };
		match voxel_load {
			ServerVoxelLoad::Chunk(ChunkVoxelLoad { key, status: VoxelLoadStatus::Loading }) => {
				let should_cancel = self.chunk_loads.get_mut(&key).is_some_and(|load| {
					load.subscribers.remove(&owner);
					load.subscribers.is_empty()
				});
				if should_cancel {
					if let Some(load) = self.chunk_loads.remove(&key) {
						load.cancellation.cancel();
					}
				}
			}
			ServerVoxelLoad::TileVoxels(TileVoxelLoad { key, status: VoxelLoadStatus::Loading }) => {
				let should_cancel = self.tile_voxel_loads.get_mut(&key).is_some_and(|load| {
					load.subscribers.remove(&owner);
					load.subscribers.is_empty()
				});
				if should_cancel {
					if let Some(load) = self.tile_voxel_loads.remove(&key) {
						load.cancellation.cancel();
					}
				}
			}
			_ => {}
		}
		true
	}

	fn take_cancellation(&mut self, peer: PeerId, id: VoxelLoadId) -> bool {
		let cancelled = self.cancelled.get_mut(&peer).is_some_and(|ids| ids.remove(&id));
		if self.cancelled.get(&peer).is_some_and(HashSet::is_empty) {
			self.cancelled.remove(&peer);
		}
		cancelled
	}

	fn remove_client(&mut self, peer: PeerId) {
		let ids: Vec<_> = self.clients.get(&peer).into_iter().flat_map(|voxel_loads| voxel_loads.keys().copied()).collect();
		for id in ids {
			self.remove(peer, id);
		}
		self.cancelled.remove(&peer);
	}

	fn set_chunk_ready(&mut self, owner: VoxelLoadOwner, key: PendingChunkKey, payload: Arc<ChunkPayload>) {
		let Some(ServerVoxelLoad::Chunk(voxel_load)) = self.clients.get_mut(&owner.peer).and_then(|voxel_loads| voxel_loads.get_mut(&owner.id)) else { return };
		if voxel_load.key == key && matches!(voxel_load.status, VoxelLoadStatus::Loading) {
			voxel_load.status = VoxelLoadStatus::Ready { payload, next_send_at: 0.0 };
		}
	}

	fn set_tile_voxels_ready(&mut self, owner: VoxelLoadOwner, key: PendingTileVoxelKey, payload: Arc<TileVoxelPayload>) {
		let Some(ServerVoxelLoad::TileVoxels(voxel_load)) = self.clients.get_mut(&owner.peer).and_then(|voxel_loads| voxel_loads.get_mut(&owner.id)) else { return };
		if voxel_load.key == key && matches!(voxel_load.status, VoxelLoadStatus::Loading) {
			voxel_load.status = VoxelLoadStatus::Ready { payload, next_send_at: 0.0 };
		}
	}
}

fn compress(voxels: Option<&Voxels>) -> Result<Option<CompressedVoxels>, CompressVoxelsError> {
	voxels.map(CompressedVoxels::with_max_compression).transpose()
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn early_cancellation_is_consumed_once() {
		let peer = PeerId::Local(1);
		let id = VoxelLoadId(42);
		let mut pending = PendingVoxelLoads::default();

		pending.finish(peer, VoxelLoadFinished { id, outcome: VoxelLoadOutcome::Cancelled });
		assert!(pending.take_cancellation(peer, id));
		assert!(!pending.take_cancellation(peer, id));
		assert!(!pending.cancelled.contains_key(&peer));
	}

	#[test]
	fn disconnect_removes_cancellation_tombstones() {
		let peer = PeerId::Local(1);
		let mut pending = PendingVoxelLoads::default();
		pending.finish(peer, VoxelLoadFinished { id: VoxelLoadId(42), outcome: VoxelLoadOutcome::Cancelled });

		pending.remove_client(peer);

		assert!(!pending.cancelled.contains_key(&peer));
	}

	#[test]
	fn shared_chunk_load_is_cancelled_after_its_last_client_leaves() {
		let first = VoxelLoadOwner { peer: PeerId::Local(1), id: VoxelLoadId(1) };
		let second = VoxelLoadOwner { peer: PeerId::Local(2), id: VoxelLoadId(2) };
		let key = PendingChunkKey { grid: Entity::PLACEHOLDER, chunk: IVec3::ZERO };
		let cancellation = CancellationToken::new();
		let mut pending = PendingVoxelLoads::default();
		pending.clients.entry(first.peer).or_default().insert(first.id, ServerVoxelLoad::Chunk(ChunkVoxelLoad { key, status: VoxelLoadStatus::Loading }));
		pending.clients.entry(second.peer).or_default().insert(second.id, ServerVoxelLoad::Chunk(ChunkVoxelLoad { key, status: VoxelLoadStatus::Loading }));
		pending.chunk_loads.insert(key, PendingChunkLoad {
			subscribers: HashSet::from([first, second]),
			cancellation: cancellation.clone(),
		});

		pending.remove(first.peer, first.id);
		assert!(!cancellation.is_cancelled());
		assert!(pending.chunk_loads.contains_key(&key));

		pending.remove(second.peer, second.id);
		assert!(cancellation.is_cancelled());
		assert!(!pending.chunk_loads.contains_key(&key));
	}
}
