use std::collections::{BTreeMap, HashMap, VecDeque};

use bevy::log::warn;
use bevy::math::IVec3;
use voxel_data::compressed_voxels::CompressedVoxels;
use voxel_data::grid::GridId;
use voxel_data::voxels::VoxelTypeId;
use voxel_sources::{CancellationToken, TileVoxelKey, SourceHandle};

use super::super::{
	VoxelLoadFinished,
	VoxelLoadId,
	VoxelLoadOutcome,
	VoxelLoadRequest,
	VoxelLoadRequestKind,
	VoxelLoadResponse,
	VoxelLoadResponseKind,
};
use crate::chunks::invalidation::{RemoteChunkChangeKind, RemoteChunkChanged};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct PendingChunkKey {
	grid: GridId,
	chunk: IVec3,
}

#[derive(Clone, Debug)]
struct PendingChunk {
	key: PendingChunkKey,
	request_generation: u64,
	possible_generations: BTreeMap<u64, u64>,
	cancellation: CancellationToken,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct PendingTileVoxelKey {
	grid: GridId,
	key: TileVoxelKey,
	voxel_type: VoxelTypeId,
}

#[derive(Clone, Debug)]
struct PendingTileVoxels {
	key: PendingTileVoxelKey,
	request_generation: u64,
	possible_generations: BTreeMap<u64, u64>,
	cancellation: CancellationToken,
}

enum PendingVoxelLoad {
	Chunk(PendingChunk),
	TileVoxels(PendingTileVoxels),
}

fn generation_for_response(request_generation: u64, possible_generations: &BTreeMap<u64, u64>, source_generation: u64) -> u64 {
	possible_generations
		.range(..=source_generation)
		.next_back()
		.map_or(request_generation, |(_, local_claim_generation)| *local_claim_generation)
}

impl PendingVoxelLoad {
	fn is_cancelled(&self) -> bool {
		match self {
			Self::Chunk(pending) => pending.cancellation.is_cancelled(),
			Self::TileVoxels(pending) => pending.cancellation.is_cancelled(),
		}
	}
}

#[derive(Default)]
pub(crate) struct ClientLoadRegistry {
	next_id: u64,
	pending: HashMap<VoxelLoadId, PendingVoxelLoad>,
	chunk_ids: HashMap<PendingChunkKey, VoxelLoadId>,
	tile_voxel_ids: HashMap<PendingTileVoxelKey, VoxelLoadId>,
	requests: VecDeque<VoxelLoadRequest>,
	finished: VecDeque<VoxelLoadFinished>,
}

impl ClientLoadRegistry {
	pub fn request_chunk(&mut self, grid: GridId, chunk: IVec3, request_generation: u64, cancellation: CancellationToken) {
		let key = PendingChunkKey { grid, chunk };
		if let Some(previous) = self.chunk_ids.get(&key).copied() {
			self.finish(previous, VoxelLoadOutcome::Cancelled);
		}

		let id = self.allocate_id();
		self.pending.insert(id, PendingVoxelLoad::Chunk(PendingChunk {
			key,
			request_generation,
			possible_generations: BTreeMap::new(),
			cancellation,
		}));
		self.chunk_ids.insert(key, id);
		self.requests.push_back(VoxelLoadRequest { id, kind: VoxelLoadRequestKind::Chunk { grid, chunk } });
	}

	pub fn request_tile_voxels(
		&mut self,
		grid: GridId,
		key: TileVoxelKey,
		voxel_type: VoxelTypeId,
		priority: f32,
		request_generation: u64,
		cancellation: CancellationToken,
	) {
		let pending_key = PendingTileVoxelKey { grid, key, voxel_type };
		if let Some(previous) = self.tile_voxel_ids.get(&pending_key).copied() {
			self.finish(previous, VoxelLoadOutcome::Cancelled);
		}

		let id = self.allocate_id();
		self.pending.insert(id, PendingVoxelLoad::TileVoxels(PendingTileVoxels {
			key: pending_key,
			request_generation,
			possible_generations: BTreeMap::new(),
			cancellation,
		}));
		self.tile_voxel_ids.insert(pending_key, id);
		self.requests.push_back(VoxelLoadRequest { id, kind: VoxelLoadRequestKind::TileVoxels { grid, key, voxel_type, priority } });
	}

	pub fn drain_cancelled(&mut self) {
		let cancelled: Vec<_> = self.pending.iter().filter_map(|(&id, pending)| pending.is_cancelled().then_some(id)).collect();
		for id in cancelled {
			self.finish(id, VoxelLoadOutcome::Cancelled);
		}
	}

	pub fn receive_chunk_changed(&mut self, handle: &SourceHandle, event: RemoteChunkChanged) {
		let generation = match event.kind {
			RemoteChunkChangeKind::Changed { generation } | RemoteChunkChangeKind::Removed { generation } => generation,
		};
		let claim_generation = match event.kind {
			RemoteChunkChangeKind::Changed { .. } => Some(handle.claim(event.grid, event.min, event.size)),
			RemoteChunkChangeKind::Removed { .. } => None,
		};
		for x in event.min.x..event.min.x + event.size.x {
			for y in event.min.y..event.min.y + event.size.y {
				for z in event.min.z..event.min.z + event.size.z {
					let chunk = IVec3::new(x, y, z);
					let key = PendingChunkKey { grid: event.grid, chunk };
					match event.kind {
						RemoteChunkChangeKind::Changed { .. } => {
							if let Some(id) = self.chunk_ids.get(&key) {
								let Some(PendingVoxelLoad::Chunk(pending)) = self.pending.get_mut(id) else {
									unreachable!("chunk voxel load index is inconsistent")
								};
								pending.possible_generations.insert(generation, claim_generation.unwrap());
							}
						}
						RemoteChunkChangeKind::Removed { .. } => {
							if let Some(id) = self.chunk_ids.get(&key).copied() {
								self.finish(id, VoxelLoadOutcome::Cancelled);
							}
							handle.unavailable(event.grid, chunk, IVec3::ONE);
						}
					}
				}
			}
		}

		let Some(claim_generation) = claim_generation else { return };
		for voxel_load in self.pending.values_mut() {
			let PendingVoxelLoad::TileVoxels(pending) = voxel_load else { continue };
			if pending.key.grid != event.grid || !tile_voxels_overlap_area(pending.key.key, event.min, event.size) {
				continue;
			}
			pending.possible_generations.insert(generation, claim_generation);
		}
	}

	pub fn receive_response(&mut self, handle: &SourceHandle, from: impl std::fmt::Debug, response: &mut VoxelLoadResponse) {
		match &mut response.kind {
			VoxelLoadResponseKind::Chunk { grid, chunk, generation, voxels } => {
				self.receive_chunk_response(handle, from, response.id, *grid, *chunk, *generation, voxels);
			}
			VoxelLoadResponseKind::TileVoxels { grid, key, generation, voxel_type, voxels } => {
				self.receive_tile_voxel_response(handle, from, response.id, *grid, *key, *generation, *voxel_type, voxels);
			}
		}
	}

	pub(crate) fn pop_request(&mut self) -> Option<VoxelLoadRequest> { self.requests.pop_front() }
	pub(crate) fn pop_finished(&mut self) -> Option<VoxelLoadFinished> { self.finished.pop_front() }

	fn receive_chunk_response(
		&mut self,
		handle: &SourceHandle,
		from: impl std::fmt::Debug,
		id: VoxelLoadId,
		grid: GridId,
		chunk: IVec3,
		generation: u64,
		compressed: &mut Option<CompressedVoxels>,
	) {
		let Some(pending) = self.pending.get(&id) else { return };
		let PendingVoxelLoad::Chunk(pending) = pending else {
			warn!(?id, ?from, "ignoring chunk response for a non-chunk voxel load");
			return;
		};
		if pending.cancellation.is_cancelled() {
			self.finish(id, VoxelLoadOutcome::Cancelled);
			return;
		}
		if pending.key.grid != grid || pending.key.chunk != chunk {
			warn!(?id, ?grid, ?chunk, ?from, "ignoring mismatched remote chunk response");
			return;
		}

		let voxels = match decompress(compressed) {
			Ok(voxels) => voxels,
			Err(err) => {
				warn!(?id, ?grid, ?chunk, ?from, error=%err, "failed to decompress remote chunk response");
				return;
			}
		};
		let Some(PendingVoxelLoad::Chunk(pending)) = self.finish(id, VoxelLoadOutcome::Received) else {
			unreachable!("checked chunk voxel load disappeared")
		};
		let generation = generation_for_response(pending.request_generation, &pending.possible_generations, generation);
		handle.loaded(grid, chunk, generation, voxels);
	}

	fn receive_tile_voxel_response(
		&mut self,
		handle: &SourceHandle,
		from: impl std::fmt::Debug,
		id: VoxelLoadId,
		grid: GridId,
		key: TileVoxelKey,
		generation: u64,
		voxel_type: VoxelTypeId,
		compressed: &mut Option<CompressedVoxels>,
	) {
		let Some(pending) = self.pending.get(&id) else { return };
		let PendingVoxelLoad::TileVoxels(pending) = pending else {
			warn!(?id, ?from, "ignoring lod response for a non-lod voxel load");
			return;
		};
		if pending.cancellation.is_cancelled() {
			self.finish(id, VoxelLoadOutcome::Cancelled);
			return;
		}
		if pending.key.grid != grid || pending.key.key != key || pending.key.voxel_type != voxel_type {
			warn!(?id, ?grid, min=?key.min, size=?key.size, lod=key.lod, ?voxel_type, ?from, "ignoring mismatched remote lod response");
			return;
		}

		let voxels = match decompress(compressed) {
			Ok(voxels) => voxels,
			Err(err) => {
				warn!(?id, ?grid, min=?key.min, size=?key.size, lod=key.lod, ?from, error=%err, "failed to decompress remote lod response");
				return;
			}
		};
		let Some(PendingVoxelLoad::TileVoxels(pending)) = self.finish(id, VoxelLoadOutcome::Received) else {
			unreachable!("checked lod voxel load disappeared")
		};
		let generation = generation_for_response(pending.request_generation, &pending.possible_generations, generation);
		handle.loaded_tile_voxels(grid, key.min, key.size, key.lod as f32, voxel_type, generation, voxels);
	}

	fn allocate_id(&mut self) -> VoxelLoadId {
		self.next_id = self.next_id.checked_add(1).expect("voxel load ID space exhausted");
		VoxelLoadId(self.next_id)
	}

	fn finish(&mut self, id: VoxelLoadId, outcome: VoxelLoadOutcome) -> Option<PendingVoxelLoad> {
		let pending = self.pending.remove(&id)?;
		match &pending {
			PendingVoxelLoad::Chunk(pending) if self.chunk_ids.get(&pending.key) == Some(&id) => {
				self.chunk_ids.remove(&pending.key);
			}
			PendingVoxelLoad::TileVoxels(pending) if self.tile_voxel_ids.get(&pending.key) == Some(&id) => {
				self.tile_voxel_ids.remove(&pending.key);
			}
			_ => {}
		}
		self.finished.push_back(VoxelLoadFinished { id, outcome });
		Some(pending)
	}
}

fn decompress(compressed: &mut Option<CompressedVoxels>) -> Result<Option<voxel_data::voxels::Voxels>, voxel_data::compressed_voxels::DecompressVoxelsError> {
	compressed.take().map(|voxels| voxels.decompress()).transpose()
}

fn tile_voxels_overlap_area(key: TileVoxelKey, min: IVec3, size: IVec3) -> bool {
	let max = min + size;
	let key_max = key.min + key.size;
	key.min.cmplt(max).all() && min.cmplt(key_max).all()
}

#[cfg(test)]
mod tests {
	use bevy::prelude::Entity;

	use super::*;

	#[test]
	fn response_uses_the_latest_claimed_generation_it_includes() {
		let possible_generations = BTreeMap::from([(10, 20), (15, 30)]);
		assert_eq!(generation_for_response(5, &possible_generations, 9), 5);
		assert_eq!(generation_for_response(5, &possible_generations, 10), 20);
		assert_eq!(generation_for_response(5, &possible_generations, 14), 20);
		assert_eq!(generation_for_response(5, &possible_generations, 15), 30);
	}

	#[test]
	fn cancelled_chunk_has_one_terminal_outcome() {
		let cancellation = CancellationToken::new();
		let mut loads = ClientLoadRegistry::default();
		loads.request_chunk(Entity::PLACEHOLDER, IVec3::ZERO, 7, cancellation.clone());
		let request = loads.pop_request().unwrap();

		cancellation.cancel();
		loads.drain_cancelled();

		assert_eq!(loads.pop_finished(), Some(VoxelLoadFinished { id: request.id, outcome: VoxelLoadOutcome::Cancelled }));
		assert_eq!(loads.pop_finished(), None);
		assert!(!loads.pending.contains_key(&request.id));
	}

	#[test]
	fn cancelled_tile_voxels_has_one_terminal_outcome() {
		let cancellation = CancellationToken::new();
		let key = TileVoxelKey { min: IVec3::ZERO, size: IVec3::ONE, lod: 1 };
		let mut loads = ClientLoadRegistry::default();
		loads.request_tile_voxels(Entity::PLACEHOLDER, key, VoxelTypeId(1), 0.0, 7, cancellation.clone());
		let request = loads.pop_request().unwrap();

		cancellation.cancel();
		loads.drain_cancelled();

		assert_eq!(loads.pop_finished(), Some(VoxelLoadFinished { id: request.id, outcome: VoxelLoadOutcome::Cancelled }));
		assert_eq!(loads.pop_finished(), None);
		assert!(!loads.pending.contains_key(&request.id));
	}

	#[test]
	fn received_voxel_load_cannot_be_cancelled_afterward() {
		let mut loads = ClientLoadRegistry::default();
		loads.request_chunk(Entity::PLACEHOLDER, IVec3::ZERO, 1, CancellationToken::new());
		let request = loads.pop_request().unwrap();

		assert!(loads.finish(request.id, VoxelLoadOutcome::Received).is_some());
		assert!(loads.finish(request.id, VoxelLoadOutcome::Cancelled).is_none());
		assert_eq!(loads.pop_finished(), Some(VoxelLoadFinished { id: request.id, outcome: VoxelLoadOutcome::Received }));
		assert_eq!(loads.pop_finished(), None);
	}

	#[test]
	fn replacement_cancels_previous_chunk_voxel_load() {
		let mut loads = ClientLoadRegistry::default();
		loads.request_chunk(Entity::PLACEHOLDER, IVec3::ZERO, 1, CancellationToken::new());
		let first = loads.pop_request().unwrap();
		loads.request_chunk(Entity::PLACEHOLDER, IVec3::ZERO, 2, CancellationToken::new());
		let second = loads.pop_request().unwrap();

		assert_ne!(first.id, second.id);
		assert_eq!(loads.pop_finished(), Some(VoxelLoadFinished { id: first.id, outcome: VoxelLoadOutcome::Cancelled }));
		assert!(!loads.pending.contains_key(&first.id));
		assert!(loads.pending.contains_key(&second.id));
	}
}
