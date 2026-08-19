use std::collections::{HashMap, HashSet, VecDeque};

use bevy::log::warn;
use bevy::prelude::*;
use lightyear::prelude::PeerId;
use tile_data::NonZeroChunkRegion;
use voxel_data::compressed_voxels::CompressedVoxels;
use voxel_data::grid::GridId;
use voxel_sources::{RequestId, SourceManager, SourceResult, SourceResultData};

use super::super::{VoxelLoadComplete, VoxelLoadPayload, VoxelLoadRequest, VoxelRequestKey};
use crate::chunks::request_id::NetworkRequestId;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct VoxelLoadOwner {
	peer: PeerId,
	id: NetworkRequestId,
}

#[derive(Clone)]
struct VoxelPayload {
	grid: GridId,
	region: NonZeroChunkRegion,
	lod: u8,
	generation: u64,
	voxels: CompressedVoxels,
}

struct PendingVoxelLoad {
	request_id: RequestId,
	owners: HashSet<VoxelLoadOwner>,
	payloads: Vec<VoxelPayload>,
}

pub(super) enum OutgoingVoxelMessage {
	Payload { peer: PeerId, payload: VoxelLoadPayload },
	Complete { peer: PeerId, complete: VoxelLoadComplete },
}

impl OutgoingVoxelMessage {
	pub(super) fn peer(&self) -> PeerId {
		match self {
			Self::Payload { peer, .. } | Self::Complete { peer, .. } => *peer,
		}
	}

	fn owner(&self) -> VoxelLoadOwner {
		match self {
			Self::Payload { peer, payload } => VoxelLoadOwner { peer: *peer, id: payload.id },
			Self::Complete { peer, complete } => VoxelLoadOwner { peer: *peer, id: complete.id },
		}
	}
}

#[derive(Resource, Default)]
pub(crate) struct PendingVoxelLoads {
	loads: HashMap<VoxelRequestKey, PendingVoxelLoad>,
	keys_by_source_request: HashMap<RequestId, VoxelRequestKey>,
	keys_by_owner: HashMap<VoxelLoadOwner, VoxelRequestKey>,
	cancelled: HashSet<VoxelLoadOwner>,
	outgoing: VecDeque<OutgoingVoxelMessage>,
}

impl PendingVoxelLoads {
	pub(super) fn request(
		&mut self,
		peer: PeerId,
		request: VoxelLoadRequest,
		sources: &mut SourceManager,
	) {
		let owner = VoxelLoadOwner { peer, id: request.id };
		if self.cancelled.remove(&owner) { return; }
		self.remove_owner(owner, sources);

		let key = request.key;
		self.keys_by_owner.insert(owner, key);
		if let Some(load) = self.loads.get_mut(&key) {
			load.owners.insert(owner);
			return;
		}

		let request_id = sources.request_voxels(key.grid, key.region, key.lod, key.voxel_type);
		self.keys_by_source_request.insert(request_id, key);
		self.loads.insert(key, PendingVoxelLoad {
			request_id,
			owners: HashSet::from([owner]),
			payloads: Vec::new(),
		});
	}

	pub(super) fn cancel(
		&mut self,
		peer: PeerId,
		id: NetworkRequestId,
		sources: &mut SourceManager,
	) {
		let owner = VoxelLoadOwner { peer, id };
		if !self.remove_owner(owner, sources) {
			self.cancelled.insert(owner);
		}
	}

	pub(super) fn receive_source_result(&mut self, result: &SourceResult) {
		let Some(&key) = self.keys_by_source_request.get(&result.request_id) else { return };
		match &result.data {
			SourceResultData::Voxels { grid, region, lod, generation, voxels } => {
				let compressed = match CompressedVoxels::with_max_compression(voxels) {
					Ok(compressed) => compressed,
					Err(err) => {
						warn!(request_id=?result.request_id, ?grid, ?region, ?lod, error=%err, "failed to compress source voxel payload");
						return;
					}
				};
				let Some(load) = self.loads.get_mut(&key) else { return };
				load.payloads.push(VoxelPayload {
					grid: *grid,
					region: *region,
					lod: *lod,
					generation: *generation,
					voxels: compressed,
				});
			}
			SourceResultData::VoxelsLoaded => self.complete(key),
			SourceResultData::Presence { .. } | SourceResultData::PresenceLoaded => {}
		}
	}

	pub(super) fn remove_disconnected(
		&mut self,
		mut connected: impl FnMut(PeerId) -> bool,
		sources: &mut SourceManager,
	) {
		let disconnected: Vec<_> = self.keys_by_owner.keys()
			.filter(|owner| !connected(owner.peer))
			.copied()
			.collect();
		for owner in disconnected {
			self.remove_owner(owner, sources);
		}
		self.cancelled.retain(|owner| connected(owner.peer));
		self.outgoing.retain(|message| connected(message.peer()));
	}

	pub(super) fn outgoing_len(&self) -> usize { self.outgoing.len() }

	pub(super) fn pop_outgoing(&mut self) -> Option<OutgoingVoxelMessage> {
		self.outgoing.pop_front()
	}

	pub(super) fn push_outgoing(&mut self, message: OutgoingVoxelMessage) {
		self.outgoing.push_back(message);
	}

	fn complete(&mut self, key: VoxelRequestKey) {
		let Some(load) = self.loads.remove(&key) else { return };
		self.keys_by_source_request.remove(&load.request_id);
		let Ok(sent_payload_count) = u32::try_from(load.payloads.len()) else {
			warn!(request_id=?load.request_id, "source produced too many voxel payloads for the network protocol");
			for owner in load.owners {
				self.keys_by_owner.remove(&owner);
			}
			return;
		};
		for owner in load.owners {
			self.keys_by_owner.remove(&owner);
			for payload in &load.payloads {
				self.outgoing.push_back(OutgoingVoxelMessage::Payload {
					peer: owner.peer,
					payload: VoxelLoadPayload {
						id: owner.id,
						grid: payload.grid,
						region: payload.region,
						lod: payload.lod,
						generation: payload.generation,
						voxels: payload.voxels.clone(),
					},
				});
			}
			self.outgoing.push_back(OutgoingVoxelMessage::Complete {
				peer: owner.peer,
				complete: VoxelLoadComplete { id: owner.id, sent_payload_count },
			});
		}
	}

	fn remove_owner(&mut self, owner: VoxelLoadOwner, sources: &mut SourceManager) -> bool {
		let Some(key) = self.keys_by_owner.remove(&owner) else {
			self.outgoing.retain(|message| message.owner() != owner);
			return false;
		};
		self.outgoing.retain(|message| message.owner() != owner);
		let should_cancel = self.loads.get_mut(&key).is_some_and(|load| {
			load.owners.remove(&owner);
			load.owners.is_empty()
		});
		if should_cancel {
			let load = self.loads.remove(&key).expect("empty voxel load disappeared");
			self.keys_by_source_request.remove(&load.request_id);
			sources.cancel_voxels(load.request_id);
		}
		true
	}
}
