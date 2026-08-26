use std::collections::{HashMap, HashSet, VecDeque};

use bevy::log::warn;
use bevy::prelude::*;
use lightyear::prelude::PeerId;
use voxel_data::compressed_voxels::CompressedVoxels;
use voxel_sources::{RequestId, SourceManager, SourceResult, SourceResultData, edit::GridGeneration};

use super::super::{
	VoxelLoadManifest,
	VoxelLoadPayload,
	VoxelLoadRequest,
	VoxelLoadRetry,
	VoxelPayloadIndex,
	VoxelPayloadMetadata,
	VoxelRequestKey,
};
use crate::chunks::request_id::NetworkRequestId;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(super) struct VoxelLoadOwner {
	peer: PeerId,
	id: NetworkRequestId,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct SourceVoxelRequestKey {
	remote: VoxelRequestKey,
	server_generation: GridGeneration,
}

#[derive(Clone)]
struct VoxelPayload {
	metadata: VoxelPayloadMetadata,
	voxels: CompressedVoxels,
}

struct PendingVoxelLoad {
	source_request: Option<RequestId>,
	owners: HashSet<VoxelLoadOwner>,
	payloads: Vec<VoxelPayload>,
}

pub(super) enum OutgoingVoxelMessage {
	Manifest { owner: VoxelLoadOwner, manifest: VoxelLoadManifest },
	Payload { owner: VoxelLoadOwner, payload: VoxelLoadPayload },
}

impl OutgoingVoxelMessage {
	pub(super) fn peer(&self) -> PeerId {
		match self {
			Self::Manifest { owner, .. } | Self::Payload { owner, .. } => owner.peer,
		}
	}

	fn owner(&self) -> VoxelLoadOwner {
		match self {
			Self::Manifest { owner, .. } | Self::Payload { owner, .. } => *owner,
		}
	}
}

#[derive(Resource, Default)]
pub(crate) struct PendingVoxelLoads {
	loads: HashMap<SourceVoxelRequestKey, PendingVoxelLoad>,
	keys_by_source_request: HashMap<RequestId, SourceVoxelRequestKey>,
	keys_by_owner: HashMap<VoxelLoadOwner, SourceVoxelRequestKey>,
	cancelled: HashSet<VoxelLoadOwner>,
	outgoing: VecDeque<OutgoingVoxelMessage>,
}

impl PendingVoxelLoads {
	pub(super) fn request(
		&mut self,
		peer: PeerId,
		request: VoxelLoadRequest,
		server_generation: GridGeneration,
		sources: &mut SourceManager,
	) {
		let owner = VoxelLoadOwner { peer, id: request.id };
		if self.cancelled.remove(&owner) { return; }
		self.remove_owner(owner, sources);

		let key = SourceVoxelRequestKey { remote: request.key, server_generation };
		self.keys_by_owner.insert(owner, key);
		if let Some(load) = self.loads.get_mut(&key) {
			load.owners.insert(owner);
			if load.source_request.is_none() {
				self.queue_transfer(key, owner);
			}
			return;
		}

		let request_id = sources.request_voxels(
			key.remote.grid,
			key.remote.region,
			key.remote.lod,
			key.remote.voxel_type,
			key.server_generation,
		);
		self.keys_by_source_request.insert(request_id, key);
		self.loads.insert(key, PendingVoxelLoad {
			source_request: Some(request_id),
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

	pub(super) fn retry(&mut self, peer: PeerId, retry: VoxelLoadRetry) {
		let owner = VoxelLoadOwner { peer, id: retry.id };
		let Some(&key) = self.keys_by_owner.get(&owner) else { return };
		let Some(load) = self.loads.get(&key) else { return };
		if load.source_request.is_some() || !load.owners.contains(&owner) { return; }
		let payloads = retry.missing.iter()
			.filter_map(|index| {
				let payload = load.payloads.get(usize::try_from(index.0).ok()?)?;
				Some(Self::payload_message(owner, *index, payload))
			})
			.collect::<Vec<_>>();
		self.outgoing.extend(payloads);
	}

	pub(super) fn received(
		&mut self,
		peer: PeerId,
		id: NetworkRequestId,
		sources: &mut SourceManager,
	) {
		self.remove_owner(VoxelLoadOwner { peer, id }, sources);
	}

	pub(super) fn receive_source_result(&mut self, result: &SourceResult) {
		let Some(&key) = self.keys_by_source_request.get(&result.request_id) else { return };
		match &result.data {
			SourceResultData::Voxels { grid, region, lod, voxels, .. } => {
				if *grid != key.remote.grid
					|| !key.remote.region.contains_region(*region)
					|| *lod > key.remote.lod
				{
					warn!(request_id=?result.request_id, expected_grid=?key.remote.grid, expected_region=?key.remote.region, expected_lod=key.remote.lod, ?grid, ?region, ?lod, "ignoring mismatched source voxel payload");
					return;
				}
				let compressed = match CompressedVoxels::with_max_compression(voxels) {
					Ok(compressed) => compressed,
					Err(err) => {
						warn!(request_id=?result.request_id, ?grid, ?region, ?lod, error=%err, "failed to compress source voxel payload");
						return;
					}
				};
				let Some(load) = self.loads.get_mut(&key) else { return };
				load.payloads.push(VoxelPayload {
					metadata: VoxelPayloadMetadata {
						region: *region,
						lod: *lod,
						compressed_bytes: u64::try_from(compressed.len()).expect("compressed payload length exceeded protocol range"),
					},
					voxels: compressed,
				});
			}
			SourceResultData::VoxelsLoaded { .. } => self.complete(key),
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

	fn complete(&mut self, key: SourceVoxelRequestKey) {
		let Some(load) = self.loads.get_mut(&key) else { return };
		let Some(request_id) = load.source_request.take() else { return };
		self.keys_by_source_request.remove(&request_id);
		if load.payloads.len() > u32::MAX as usize {
			warn!(?request_id, "source produced too many voxel payloads for the network protocol");
			let load = self.loads.remove(&key).expect("oversized voxel load disappeared");
			for owner in load.owners {
				self.keys_by_owner.remove(&owner);
			}
			return;
		}
		let owners = load.owners.iter().copied().collect::<Vec<_>>();
		for owner in owners {
			self.queue_transfer(key, owner);
		}
	}

	fn queue_transfer(&mut self, key: SourceVoxelRequestKey, owner: VoxelLoadOwner) {
		let Some(load) = self.loads.get(&key) else { return };
		let metadata = load.payloads.iter().map(|payload| payload.metadata).collect::<Vec<_>>();
		let payloads = load.payloads.iter().enumerate()
			.map(|(index, payload)| {
				let index = VoxelPayloadIndex(u32::try_from(index).expect("voxel payload index exceeded protocol range"));
				Self::payload_message(owner, index, payload)
			})
			.collect::<Vec<_>>();
		self.outgoing.push_back(OutgoingVoxelMessage::Manifest {
			owner,
			manifest: VoxelLoadManifest { id: owner.id, payloads: metadata.into_boxed_slice() },
		});
		self.outgoing.extend(payloads);
	}

	fn payload_message(owner: VoxelLoadOwner, index: VoxelPayloadIndex, payload: &VoxelPayload) -> OutgoingVoxelMessage {
		OutgoingVoxelMessage::Payload {
			owner,
			payload: VoxelLoadPayload { id: owner.id, index, voxels: payload.voxels.clone() },
		}
	}

	fn remove_owner(&mut self, owner: VoxelLoadOwner, sources: &mut SourceManager) -> bool {
		let Some(key) = self.keys_by_owner.remove(&owner) else {
			self.outgoing.retain(|message| message.owner() != owner);
			return false;
		};
		self.outgoing.retain(|message| message.owner() != owner);
		let should_remove = self.loads.get_mut(&key).is_some_and(|load| {
			load.owners.remove(&owner);
			load.owners.is_empty()
		});
		if should_remove {
			let load = self.loads.remove(&key).expect("empty voxel load disappeared");
			if let Some(request_id) = load.source_request {
				self.keys_by_source_request.remove(&request_id);
				sources.cancel_voxels(request_id);
			}
		}
		true
	}
}
