use std::collections::{HashMap, HashSet, VecDeque};

use bevy::log::warn;
use bevy::prelude::*;
use bile_data::streaming::DataConsumingNode;
use lightyear::prelude::PeerId;
use voxel_data::compressed_voxels::CompressedVoxels;
use tile_data::VoxelRegion;
use voxel_sources::{StreamCollector, VoxelData, VoxelRequest, VoxelRequester, edit::VoxelEdit};

use super::super::{
	VoxelLoadManifest,
	VoxelLoadPayload,
	VoxelLoadRequest,
	VoxelLoadRetry,
	VoxelPayloadIndex,
	VoxelPayloadMetadata,
};
use crate::chunks::request_id::NetworkRequestId;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(super) struct VoxelLoadOwner {
	peer: PeerId,
	id: NetworkRequestId,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct LoadId(u64);

#[derive(Clone)]
struct VoxelPayload {
	metadata: VoxelPayloadMetadata,
	voxels: CompressedVoxels,
}

struct PendingVoxelLoad {
	key: VoxelRequest,
	waiting: bool,
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

/// Consumes [`VoxelData`] on behalf of remote clients.
#[derive(Component, Default)]
pub(crate) struct ServerVoxelLoadNode {
	collector: StreamCollector<VoxelRequest, VoxelRegion>,
	received: Vec<(VoxelRequest, Vec<VoxelRegion>)>,
	dirtied: Vec<VoxelRequest>,
}

impl DataConsumingNode<VoxelData> for ServerVoxelLoadNode {
	fn start_data(&mut self, params: &VoxelRequest, tick: u64) { self.collector.start(params, tick); }

	fn push_data(&mut self, params: &VoxelRequest, region: VoxelRegion) { self.collector.push(params, region); }

	fn finish_data(&mut self, params: &VoxelRequest) {
		if let Some(regions) = self.collector.finish(params) {
			self.received.push((*params, regions));
		}
	}

	fn dirty_data(&mut self, params: &VoxelRequest, last_clean_tick: u64, _diff: Option<&VoxelEdit>) {
		self.collector.dirty(params, last_clean_tick);
		self.dirtied.push(*params);
	}
}

impl ServerVoxelLoadNode {
	pub(super) fn take_received(&mut self) -> Vec<(VoxelRequest, Vec<VoxelRegion>)> {
		std::mem::take(&mut self.received)
	}

	pub(super) fn take_dirtied(&mut self) -> Vec<VoxelRequest> {
		std::mem::take(&mut self.dirtied)
	}
}

/// How the loads request and cancel [`VoxelData`] for the [`ServerVoxelLoadNode`].
pub(super) struct NodeVoxels<'a, 'w, 's> {
	pub(super) entity: Entity,
	pub(super) node: &'a mut ServerVoxelLoadNode,
	pub(super) requester: &'a mut VoxelRequester<'w, 's>,
}

impl NodeVoxels<'_, '_, '_> {
	fn request(&mut self, key: VoxelRequest) {
		self.requester.request(key, self.entity);
	}

	fn cancel(&mut self, key: &VoxelRequest) {
		self.node.collector.forget(key);
		self.requester.cancel(key, self.entity);
	}
}

/// Loads are shared by every client requesting the same voxels. Once a finished load is dirtied it
/// keeps serving retries for its owners, while new requests start a fresh load.
#[derive(Resource, Default)]
pub(crate) struct PendingVoxelLoads {
	next_id: u64,
	loads: HashMap<LoadId, PendingVoxelLoad>,
	current: HashMap<VoxelRequest, LoadId>,
	by_owner: HashMap<VoxelLoadOwner, LoadId>,
	cancelled: HashSet<VoxelLoadOwner>,
	outgoing: VecDeque<OutgoingVoxelMessage>,
}

impl PendingVoxelLoads {
	pub(super) fn request(
		&mut self,
		peer: PeerId,
		request: VoxelLoadRequest,
		voxels: &mut NodeVoxels,
	) {
		let owner = VoxelLoadOwner { peer, id: request.id };
		if self.cancelled.remove(&owner) { return; }
		self.remove_owner(owner, voxels);

		let key = VoxelRequest {
			grid: request.key.grid,
			area: request.key.region,
			lod: request.key.lod,
			voxel_type: request.key.voxel_type,
		};
		if let Some(&id) = self.current.get(&key) {
			self.by_owner.insert(owner, id);
			let load = self.loads.get_mut(&id).expect("current voxel load disappeared");
			load.owners.insert(owner);
			if !load.waiting {
				self.queue_transfer(id, owner);
			}
			return;
		}

		let id = LoadId(self.next_id);
		self.next_id += 1;
		self.current.insert(key, id);
		self.by_owner.insert(owner, id);
		self.loads.insert(id, PendingVoxelLoad {
			key,
			waiting: true,
			owners: HashSet::from([owner]),
			payloads: Vec::new(),
		});
		voxels.request(key);
	}

	pub(super) fn cancel(
		&mut self,
		peer: PeerId,
		id: NetworkRequestId,
		voxels: &mut NodeVoxels,
	) {
		let owner = VoxelLoadOwner { peer, id };
		if !self.remove_owner(owner, voxels) {
			self.cancelled.insert(owner);
		}
	}

	pub(super) fn retry(&mut self, peer: PeerId, retry: VoxelLoadRetry) {
		let owner = VoxelLoadOwner { peer, id: retry.id };
		let Some(&id) = self.by_owner.get(&owner) else { return };
		let Some(load) = self.loads.get(&id) else { return };
		if load.waiting || !load.owners.contains(&owner) { return; }
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
		voxels: &mut NodeVoxels,
	) {
		self.remove_owner(VoxelLoadOwner { peer, id }, voxels);
	}

	pub(super) fn receive_voxels(&mut self, key: VoxelRequest, data: &[VoxelRegion], voxels: &mut NodeVoxels) {
		let Some(&id) = self.current.get(&key) else { return };
		if !self.loads[&id].waiting { return; }
		self.complete(id, data, voxels);
	}

	pub(super) fn dirty_voxels(&mut self, key: VoxelRequest, voxels: &mut NodeVoxels) {
		let Some(&id) = self.current.get(&key) else { return };
		if self.loads[&id].waiting {
			// What was streamed so far is stale, the voxels are streamed again.
			voxels.request(key);
			return;
		}
		self.current.remove(&key);
		voxels.cancel(&key);
	}

	pub(super) fn remove_disconnected(
		&mut self,
		mut connected: impl FnMut(PeerId) -> bool,
		voxels: &mut NodeVoxels,
	) {
		let disconnected: Vec<_> = self.by_owner.keys()
			.filter(|owner| !connected(owner.peer))
			.copied()
			.collect();
		for owner in disconnected {
			self.remove_owner(owner, voxels);
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

	fn complete(&mut self, id: LoadId, data: &[VoxelRegion], voxels: &mut NodeVoxels) {
		let Some(key) = self.loads.get(&id).map(|load| load.key) else { return };
		let payloads: Vec<_> = data.iter().filter_map(|result| {
			if !key.area.contains_region(result.area) || result.lod > key.lod {
				warn!(expected_grid=?key.grid, expected_region=?key.area, expected_lod=key.lod, region=?result.area, lod=result.lod, "ignoring mismatched source voxel payload");
				return None;
			}
			let compressed = match CompressedVoxels::with_max_compression(&result.voxels) {
				Ok(compressed) => compressed,
				Err(err) => {
					warn!(grid=?key.grid, region=?result.area, lod=result.lod, error=%err, "failed to compress source voxel payload");
					return None;
				}
			};
			Some(VoxelPayload {
				metadata: VoxelPayloadMetadata {
					region: result.area,
					lod: result.lod,
					compressed_bytes: u64::try_from(compressed.len()).expect("compressed payload length exceeded protocol range"),
				},
				voxels: compressed,
			})
		}).collect();
		if payloads.len() > u32::MAX as usize {
			warn!(grid=?key.grid, region=?key.area, "source produced too many voxel payloads for the network protocol");
			let load = self.loads.remove(&id).expect("oversized voxel load disappeared");
			for owner in load.owners {
				self.by_owner.remove(&owner);
			}
			self.current.remove(&key);
			voxels.cancel(&key);
			return;
		}
		let load = self.loads.get_mut(&id).expect("completed voxel load disappeared");
		load.waiting = false;
		load.payloads = payloads;
		let owners = load.owners.iter().copied().collect::<Vec<_>>();
		for owner in owners {
			self.queue_transfer(id, owner);
		}
	}

	fn queue_transfer(&mut self, id: LoadId, owner: VoxelLoadOwner) {
		let Some(load) = self.loads.get(&id) else { return };
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

	fn remove_owner(&mut self, owner: VoxelLoadOwner, voxels: &mut NodeVoxels) -> bool {
		self.outgoing.retain(|message| message.owner() != owner);
		let Some(id) = self.by_owner.remove(&owner) else { return false };
		let should_remove = self.loads.get_mut(&id).is_some_and(|load| {
			load.owners.remove(&owner);
			load.owners.is_empty()
		});
		if should_remove {
			let load = self.loads.remove(&id).expect("empty voxel load disappeared");
			if self.current.get(&load.key) == Some(&id) {
				self.current.remove(&load.key);
				voxels.cancel(&load.key);
			}
		}
		true
	}
}
