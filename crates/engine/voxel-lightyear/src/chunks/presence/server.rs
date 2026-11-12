use std::collections::{HashMap, HashSet, VecDeque};

use bevy::log::warn;
use bevy::prelude::*;
use bile_data::DataConsumingNode;
use lightyear::prelude::{EventSender, PeerId, NetworkingMetadata, RemoteEvent};
use voxel_data::grid::GridId;
use voxel_sources::{PresenceRegions, PresenceRequester, VoxelPresence};

use super::{PresenceCancel, PresenceLoad, PresenceLoaded, PresenceRequest};
use crate::ReplicateVoxelsRestriction;
use crate::chunks::request_id::NetworkRequestId;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct PresenceOwner {
	peer: PeerId,
	id: NetworkRequestId,
}

enum OutgoingPresenceMessage {
	Load { owner: PresenceOwner, load: PresenceLoad },
	Loaded { owner: PresenceOwner, loaded: PresenceLoaded },
}

impl OutgoingPresenceMessage {
	fn owner(&self) -> PresenceOwner {
		match self {
			Self::Load { owner, .. } | Self::Loaded { owner, .. } => *owner,
		}
	}
}

/// Consumes [`VoxelPresence`] on behalf of remote clients.
#[derive(Component, Default)]
pub(super) struct ServerPresenceNode {
	received: Vec<(GridId, PresenceRegions)>,
}

impl DataConsumingNode<VoxelPresence> for ServerPresenceNode {
	fn pass_data(&mut self, grid: &GridId, _tick: u64, regions: PresenceRegions) {
		self.received.push((*grid, regions));
	}

	fn dirty_data(&mut self, _grid: &GridId, _last_clean_tick: u64) {}
}

#[derive(Resource, Default)]
pub(super) struct PendingPresenceLoads {
	owners_by_grid: HashMap<GridId, HashSet<PresenceOwner>>,
	by_owner: HashMap<PresenceOwner, GridId>,
	ready: Vec<(GridId, PresenceRegions)>,
	cancelled: HashSet<PresenceOwner>,
	outgoing: VecDeque<OutgoingPresenceMessage>,
}

impl PendingPresenceLoads {
	fn request(&mut self, peer: PeerId, request: PresenceRequest, node: Entity, requester: &mut PresenceRequester) {
		let owner = PresenceOwner { peer, id: request.id };
		if self.cancelled.remove(&owner) { return; }
		self.remove_owner(owner, node, requester);
		self.by_owner.insert(owner, request.grid);
		let owners = self.owners_by_grid.entry(request.grid).or_default();
		let first = owners.is_empty();
		owners.insert(owner);
		if first && let Some(regions) = requester.request(request.grid, node) {
			self.ready.push((request.grid, regions));
		}
	}

	fn cancel(&mut self, peer: PeerId, id: NetworkRequestId, node: Entity, requester: &mut PresenceRequester) {
		let owner = PresenceOwner { peer, id };
		if !self.remove_owner(owner, node, requester) {
			self.cancelled.insert(owner);
		}
	}

	fn remove_owner(&mut self, owner: PresenceOwner, node: Entity, requester: &mut PresenceRequester) -> bool {
		self.outgoing.retain(|message| message.owner() != owner);
		let Some(grid) = self.by_owner.remove(&owner) else { return false };
		if let Some(owners) = self.owners_by_grid.get_mut(&grid) {
			owners.remove(&owner);
			if owners.is_empty() {
				self.owners_by_grid.remove(&grid);
				requester.cancel(grid, node);
			}
		}
		true
	}

	fn remove_disconnected(&mut self, connected: impl Fn(PeerId) -> bool, node: Entity, requester: &mut PresenceRequester) {
		let disconnected: Vec<_> = self.by_owner.keys()
			.filter(|owner| !connected(owner.peer))
			.copied()
			.collect();
		for owner in disconnected {
			self.remove_owner(owner, node, requester);
		}
		self.cancelled.retain(|owner| connected(owner.peer));
		self.outgoing.retain(|message| connected(message.owner().peer));
	}

	fn deliver(
		&mut self,
		grid: GridId,
		regions: &PresenceRegions,
		restriction: Option<&ReplicateVoxelsRestriction>,
		node: Entity,
		requester: &mut PresenceRequester,
	) {
		let Some(owners) = self.owners_by_grid.remove(&grid) else { return };
		requester.cancel(grid, node);
		for owner in owners {
			self.by_owner.remove(&owner);
			for &region in regions.iter() {
				let region = if let Some(readable) = restriction.and_then(|restriction| restriction.readable_aabb(owner.peer)) {
					region.intersection(readable)
				} else if restriction.is_some() {
					None
				} else {
					Some(region)
				};
				if let Some(region) = region {
					self.outgoing.push_back(OutgoingPresenceMessage::Load {
						owner,
						load: PresenceLoad { id: owner.id, grid, region },
					});
				}
			}
			self.outgoing.push_back(OutgoingPresenceMessage::Loaded {
				owner,
				loaded: PresenceLoaded { id: owner.id },
			});
		}
	}
}

pub(super) fn receive_request(
	trigger: On<RemoteEvent<PresenceRequest>>,
	node: Single<Entity, With<ServerPresenceNode>>,
	mut requester: PresenceRequester,
	mut pending: ResMut<PendingPresenceLoads>,
) {
	let event = trigger.event();
	pending.request(event.from, event.trigger, *node, &mut requester);
}

pub(super) fn receive_cancel(
	trigger: On<RemoteEvent<PresenceCancel>>,
	node: Single<Entity, With<ServerPresenceNode>>,
	mut requester: PresenceRequester,
	mut pending: ResMut<PendingPresenceLoads>,
) {
	let event = trigger.event();
	pending.cancel(event.from, event.trigger.id, *node, &mut requester);
}

pub(super) fn flush_presence(
	node: Single<(Entity, &mut ServerPresenceNode)>,
	grids: Query<Option<&ReplicateVoxelsRestriction>>,
	mut requester: PresenceRequester,
	mut pending: ResMut<PendingPresenceLoads>,
) {
	let (entity, mut node) = node.into_inner();
	let mut ready = std::mem::take(&mut node.received);
	ready.append(&mut pending.ready);
	for (grid, regions) in ready {
		let Ok(restriction) = grids.get(grid) else {
			warn!(?grid, "ignoring source presence for a missing grid");
			continue;
		};
		pending.deliver(grid, &regions, restriction, entity, &mut requester);
	}
}

pub(super) fn send_pending_presence(
	peer_metadata: Option<Res<NetworkingMetadata>>,
	mut senders: Query<(&mut EventSender<PresenceLoad>, &mut EventSender<PresenceLoaded>)>,
	mut pending: ResMut<PendingPresenceLoads>,
) {
	let Some(peer_metadata) = peer_metadata else { return };
	for _ in 0..pending.outgoing.len() {
		let Some(message) = pending.outgoing.pop_front() else { break };
		let owner = message.owner();
		let Some(&entity) = peer_metadata.peer_map.get(&owner.peer) else {
			pending.outgoing.push_back(message);
			continue;
		};
		let Ok((mut load_sender, mut loaded_sender)) = senders.get_mut(entity) else {
			pending.outgoing.push_back(message);
			continue;
		};
		match message {
			OutgoingPresenceMessage::Load { load, .. } => {
				load_sender.trigger::<crate::chunks::ServerToClientChannel>(load);
			}
			OutgoingPresenceMessage::Loaded { loaded, .. } => {
				loaded_sender.trigger::<crate::chunks::ServerToClientChannel>(loaded);
			}
		}
	}
}

pub(super) fn cleanup_disconnected(
	peer_metadata: Option<Res<NetworkingMetadata>>,
	node: Single<Entity, With<ServerPresenceNode>>,
	mut requester: PresenceRequester,
	mut pending: ResMut<PendingPresenceLoads>,
) {
	let Some(peer_metadata) = peer_metadata else { return };
	pending.remove_disconnected(|peer| peer_metadata.peer_map.contains_key(&peer), *node, &mut requester);
}
