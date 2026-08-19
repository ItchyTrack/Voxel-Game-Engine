use std::collections::{HashMap, HashSet, VecDeque};

use bevy::ecs::message::MessageReader;
use bevy::log::warn;
use bevy::prelude::*;
use lightyear::prelude::{EventSender, PeerId, PeerMetadata, RemoteEvent};
use voxel_data::grid::GridId;
use voxel_sources::{RequestId, SourceManager, SourceResult, SourceResultData};

use super::{PresenceCancel, PresenceLoad, PresenceLoaded, PresenceRequest};
use crate::ReplicateVoxelsRestriction;
use crate::chunks::request_id::NetworkRequestId;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct PresenceOwner {
	peer: PeerId,
	id: NetworkRequestId,
}

struct PendingPresence {
	owner: PresenceOwner,
	grid: GridId,
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

#[derive(Resource, Default)]
pub(super) struct PendingPresenceLoads {
	by_source_request: HashMap<RequestId, PendingPresence>,
	by_owner: HashMap<PresenceOwner, RequestId>,
	cancelled: HashSet<PresenceOwner>,
	outgoing: VecDeque<OutgoingPresenceMessage>,
}

impl PendingPresenceLoads {
	fn request(&mut self, peer: PeerId, request: PresenceRequest, sources: &mut SourceManager) {
		let owner = PresenceOwner { peer, id: request.id };
		if self.cancelled.remove(&owner) { return; }
		self.remove_owner(owner, sources);
		let request_id = sources.request_presence(request.grid);
		self.by_owner.insert(owner, request_id);
		self.by_source_request.insert(request_id, PendingPresence { owner, grid: request.grid });
	}

	fn cancel(&mut self, peer: PeerId, id: NetworkRequestId, sources: &mut SourceManager) {
		let owner = PresenceOwner { peer, id };
		if !self.remove_owner(owner, sources) {
			self.cancelled.insert(owner);
		}
	}

	fn remove_owner(&mut self, owner: PresenceOwner, sources: &mut SourceManager) -> bool {
		self.outgoing.retain(|message| message.owner() != owner);
		let Some(request_id) = self.by_owner.remove(&owner) else { return false };
		self.by_source_request.remove(&request_id);
		sources.cancel_presence(request_id);
		true
	}

	fn remove_disconnected(&mut self, connected: impl Fn(PeerId) -> bool, sources: &mut SourceManager) {
		let disconnected: Vec<_> = self.by_owner.keys()
			.filter(|owner| !connected(owner.peer))
			.copied()
			.collect();
		for owner in disconnected {
			self.remove_owner(owner, sources);
		}
		self.cancelled.retain(|owner| connected(owner.peer));
		self.outgoing.retain(|message| connected(message.owner().peer));
	}
}

pub(super) fn receive_request(
	trigger: On<RemoteEvent<PresenceRequest>>,
	mut sources: ResMut<SourceManager>,
	mut pending: ResMut<PendingPresenceLoads>,
) {
	let event = trigger.event();
	pending.request(event.from, event.trigger, &mut sources);
}

pub(super) fn receive_cancel(
	trigger: On<RemoteEvent<PresenceCancel>>,
	mut sources: ResMut<SourceManager>,
	mut pending: ResMut<PendingPresenceLoads>,
) {
	let event = trigger.event();
	pending.cancel(event.from, event.trigger.id, &mut sources);
}

pub(super) fn flush_source_results(
	mut results: MessageReader<SourceResult>,
	grids: Query<Option<&ReplicateVoxelsRestriction>>,
	mut pending: ResMut<PendingPresenceLoads>,
) {
	for result in results.read() {
		let Some(request) = pending.by_source_request.get(&result.request_id) else { continue };
		let owner = request.owner;
		let expected_grid = request.grid;
		match &result.data {
			SourceResultData::Presence { grid, region } => {
				if expected_grid != *grid {
					warn!(request_id=?result.request_id, expected=?expected_grid, actual=?grid, "ignoring mismatched source presence payload");
					continue;
				}
				let Ok(restriction) = grids.get(*grid) else {
					warn!(request_id=?result.request_id, ?grid, "ignoring source presence for a missing grid");
					continue;
				};
				let region = if let Some((min, size)) = restriction.and_then(|restriction| restriction.readable_aabb(owner.peer)) {
					let readable = tile_data::NonZeroChunkRegion::from_min_size(min, size)
						.expect("non-empty readable presence had an empty bounding box");
					region.intersection(readable)
				} else if restriction.is_some() {
					None
				} else {
					Some(*region)
				};
				if let Some(region) = region {
					pending.outgoing.push_back(OutgoingPresenceMessage::Load {
						owner,
						load: PresenceLoad { id: owner.id, grid: *grid, region },
					});
				}
			}
			SourceResultData::PresenceLoaded => {
				let request = pending.by_source_request.remove(&result.request_id)
					.expect("routed presence request disappeared");
				pending.by_owner.remove(&request.owner);
				pending.outgoing.push_back(OutgoingPresenceMessage::Loaded {
					owner: request.owner,
					loaded: PresenceLoaded { id: request.owner.id },
				});
			}
			SourceResultData::Voxels { .. } | SourceResultData::VoxelsLoaded => {}
		}
	}
}

pub(super) fn send_pending_presence(
	peer_metadata: Option<Res<PeerMetadata>>,
	mut senders: Query<(&mut EventSender<PresenceLoad>, &mut EventSender<PresenceLoaded>)>,
	mut pending: ResMut<PendingPresenceLoads>,
) {
	let Some(peer_metadata) = peer_metadata else { return };
	for _ in 0..pending.outgoing.len() {
		let Some(message) = pending.outgoing.pop_front() else { break };
		let owner = message.owner();
		let Some(&entity) = peer_metadata.mapping.get(&owner.peer) else {
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
	peer_metadata: Option<Res<PeerMetadata>>,
	mut sources: ResMut<SourceManager>,
	mut pending: ResMut<PendingPresenceLoads>,
) {
	let Some(peer_metadata) = peer_metadata else { return };
	pending.remove_disconnected(|peer| peer_metadata.mapping.contains_key(&peer), &mut sources);
}
