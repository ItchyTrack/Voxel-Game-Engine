use std::collections::HashMap;

use bevy::math::IVec3;
use bevy::prelude::*;
use lightyear::prelude::{EventSender, PeerId, PeerMetadata, RemoteEvent};
use voxel_data::grid::GridId;
use tile_data::{ChunkRegion, chunks_covering_nonzero_voxel_region};
use voxel_sources::edit::GridEditMessage;

use super::{EditInterest, EditStreamStart, RemoteGridEdit};
use crate::chunks::ServerToClientChannel;

type Area = ChunkRegion;

#[derive(Default)]
struct GridSubscription {
	areas: HashMap<Area, ()>,
	area_versions: HashMap<Area, u64>,
	next_stream_sequence: u64,
}

impl GridSubscription {
	fn overlaps(&self, min: IVec3, size: IVec3) -> bool {
		self.areas.keys().any(|area| overlaps(area.min(), area.size().as_ivec3(), min, size))
	}
}

#[derive(Resource, Default)]
pub(crate) struct EditSubscriptions {
	clients: HashMap<PeerId, HashMap<GridId, GridSubscription>>,
}


pub(super) fn receive_interest(
	trigger: On<RemoteEvent<EditInterest>>,
	peer_metadata: Option<Res<PeerMetadata>>,
	mut senders: Query<&mut EventSender<EditStreamStart>>,
	mut subscriptions: ResMut<EditSubscriptions>,
) {
	let event = trigger.event();
	let peer = event.from;
	let interest = event.trigger;
	let area: Area = interest.region.into();
	let subscription = subscriptions.clients.entry(peer).or_default().entry(interest.grid).or_default();
	let was_empty = subscription.areas.is_empty();
	let previous_version = subscription.area_versions.get(&area).copied();
	if previous_version.is_some_and(|previous| interest.version <= previous) { return; }
	subscription.area_versions.insert(area, interest.version);
	if interest.interested {
		subscription.areas.insert(area, ());
	} else {
		subscription.areas.remove(&area);
	}
	if !was_empty || subscription.areas.is_empty() { return; }
	let Some(peer_metadata) = peer_metadata else { return };
	let Some(&entity) = peer_metadata.mapping.get(&peer) else { return };
	let Ok(mut sender) = senders.get_mut(entity) else { return };
	sender.trigger::<ServerToClientChannel>(EditStreamStart {
		grid: interest.grid,
		first_stream_sequence: subscription.next_stream_sequence + 1,
	});
}

pub(super) fn flush_edits(
	mut edits: MessageReader<GridEditMessage>,
	peer_metadata: Option<Res<PeerMetadata>>,
	mut senders: Query<&mut EventSender<RemoteGridEdit>>,
	mut subscriptions: ResMut<EditSubscriptions>,
) {
	let Some(peer_metadata) = peer_metadata else { return };
	for event in edits.read() {
		let affected_region = chunks_covering_nonzero_voxel_region(event.edit().affected_region());
		for (&peer, grids) in &mut subscriptions.clients {
			let Some(subscription) = grids.get_mut(&event.grid_id()) else { continue };
			if !subscription.overlaps(affected_region.min(), region.size().as_ivec3()) { continue; }
			subscription.next_stream_sequence += 1;
			let Some(&entity) = peer_metadata.mapping.get(&peer) else { continue };
			let Ok(mut sender) = senders.get_mut(entity) else { continue };
			sender.trigger::<ServerToClientChannel>(RemoteGridEdit {
				grid: event.grid_id(),
				stream_sequence: subscription.next_stream_sequence,
				generation: event.generation,
				edit: postcard::to_vec(event.edit()).expect("This should work...").as_slice().into(),
			});
		}
	}
}

pub(super) fn cleanup_disconnected(
	peer_metadata: Option<Res<PeerMetadata>>,
	mut subscriptions: ResMut<EditSubscriptions>,
) {
	let Some(peer_metadata) = peer_metadata else { return };
	subscriptions.clients.retain(|peer, _| peer_metadata.mapping.contains_key(peer));
}

fn overlaps(min_a: IVec3, size_a: IVec3, min_b: IVec3, size_b: IVec3) -> bool {
	min_a.cmplt(min_b + size_b).all() && min_b.cmplt(min_a + size_a).all()
}
