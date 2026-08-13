use std::collections::{BTreeMap, HashMap};

use bevy::log::warn;
use bevy::math::IVec3;
use bevy::prelude::*;
use lightyear::prelude::{EventSender, PeerId, PeerMetadata, RemoteEvent};
use voxel_data::grid::GridId;
use voxel_sources::{ChunksEdited, SourceManager};
use tile_data::ChunkRegion;

use super::{EditInterest, EditStreamStart, RemoteGridEdit, WireGridEdit};
use crate::chunks::ServerToClientChannel;

type Area = ChunkRegion;

#[derive(Default)]
struct GridSubscription {
	areas: HashMap<Area, u32>,
	next_edit_index: u64,
	baseline_server_index: u64,
	baseline_client_index: u64,
	server_to_client: BTreeMap<u64, u64>,
}

impl GridSubscription {
	fn overlaps(&self, min: IVec3, size: IVec3) -> bool {
		self.areas.keys().any(|area| overlaps(area.min(), area.size().as_ivec3(), min, size))
	}

	fn snapshot_index(&self, server_index: u64) -> u64 {
		self.server_to_client
			.range(..=server_index)
			.next_back()
			.map(|(_, index)| *index)
			.unwrap_or_else(|| {
				if server_index >= self.baseline_server_index { self.baseline_client_index } else { 0 }
			})
	}
}

#[derive(Resource, Default)]
pub(crate) struct EditSubscriptions {
	clients: HashMap<PeerId, HashMap<GridId, GridSubscription>>,
}

impl EditSubscriptions {
	pub(crate) fn snapshot_index(&self, peer: PeerId, grid: GridId, server_index: u64) -> u64 {
		self.clients
			.get(&peer)
			.and_then(|grids| grids.get(&grid))
			.map_or(0, |subscription| subscription.snapshot_index(server_index))
	}
}

pub(super) fn receive_interest(
	trigger: On<RemoteEvent<EditInterest>>,
	sources: Res<SourceManager>,
	peer_metadata: Option<Res<PeerMetadata>>,
	mut senders: Query<&mut EventSender<EditStreamStart>>,
	mut subscriptions: ResMut<EditSubscriptions>,
) {
	let event = trigger.event();
	let peer = event.from;
	let interest = event.trigger;
	let area = interest.region;
	let subscription = subscriptions.clients.entry(peer).or_default().entry(interest.grid).or_default();
	let was_empty = subscription.areas.is_empty();
	if interest.interested {
		*subscription.areas.entry(area).or_default() += 1;
	} else if let Some(count) = subscription.areas.get_mut(&area) {
		*count = count.saturating_sub(1);
		if *count == 0 { subscription.areas.remove(&area); }
	}
	if !was_empty || subscription.areas.is_empty() { return; }
	subscription.baseline_server_index = sources.current_edit_index(interest.grid);
	subscription.baseline_client_index = subscription.next_edit_index;
	let Some(peer_metadata) = peer_metadata else { return };
	let Some(&entity) = peer_metadata.mapping.get(&peer) else { return };
	let Ok(mut sender) = senders.get_mut(entity) else { return };
	sender.trigger::<ServerToClientChannel>(EditStreamStart {
		grid: interest.grid,
		first_edit_index: subscription.next_edit_index + 1,
	});
}

pub(super) fn flush_edits(
	mut edits: MessageReader<ChunksEdited>,
	peer_metadata: Option<Res<PeerMetadata>>,
	mut senders: Query<&mut EventSender<RemoteGridEdit>>,
	mut subscriptions: ResMut<EditSubscriptions>,
) {
	let Some(peer_metadata) = peer_metadata else { return };
	for event in edits.read() {
		let Some(edit) = WireGridEdit::from_edit(&event.edit) else {
			warn!(grid=?event.grid, edit_index=event.edit_index, "cannot replicate non-serializable voxel edit");
			continue;
		};
		for (&peer, grids) in &mut subscriptions.clients {
			let Some(subscription) = grids.get_mut(&event.grid) else { continue };
			if !subscription.overlaps(event.region.min(), event.region.size().as_ivec3()) { continue; }
			subscription.next_edit_index += 1;
			subscription.server_to_client.insert(event.edit_index, subscription.next_edit_index);
			let Some(&entity) = peer_metadata.mapping.get(&peer) else { continue };
			let Ok(mut sender) = senders.get_mut(entity) else { continue };
			sender.trigger::<ServerToClientChannel>(RemoteGridEdit {
				grid: event.grid,
				region: event.region,
				edit_index: subscription.next_edit_index,
				edit: edit.clone(),
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
