use std::collections::HashMap;

use bevy::prelude::*;
use lightyear::prelude::{EventSender, PeerId, NetworkingMetadata, RemoteEvent};
use tile_data::chunk_aligned;
use voxel_data::grid::GridId;
use voxel_sources::edit::{GridEditIdManager, GridEditMessage};
use voxel_sources::ChunkEditInterest;

use super::{EditInterest, RemoteGridEdit};
use crate::chunks::ServerToClientChannel;

#[derive(Default)]
struct GridSubscription {
	interest: ChunkEditInterest,
	remote_edits: GridEditIdManager,
}

impl GridSubscription {
	fn remap_edit(&mut self) -> voxel_sources::edit::GridEditId {
		self.remote_edits.apply_edit().0
	}
}

#[derive(Resource, Default)]
pub(crate) struct EditSubscriptions {
	clients: HashMap<PeerId, HashMap<GridId, GridSubscription>>,
}

pub(super) fn receive_interest(
	trigger: On<RemoteEvent<EditInterest>>,
	mut subscriptions: ResMut<EditSubscriptions>,
) {
	let event = trigger.event();
	let peer = event.from;
	let interest = event.trigger;
	let subscription = subscriptions.clients.entry(peer).or_default().entry(interest.grid).or_default();
	if interest.interested {
		subscription.interest.retain(interest.region);
	} else {
		subscription.interest.release(interest.region);
	}
}

pub(super) fn flush_edits(
	mut edits: MessageReader<GridEditMessage>,
	peer_metadata: Option<Res<NetworkingMetadata>>,
	mut senders: Query<&mut EventSender<RemoteGridEdit>>,
	mut subscriptions: ResMut<EditSubscriptions>,
) {
	let Some(peer_metadata) = peer_metadata else { return };
	for message in edits.read() {
		let edit = match postcard::to_stdvec(message.edit()) {
			Ok(edit) => edit.into_boxed_slice(),
			Err(error) => {
				warn!(grid=?message.grid_id(), edit_id=?message.edit_id(), %error, "failed to encode grid edit");
				continue;
		}
		};
		let affected_region = chunk_aligned(message.edit().affected_region());
		for (&peer, grids) in &mut subscriptions.clients {
			let Some(subscription) = grids.get_mut(&message.grid_id()) else { continue };
			if !subscription.interest.overlaps(affected_region) { continue; }
			let Some(&entity) = peer_metadata.peer_map.get(&peer) else { continue };
			let Ok(mut sender) = senders.get_mut(entity) else { continue };
			let edit_id = subscription.remap_edit();
			sender.trigger::<ServerToClientChannel>(RemoteGridEdit {
				grid: message.grid_id(),
				edit_id,
				edit: edit.clone(),
			});
		}
	}
}

pub(super) fn cleanup_disconnected(
	peer_metadata: Option<Res<NetworkingMetadata>>,
	mut subscriptions: ResMut<EditSubscriptions>,
) {
	let Some(peer_metadata) = peer_metadata else { return };
	subscriptions.clients.retain(|peer, _| peer_metadata.peer_map.contains_key(peer));
}
