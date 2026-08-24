use std::collections::{BTreeMap, HashMap};

use bevy::math::IVec3;
use bevy::prelude::*;
use lightyear::prelude::{EventSender, PeerId, PeerMetadata, RemoteEvent};
use tile_data::{ChunkRegion, chunks_covering_nonzero_voxel_region};
use voxel_data::grid::GridId;
use voxel_sources::edit::{GridEditIdManager, GridEditMessage, GridGeneration};

use super::{EditInterest, RemoteGridEdit};
use crate::chunks::ServerToClientChannel;

type Area = ChunkRegion;

struct GridSubscription {
	areas: HashMap<Area, ()>,
	area_versions: HashMap<Area, u64>,
	remote_edits: GridEditIdManager,
	server_generations: BTreeMap<GridGeneration, GridGeneration>,
}

impl Default for GridSubscription {
	fn default() -> Self {
		Self {
			areas: HashMap::new(),
			area_versions: HashMap::new(),
			remote_edits: GridEditIdManager::default(),
			server_generations: BTreeMap::from([(GridGeneration::default(), GridGeneration::default())]),
		}
	}
}

impl GridSubscription {
	fn overlaps(&self, min: IVec3, size: IVec3) -> bool {
		self.areas.keys().any(|area| overlaps(area.min(), area.size().as_ivec3(), min, size))
	}

	fn remap_edit(&mut self, server_generation: GridGeneration) -> voxel_sources::edit::GridEditId {
		let (edit_id, remote_generation) = self.remote_edits.apply_edit();
		self.server_generations.insert(remote_generation, server_generation);
		edit_id
	}

	fn server_generation(&self, remote_generation: GridGeneration) -> GridGeneration {
		self.server_generations.range(..=remote_generation)
			.next_back()
			.map(|(_, generation)| *generation)
			.unwrap_or_default()
	}
}

#[derive(Resource, Default)]
pub(crate) struct EditSubscriptions {
	clients: HashMap<PeerId, HashMap<GridId, GridSubscription>>,
}

impl EditSubscriptions {
	pub(crate) fn server_generation(
		&self,
		peer: PeerId,
		grid: GridId,
		remote_generation: GridGeneration,
	) -> GridGeneration {
		self.clients.get(&peer)
			.and_then(|grids| grids.get(&grid))
			.map(|subscription| subscription.server_generation(remote_generation))
			.unwrap_or_default()
	}
}

pub(super) fn receive_interest(
	trigger: On<RemoteEvent<EditInterest>>,
	mut subscriptions: ResMut<EditSubscriptions>,
) {
	let event = trigger.event();
	let peer = event.from;
	let interest = event.trigger;
	let area: Area = interest.region.into();
	let subscription = subscriptions.clients.entry(peer).or_default().entry(interest.grid).or_default();
	let previous_version = subscription.area_versions.get(&area).copied();
	if previous_version.is_some_and(|previous| interest.version <= previous) { return; }
	subscription.area_versions.insert(area, interest.version);
	if interest.interested {
		subscription.areas.insert(area, ());
	} else {
		subscription.areas.remove(&area);
	}
}

pub(super) fn flush_edits(
	mut edits: MessageReader<GridEditMessage>,
	peer_metadata: Option<Res<PeerMetadata>>,
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
		let affected_region = chunks_covering_nonzero_voxel_region(message.edit().affected_region());
		for (&peer, grids) in &mut subscriptions.clients {
			let Some(subscription) = grids.get_mut(&message.grid_id()) else { continue };
			if !subscription.overlaps(affected_region.min(), affected_region.size().as_ivec3()) { continue; }
			let Some(&entity) = peer_metadata.mapping.get(&peer) else { continue };
			let Ok(mut sender) = senders.get_mut(entity) else { continue };
			let edit_id = subscription.remap_edit(message.grid_generation());
			sender.trigger::<ServerToClientChannel>(RemoteGridEdit {
				grid: message.grid_id(),
				edit_id,
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
