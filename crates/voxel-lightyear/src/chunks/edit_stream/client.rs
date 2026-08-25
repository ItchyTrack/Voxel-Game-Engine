use std::collections::{HashMap, VecDeque};

use bevy::prelude::*;
use lightyear::prelude::{Client, EventSender, RemoteEvent};
use voxel_content::grid_store::GridStoreEditApi;
use voxel_data::grid::GridId;
use voxel_sources::edit::{GridEdit, GridEditId};
use voxel_streaming::ChunkEditInterestChanged;

use super::{EditInterest, RemoteGridEdit};
use crate::chunks::ClientToServerChannel;

struct PendingGridEdit {
	edit_id: GridEditId,
	edit: Box<dyn GridEdit>,
}

#[derive(Default)]
struct ClientGridEdits {
	last_applied: GridEditId,
	pending: VecDeque<PendingGridEdit>,
}

#[derive(Resource, Default)]
pub(super) struct ClientEditStreams {
	grids: HashMap<GridId, ClientGridEdits>,
}

pub(super) fn flush_interest(
	mut interests: MessageReader<ChunkEditInterestChanged>,
	mut senders: Query<&mut EventSender<EditInterest>, With<Client>>,
) {
	let Ok(mut sender) = senders.single_mut() else { return };
	for interest in interests.read().copied() {
		sender.trigger::<ClientToServerChannel>(EditInterest {
			grid: interest.grid,
			region: interest.region,
			interested: interest.interested,
		});
	}
}

pub(super) fn receive_edit(
	trigger: On<RemoteEvent<RemoteGridEdit>>,
	mut streams: ResMut<ClientEditStreams>,
) {
	let remote = &trigger.event().trigger;
	let stream = streams.grids.entry(remote.grid).or_default();
	let previous_id = stream.pending.back().map_or(stream.last_applied, |edit| edit.edit_id);
	if !previous_id.is_next(remote.edit_id) {
		warn!(grid=?remote.grid, previous=?previous_id, received=?remote.edit_id, "remote grid edit arrived out of order");
		return;
	}
	let edit = match postcard::from_bytes::<Box<dyn GridEdit>>(&remote.edit) {
		Ok(edit) => edit,
		Err(error) => {
			warn!(grid=?remote.grid, edit_id=?remote.edit_id, %error, "failed to decode remote grid edit");
			return;
		}
	};
	stream.pending.push_back(PendingGridEdit { edit_id: remote.edit_id, edit });
}

pub(super) fn apply_pending_edits(
	mut streams: ResMut<ClientEditStreams>,
	mut edits: GridStoreEditApi,
) {
	for (&grid, stream) in &mut streams.grids {
		while let Some(pending) = stream.pending.pop_front() {
			let edit_id = pending.edit_id;
			if let Err(edit) = edits.apply_boxed(grid, pending.edit) {
				stream.pending.push_front(PendingGridEdit { edit_id, edit });
				break;
			}
			stream.last_applied = edit_id;
		}
	}
}
