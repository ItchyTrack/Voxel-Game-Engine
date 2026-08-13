use std::collections::{BTreeMap, HashMap};

use bevy::prelude::*;
use lightyear::prelude::{Client, EventSender, RemoteEvent};
use voxel_data::grid::GridId;
use voxel_edit::GridEdits;
use voxel_streaming::ChunkEditInterestChanged;

use super::{EditInterest, EditStreamStart, RemoteGridEdit, WireGridEdit};
use crate::chunks::{ClientToServerChannel, client_source::ClientChunkSource};

#[derive(Default)]
struct ClientGridStream {
	next_edit_index: u64,
	pending: BTreeMap<u64, WireGridEdit>,
}

#[derive(Resource, Default)]
pub(super) struct ClientEditStreams {
	grids: HashMap<GridId, ClientGridStream>,
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

pub(super) fn receive_start(
	trigger: On<RemoteEvent<EditStreamStart>>,
	source: Res<ClientChunkSource>,
	mut streams: ResMut<ClientEditStreams>,
) {
	let event = trigger.event().trigger;
	let next = event.first_edit_index.saturating_sub(1);
	let stream = streams.grids.entry(event.grid).or_default();
	stream.next_edit_index = stream.next_edit_index.max(next);
	if let Some(handle) = source.state.handle.get() { handle.synchronize_edit_index(event.grid, stream.next_edit_index); }
}

pub(super) fn receive_edit(
	trigger: On<RemoteEvent<RemoteGridEdit>>,
	mut streams: ResMut<ClientEditStreams>,
	mut grids: Query<&mut GridEdits>,
) {
	let event = trigger.event().trigger.clone();
	let stream = streams.grids.entry(event.grid).or_default();
	if event.edit_index <= stream.next_edit_index { return; }
	stream.pending.insert(event.edit_index, event.edit);
	let Ok(mut edits) = grids.get_mut(event.grid) else { return };
	while let Some(edit) = stream.pending.remove(&(stream.next_edit_index + 1)) {
		edits.push_edit(edit.into_edit());
		stream.next_edit_index += 1;
	}
}
