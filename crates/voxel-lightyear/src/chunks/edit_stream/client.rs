use std::collections::{BTreeMap, HashMap};

use bevy::ecs::message::MessageWriter;
use bevy::prelude::*;
use lightyear::prelude::{Client, EventSender, RemoteEvent};
use voxel_data::grid::GridId;
use voxel_streaming::{AuthoritativeGridCommand, ChunkEditInterestChanged};

use super::{EditInterest, EditStreamStart, RemoteGridEdit};
use crate::chunks::ClientToServerChannel;

#[derive(Default)]
struct ClientGridStream {
	next_stream_sequence: u64,
	pending: BTreeMap<u64, RemoteGridEdit>,
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
			region: interest.region.into(),
			version: interest.version,
			interested: interest.interested,
		});
	}
}

pub(super) fn receive_start(
	trigger: On<RemoteEvent<EditStreamStart>>,
	mut streams: ResMut<ClientEditStreams>,
) {
	let event = trigger.event().trigger;
	let next = event.first_stream_sequence.saturating_sub(1);
	let stream = streams.grids.entry(event.grid).or_default();
	if next > stream.next_stream_sequence {
		stream.next_stream_sequence = next;
		stream.pending.retain(|sequence, _| *sequence > next);
	}
}

pub(super) fn receive_edit(
	trigger: On<RemoteEvent<RemoteGridEdit>>,
	mut streams: ResMut<ClientEditStreams>,
	mut commands: MessageWriter<AuthoritativeGridCommand>,
) {
	let event = trigger.event().trigger.clone();
	let stream = streams.grids.entry(event.grid).or_default();
	if event.stream_sequence <= stream.next_stream_sequence { return; }
	stream.pending.insert(event.stream_sequence, event);
	while let Some(event) = stream.pending.remove(&(stream.next_stream_sequence + 1)) {
		stream.next_stream_sequence += 1;
		commands.write(AuthoritativeGridCommand {
			grid: event.grid,
			region: event.region,
			stream_sequence: stream.next_stream_sequence,
			generation: event.generation,
			edit: event.edit.into_edit(),
		});
	}
}
