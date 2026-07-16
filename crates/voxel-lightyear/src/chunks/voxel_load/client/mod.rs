use bevy::prelude::*;
use lightyear::prelude::{Client, EventSender, RemoteEvent};

mod registry;

pub(crate) use registry::ClientLoadRegistry;

use super::{VoxelLoadFinished, VoxelLoadRequest, VoxelLoadResponse};
use crate::chunks::client_source::ClientChunkSource;

pub(super) fn flush_messages(
	source: Res<ClientChunkSource>,
	mut senders: Query<(&mut EventSender<VoxelLoadRequest>, &mut EventSender<VoxelLoadFinished>), With<Client>>,
) {
	let Ok((mut request_sender, mut finished_sender)) = senders.single_mut() else { return };
	let mut loads = source.state.loads.lock().unwrap();
	loads.drain_cancelled();
	while let Some(request) = loads.pop_request() {
		request_sender.trigger::<crate::chunks::ClientToServerChannel>(request);
	}
	while let Some(finished) = loads.pop_finished() {
		finished_sender.trigger::<crate::chunks::ClientToServerChannel>(finished);
	}
}

pub(super) fn receive_response(
	mut trigger: On<RemoteEvent<VoxelLoadResponse>>,
	source: Res<ClientChunkSource>,
) {
	let Some(handle) = source.state.handle.get() else { return };
	let from = trigger.event().from;
	let response = &mut trigger.event_mut().trigger;
	source.state.loads.lock().unwrap().receive_response(handle, from, response);
}
