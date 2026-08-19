use bevy::prelude::*;
use lightyear::prelude::{Client, EventSender, RemoteEvent};

mod registry;

pub(crate) use registry::ClientLoadRegistry;

use super::{VoxelLoadCancel, VoxelLoadComplete, VoxelLoadPayload, VoxelLoadRequest};
use crate::chunks::client_source::ClientChunkSource;

pub(super) fn flush_messages(
	source: Res<ClientChunkSource>,
	mut senders: Query<(&mut EventSender<VoxelLoadRequest>, &mut EventSender<VoxelLoadCancel>), With<Client>>,
) {
	let Ok((mut request_sender, mut cancel_sender)) = senders.single_mut() else { return };
	let mut loads = source.state.loads.lock().unwrap();
	loads.drain_cancelled();
	while let Some(request) = loads.pop_request() {
		request_sender.trigger::<crate::chunks::ClientToServerChannel>(request);
	}
	while let Some(cancel) = loads.pop_cancellation() {
		cancel_sender.trigger::<crate::chunks::ClientToServerChannel>(cancel);
	}
}

pub(super) fn receive_payload(
	trigger: On<RemoteEvent<VoxelLoadPayload>>,
	source: Res<ClientChunkSource>,
) {
	let Some(handle) = source.state.handle.get() else { return };
	let event = trigger.event();
	source.state.loads.lock().unwrap().receive_payload(handle, event.from, &event.trigger);
}

pub(super) fn receive_complete(
	trigger: On<RemoteEvent<VoxelLoadComplete>>,
	source: Res<ClientChunkSource>,
) {
	let Some(handle) = source.state.handle.get() else { return };
	let event = trigger.event();
	source.state.loads.lock().unwrap().receive_complete(handle, event.from, event.trigger);
}
