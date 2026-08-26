use bevy::prelude::*;
use bevy::time::Real;
use lightyear::prelude::{Client, EventSender, RemoteEvent};

mod registry;

pub(crate) use registry::ClientLoadRegistry;

use super::{
	VoxelLoadCancel,
	VoxelLoadManifest,
	VoxelLoadPayload,
	VoxelLoadReceived,
	VoxelLoadRequest,
	VoxelLoadRetry,
};
use crate::chunks::client_source::ClientChunkSource;

pub(super) fn flush_messages(
	time: Res<Time<Real>>,
	source: Res<ClientChunkSource>,
	mut senders: Query<(
		&mut EventSender<VoxelLoadRequest>,
		&mut EventSender<VoxelLoadCancel>,
		&mut EventSender<VoxelLoadRetry>,
		&mut EventSender<VoxelLoadReceived>,
	), With<Client>>,
) {
	let Ok((mut request_sender, mut cancel_sender, mut retry_sender, mut received_sender)) = senders.single_mut() else { return };
	let mut loads = source.state.loads.lock().unwrap();
	loads.drain_cancelled();
	loads.queue_due_retries(time.elapsed());
	while let Some(request) = loads.pop_request() {
		request_sender.trigger::<crate::chunks::ClientToServerChannel>(request);
	}
	while let Some(cancel) = loads.pop_cancellation() {
		cancel_sender.trigger::<crate::chunks::ClientToServerChannel>(cancel);
	}
	while let Some(retry) = loads.pop_retry() {
		retry_sender.trigger::<crate::chunks::ClientToServerChannel>(retry);
	}
	while let Some(received) = loads.pop_received() {
		received_sender.trigger::<crate::chunks::ClientToServerChannel>(received);
	}
}

pub(super) fn receive_payload(
	trigger: On<RemoteEvent<VoxelLoadPayload>>,
	source: Res<ClientChunkSource>,
) {
	let Some(handle) = source.state.handle.get() else { return };
	let event = trigger.event();
	source.state.loads.lock().unwrap().receive_payload(handle, &event.from, &event.trigger);
}

pub(super) fn receive_manifest(
	trigger: On<RemoteEvent<VoxelLoadManifest>>,
	time: Res<Time<Real>>,
	source: Res<ClientChunkSource>,
) {
	let Some(handle) = source.state.handle.get() else { return };
	let event = trigger.event();
	source.state.loads.lock().unwrap().receive_manifest(handle, &event.from, &event.trigger, time.elapsed());
}
