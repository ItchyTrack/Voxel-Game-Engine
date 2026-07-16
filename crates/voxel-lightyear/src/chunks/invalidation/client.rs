use bevy::prelude::*;
use lightyear::prelude::RemoteEvent;

use super::RemoteChunkChanged;
use crate::chunks::client_source::ClientChunkSource;

pub(super) fn receive_changed(
	trigger: On<RemoteEvent<RemoteChunkChanged>>,
	source: Res<ClientChunkSource>,
) {
	let Some(handle) = source.state.handle.get() else { return };
	let event = trigger.event().trigger;
	source.state.loads.lock().unwrap().receive_chunk_changed(handle, event);
}
