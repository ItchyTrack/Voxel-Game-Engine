use bevy::prelude::*;
use lightyear::prelude::{Client, EventSender, RemoteEvent};

use super::{PresenceLoad, PresenceRequest};
use crate::chunks::client_source::ClientChunkSource;

pub(super) fn flush_requests(
	source: Res<ClientChunkSource>,
	mut senders: Query<&mut EventSender<PresenceRequest>, With<Client>>,
) {
	let Ok(mut sender) = senders.single_mut() else { return };
	let mut requests = source.state.presence_requests.lock().unwrap();
	while let Some(request) = requests.pop_front() {
		sender.trigger::<crate::chunks::ClientToServerChannel>(request);
	}
}

pub(super) fn receive_load(trigger: On<RemoteEvent<PresenceLoad>>, source: Res<ClientChunkSource>) {
	let Some(handle) = source.state.handle.get() else { return };
	let event = trigger.event().trigger;
	if let Some((min, size)) = event.area {
		handle.claim(event.grid, min, size);
	}
	handle.presence_loaded(event.grid);
}
