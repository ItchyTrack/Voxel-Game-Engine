use bevy::prelude::*;
use lightyear::prelude::{AppTriggerExt, NetworkDirection};

mod client;
mod messages;
mod server;

pub(super) use client::ClientPresenceRegistry;
pub(super) use messages::{PresenceCancel, PresenceLoad, PresenceLoaded, PresenceRequest};

pub(super) struct PresencePlugin {
	pub enable_client: bool,
	pub enable_server: bool,
}

impl Plugin for PresencePlugin {
	fn build(&self, app: &mut App) {
		app.register_event::<PresenceRequest>()
			.add_map_entities()
			.add_direction(NetworkDirection::ClientToServer);
		app.register_event::<PresenceCancel>()
			.add_direction(NetworkDirection::ClientToServer);
		app.register_event::<PresenceLoad>()
			.add_map_entities()
			.add_direction(NetworkDirection::ServerToClient);
		app.register_event::<PresenceLoaded>()
			.add_direction(NetworkDirection::ServerToClient);

		if self.enable_server {
			app.init_resource::<server::PendingPresenceLoads>()
				.add_observer(server::receive_request)
				.add_observer(server::receive_cancel)
				.add_systems(
					Update,
					(
						server::flush_source_results,
						server::send_pending_presence,
						server::cleanup_disconnected,
					).chain(),
				);
		}
		if self.enable_client {
			app.add_observer(client::receive_load)
				.add_observer(client::receive_loaded)
				.add_systems(Update, client::flush_requests);
		}
	}
}
