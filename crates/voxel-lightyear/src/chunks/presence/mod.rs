use bevy::prelude::*;
use lightyear::prelude::{AppTriggerExt, NetworkDirection};

mod client;
mod messages;
mod server;

pub(super) use messages::PresenceRequest;
use messages::PresenceLoad;

pub(super) struct PresencePlugin {
	pub enable_client: bool,
	pub enable_server: bool,
}

impl Plugin for PresencePlugin {
	fn build(&self, app: &mut App) {
		app.register_event::<PresenceRequest>()
			.add_map_entities()
			.add_direction(NetworkDirection::ClientToServer);
		app.register_event::<PresenceLoad>()
			.add_map_entities()
			.add_direction(NetworkDirection::ServerToClient);

		if self.enable_server {
			app.add_observer(server::receive_request);
		}
		if self.enable_client {
			app.add_observer(client::receive_load).add_systems(Update, client::flush_requests);
		}
	}
}
