use bevy::prelude::*;
use lightyear::prelude::{AppTriggerExt, NetworkDirection};

mod client;
mod messages;
mod server;

pub(super) use messages::{RemoteChunkChangeKind, RemoteChunkChanged};

pub(super) struct InvalidationPlugin {
	pub enable_client: bool,
	pub enable_server: bool,
}

impl Plugin for InvalidationPlugin {
	fn build(&self, app: &mut App) {
		app.register_event::<RemoteChunkChanged>()
			.add_map_entities()
			.add_direction(NetworkDirection::ServerToClient);

		if self.enable_server {
			app.add_systems(Update, server::flush_changed);
		}
		if self.enable_client {
			app.add_observer(client::receive_changed);
		}
	}
}
