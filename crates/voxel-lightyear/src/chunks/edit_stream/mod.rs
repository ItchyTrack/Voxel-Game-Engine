use bevy::prelude::*;
use lightyear::prelude::{AppTriggerExt, NetworkDirection};

mod client;
mod messages;
mod server;

use messages::{EditInterest, EditStreamStart, RemoteGridEdit, WireGridEdit};
pub(crate) use server::EditSubscriptions;

pub(super) struct EditStreamPlugin {
	pub enable_client: bool,
	pub enable_server: bool,
}

impl Plugin for EditStreamPlugin {
	fn build(&self, app: &mut App) {
		app.register_event::<EditInterest>().add_map_entities().add_direction(NetworkDirection::ClientToServer);
		app.register_event::<EditStreamStart>().add_map_entities().add_direction(NetworkDirection::ServerToClient);
		app.register_event::<RemoteGridEdit>().add_map_entities().add_direction(NetworkDirection::ServerToClient);
		if self.enable_server {
			app.init_resource::<EditSubscriptions>()
				.add_observer(server::receive_interest)
				.add_systems(Update, (server::flush_edits, server::cleanup_disconnected));
		}
		if self.enable_client {
			app.init_resource::<client::ClientEditStreams>()
				.add_observer(client::receive_start)
				.add_observer(client::receive_edit)
				.add_systems(Update, client::flush_interest);
		}
	}
}
