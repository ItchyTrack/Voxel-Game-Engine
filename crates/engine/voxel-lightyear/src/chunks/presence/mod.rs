use bevy::prelude::*;
use bevy_trait_query::RegisterExt;
use bile_data::DataConsumingNode;
use lightyear::prelude::{AppTriggerExt, NetworkDirection};
use voxel_sources::{VoxelPresence, VoxelSourceNodeSet};

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
			app.world_mut().spawn((Name::new("Server Presence Node"), server::ServerPresenceNode::default()));
			app.init_resource::<server::PendingPresenceLoads>()
				.register_component_as::<dyn DataConsumingNode<VoxelPresence>, server::ServerPresenceNode>()
				.add_observer(server::receive_request)
				.add_observer(server::receive_cancel)
				.add_systems(
					Update,
					(
						server::flush_presence,
						server::send_pending_presence,
						server::cleanup_disconnected,
					).chain().before(VoxelSourceNodeSet),
				);
		}
		if self.enable_client {
			app.add_observer(client::receive_load)
				.add_observer(client::receive_loaded)
				.add_systems(Update, client::flush_requests);
		}
	}
}
