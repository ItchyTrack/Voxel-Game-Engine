use bevy::prelude::*;
use lightyear::prelude::{AppTriggerExt, NetworkDirection};

mod client;
mod messages;
mod server;

pub(super) use client::ClientLoadRegistry;
use messages::{
	VoxelLoadCancel,
	VoxelLoadComplete,
	VoxelLoadPayload,
	VoxelLoadRequest,
	VoxelRequestKey,
};
use server::PendingVoxelLoads;

pub(super) struct VoxelLoadPlugin {
	pub enable_client: bool,
	pub enable_server: bool,
}

impl Plugin for VoxelLoadPlugin {
	fn build(&self, app: &mut App) {
		app.register_event::<VoxelLoadRequest>()
			.add_map_entities()
			.add_direction(NetworkDirection::ClientToServer);
		app.register_event::<VoxelLoadCancel>()
			.add_direction(NetworkDirection::ClientToServer);
		app.register_event::<VoxelLoadPayload>()
			.add_map_entities()
			.add_direction(NetworkDirection::ServerToClient);
		app.register_event::<VoxelLoadComplete>()
			.add_direction(NetworkDirection::ServerToClient);

		if self.enable_server {
			app.init_resource::<PendingVoxelLoads>()
				.add_observer(server::receive_voxel_load_request)
				.add_observer(server::receive_voxel_load_cancel)
				.add_systems(
					Update,
					(
						server::flush_source_results,
						server::send_pending_voxel_load_responses,
						server::cleanup_disconnected_requests,
					).chain(),
				);
		}
		if self.enable_client {
			app.add_observer(client::receive_payload)
				.add_observer(client::receive_complete)
				.add_systems(Update, client::flush_messages);
		}
	}
}
