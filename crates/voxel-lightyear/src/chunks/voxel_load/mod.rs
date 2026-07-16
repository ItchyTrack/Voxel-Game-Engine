use bevy::prelude::*;
use lightyear::prelude::{AppTriggerExt, NetworkDirection};

mod client;
mod messages;
mod server;

pub(super) use client::ClientLoadRegistry;
use messages::{
	VoxelLoadFinished,
	VoxelLoadId,
	VoxelLoadOutcome,
	VoxelLoadRequest,
	VoxelLoadRequestKind,
	VoxelLoadResponse,
	VoxelLoadResponseKind,
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
		app.register_event::<VoxelLoadResponse>()
			.add_map_entities()
			.add_direction(NetworkDirection::ServerToClient);
		app.register_event::<VoxelLoadFinished>()
			.add_direction(NetworkDirection::ClientToServer);

		if self.enable_server {
			app.init_resource::<PendingVoxelLoads>()
				.add_observer(server::receive_voxel_load_request)
				.add_observer(server::receive_voxel_load_finished)
				.add_systems(
					Update,
					(
						server::flush_chunk_results,
						server::flush_lod_results,
						server::send_pending_voxel_load_responses.after(server::flush_chunk_results).after(server::flush_lod_results),
						server::cleanup_disconnected_requests,
					),
				);
		}
		if self.enable_client {
			app.add_observer(client::receive_response).add_systems(Update, client::flush_messages);
		}
	}
}
