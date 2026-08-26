use bevy::prelude::*;
use lightyear::prelude::{AppTriggerExt, NetworkDirection};

mod client;
mod messages;
mod server;

pub(super) use client::ClientLoadRegistry;
use messages::{
	VoxelLoadCancel,
	VoxelLoadManifest,
	VoxelLoadPayload,
	VoxelLoadReceived,
	VoxelLoadRequest,
	VoxelLoadRetry,
	VoxelPayloadIndex,
	VoxelPayloadMetadata,
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
		app.register_event::<VoxelLoadRetry>()
			.add_direction(NetworkDirection::ClientToServer);
		app.register_event::<VoxelLoadReceived>()
			.add_direction(NetworkDirection::ClientToServer);
		app.register_event::<VoxelLoadManifest>()
			.add_direction(NetworkDirection::ServerToClient);
		app.register_event::<VoxelLoadPayload>()
			.add_direction(NetworkDirection::ServerToClient);

		if self.enable_server {
			app.init_resource::<PendingVoxelLoads>()
				.add_observer(server::receive_voxel_load_request)
				.add_observer(server::receive_voxel_load_cancel)
				.add_observer(server::receive_voxel_load_retry)
				.add_observer(server::receive_voxel_load_received)
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
				.add_observer(client::receive_manifest)
				.add_systems(Update, client::flush_messages);
		}
	}
}
