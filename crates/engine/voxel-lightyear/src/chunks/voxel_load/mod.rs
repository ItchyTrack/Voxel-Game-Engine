use bevy::prelude::*;
use bevy_trait_query::RegisterExt;
use bile_data::DataConsumingNode;
use lightyear::prelude::{AppTriggerExt, NetworkDirection};
use voxel_sources::{VoxelData, VoxelSourceNodeSet};

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
use server::{PendingVoxelLoads, ServerVoxelLoadNode};

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
			app.world_mut().spawn((Name::new("Server Voxel Load Node"), ServerVoxelLoadNode::default()));
			app.init_resource::<PendingVoxelLoads>()
				.register_component_as::<dyn DataConsumingNode<VoxelData>, ServerVoxelLoadNode>()
				.add_observer(server::receive_voxel_load_request)
				.add_observer(server::receive_voxel_load_cancel)
				.add_observer(server::receive_voxel_load_retry)
				.add_observer(server::receive_voxel_load_received)
				.add_systems(
					Update,
					(
						server::flush_voxel_data,
						server::send_pending_voxel_load_responses,
						server::cleanup_disconnected_requests,
					).chain().before(VoxelSourceNodeSet),
				);
		}
		if self.enable_client {
			app.add_observer(client::receive_payload)
				.add_observer(client::receive_manifest)
				.add_systems(Update, client::flush_messages);
		}
	}
}
