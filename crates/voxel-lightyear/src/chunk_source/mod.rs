use std::time::Duration;

use bevy::prelude::*;
use lightyear::prelude::{AppChannelExt, AppTriggerExt, ChannelMode, ChannelSettings, NetworkDirection, ReliableSettings};

mod client;
mod messages;
mod server;

use client::ClientChunkSource;
use messages::{ChunkPresenceAabb, ChunkRequest, ChunkResponse, LodRequest, LodResponse, PresenceRequest};
use server::{PendingChunkRequests, PendingLodRequests};

pub(crate) struct ChunkRequestChannel;
pub(crate) struct ChunkResponseChannel;
pub(crate) struct LodRequestChannel;
pub(crate) struct LodResponseChannel;
pub(crate) struct PresenceRequestChannel;
pub(crate) struct ChunkPresenceAabbChannel;

pub struct ChunkSourcePlugin {
	pub enable_client_chunk_source: bool,
	pub enable_server_chunk_source: bool,
}

impl Plugin for ChunkSourcePlugin {
	fn build(&self, app: &mut App) {
		if self.enable_client_chunk_source {
			use voxel_sources::VoxelSourcesAppExt;
			let client_chunk_source = ClientChunkSource::default();
			app.insert_resource(client_chunk_source.clone()).register_source(client_chunk_source);
		}
		app.add_channel::<PresenceRequestChannel>(unordered_reliable_channel())
			.add_direction(NetworkDirection::ClientToServer);
		app.add_channel::<ChunkRequestChannel>(unordered_reliable_channel())
			.add_direction(NetworkDirection::ClientToServer);
		app.add_channel::<ChunkResponseChannel>(unordered_reliable_channel())
			.add_direction(NetworkDirection::ServerToClient);
		app.add_channel::<LodRequestChannel>(unordered_reliable_channel())
			.add_direction(NetworkDirection::ClientToServer);
		app.add_channel::<LodResponseChannel>(unordered_reliable_channel())
			.add_direction(NetworkDirection::ServerToClient);
		app.add_channel::<ChunkPresenceAabbChannel>(unordered_reliable_channel())
			.add_direction(NetworkDirection::ServerToClient);
		app.register_event::<PresenceRequest>()
			.add_map_entities()
			.add_direction(NetworkDirection::ClientToServer);
		app.register_event::<ChunkRequest>()
			.add_map_entities()
			.add_direction(NetworkDirection::ClientToServer);
		app.register_event::<ChunkResponse>()
			.add_map_entities()
			.add_direction(NetworkDirection::ServerToClient);
		app.register_event::<LodRequest>()
			.add_map_entities()
			.add_direction(NetworkDirection::ClientToServer);
		app.register_event::<LodResponse>()
			.add_map_entities()
			.add_direction(NetworkDirection::ServerToClient);
		app.register_event::<ChunkPresenceAabb>()
			.add_map_entities()
			.add_direction(NetworkDirection::ServerToClient);
		if self.enable_server_chunk_source {
			app.init_resource::<PendingChunkRequests>()
				.add_observer(server::receive_presence_request)
				.init_resource::<PendingLodRequests>()
				.add_observer(server::receive_chunk_request)
				.add_observer(server::receive_lod_request)
				.add_systems(Update, (server::flush_chunk_results, server::flush_lod_results));
		}

		if self.enable_client_chunk_source {
			app.add_observer(client::receive_remote_chunk_response)
				.add_observer(client::receive_remote_lod_response)
				.add_observer(client::receive_chunk_presence_aabb)
				.add_systems(
					Update,
					(
						client::flush_remote_presence_requests,
						client::flush_remote_chunk_requests,
						client::flush_remote_lod_requests,
					),
				);
		}
	}
}

// fn reliable_channel() -> ChannelSettings {
// 	ChannelSettings {
// 		mode: ChannelMode::OrderedReliable(ReliableSettings::default()),
// 		send_frequency: Duration::from_millis(100),
// 		priority: 1.0,
// 	}
// }

fn unordered_reliable_channel() -> ChannelSettings {      
	ChannelSettings {                                      
		mode: ChannelMode::UnorderedReliable(ReliableSettings::default()),
		send_frequency: Duration::default(),                
		priority: 1.0,                                      
	}                                                      
}  
