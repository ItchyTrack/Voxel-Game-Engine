use std::time::Duration;

use bevy::prelude::*;
use lightyear::prelude::{AppChannelExt, ChannelMode, ChannelSettings, NetworkDirection, ReliableSettings};

mod client_source;
mod edit_stream;
mod presence;
mod request_id;
mod voxel_load;

use client_source::ClientChunkSource;
use edit_stream::EditStreamPlugin;
use presence::PresencePlugin;
use voxel_load::VoxelLoadPlugin;

pub(crate) struct ClientToServerChannel;
pub(crate) struct ServerToClientChannel;

pub struct ChunkSourcePlugin {
	pub enable_client_chunk_source: bool,
	pub enable_server_chunk_source: bool,
}

impl Plugin for ChunkSourcePlugin {
	fn build(&self, app: &mut App) {
		if self.enable_client_chunk_source {
			use voxel_sources::VoxelSourcesAppExt;
			let client_chunk_source = ClientChunkSource::default();
			app.insert_resource(client_chunk_source.clone())
				.register_voxel_source(client_chunk_source)
				.add_systems(
					Update,
					client_source::register_remote_voxel_grids.before(voxel_streaming::StreamingPhase::Ingest),
				);
		}

		app.add_channel::<ClientToServerChannel>(ordered_reliable_channel())
			.add_direction(NetworkDirection::ClientToServer);
		app.add_channel::<ServerToClientChannel>(ordered_reliable_channel())
			.add_direction(NetworkDirection::ServerToClient);

		app.add_plugins((
			PresencePlugin {
				enable_client: self.enable_client_chunk_source,
				enable_server: self.enable_server_chunk_source,
			},
			EditStreamPlugin {
				enable_client: self.enable_client_chunk_source,
				enable_server: self.enable_server_chunk_source,
			},
			VoxelLoadPlugin {
				enable_client: self.enable_client_chunk_source,
				enable_server: self.enable_server_chunk_source,
			},
		));
	}
}

fn ordered_reliable_channel() -> ChannelSettings {
	ChannelSettings {
		mode: ChannelMode::OrderedReliable(ReliableSettings::default()),
		send_frequency: Duration::from_millis(100),
		priority: 1.0,
	}
}
