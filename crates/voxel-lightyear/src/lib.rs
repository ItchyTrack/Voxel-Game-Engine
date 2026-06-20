use bevy::app::{PluginGroup, PluginGroupBuilder};

mod chunk_source;

pub struct VoxelLightyearPlugins {
	pub enable_client_chunk_source: bool,
	pub enable_server_chunk_source: bool,
}

impl Default for VoxelLightyearPlugins {
	fn default() -> Self {
		Self {
			enable_client_chunk_source: false,
			enable_server_chunk_source: false,
		}
	}
}

impl PluginGroup for VoxelLightyearPlugins {
	fn build(self) -> PluginGroupBuilder {
		PluginGroupBuilder::start::<Self>().add(chunk_source::ChunkSourcePlugin {
			enable_client_chunk_source: self.enable_client_chunk_source,
			enable_server_chunk_source: self.enable_server_chunk_source,
		})
	}
}
