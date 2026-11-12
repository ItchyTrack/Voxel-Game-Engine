use bevy::app::{PluginGroup, PluginGroupBuilder};
use voxel_content::VoxelStoreSourcePlugin;

mod chunks;
mod replicate_voxels;
mod replication;

pub use replicate_voxels::{ReplicateVoxels, ReplicateVoxelsRestriction};

#[derive(Default)]
pub struct VoxelLightyearPlugins {
	pub enable_client_chunk_source: bool,
	pub enable_server_chunk_source: bool,
}

impl PluginGroup for VoxelLightyearPlugins {
	fn build(self) -> PluginGroupBuilder {
		let group = PluginGroupBuilder::start::<Self>();
		let group = if self.enable_client_chunk_source {
			group.add(VoxelStoreSourcePlugin)
		} else {
			group
		};
		let group = group.add(chunks::ChunkSourcePlugin {
			enable_client_chunk_source: self.enable_client_chunk_source,
			enable_server_chunk_source: self.enable_server_chunk_source,
		});
		if self.enable_server_chunk_source {
			group.add(replication::ReplicationPlugin)
		} else {
			group
		}
	}
}
