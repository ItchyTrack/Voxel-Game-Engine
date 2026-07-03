mod client_rendering;

use bevy::app::{PluginGroup, PluginGroupBuilder};
use bevy::prelude::*;
use voxel_data::VoxelDataPlugin;
use voxel_edit::VoxelEditPlugin;
use voxel_gpu::GpuVoxelDataPlugin;
use voxel_lightyear::VoxelLightyearPlugins;
use voxel_physics::VoxelPhysicsPlugin;
use voxel_streaming::VoxelStreamingPlugin;

use crate::client_rendering::ClientRenderingPlugins;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum VoxelEngineMode {
	#[default]
	Server,
	Client,
	Host,
}

#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq, Reflect)]
pub enum VoxelCameraMode {
	#[default]
	Ray,
	Raster,
}

pub struct VoxelEnginePlugins {
	pub mode: VoxelEngineMode,
}

impl PluginGroup for VoxelEnginePlugins {
	fn build(self) -> PluginGroupBuilder {
		let group = PluginGroupBuilder::start::<Self>()
			.add(GpuVoxelDataPlugin)
			.add(VoxelDataPlugin)
			.add(VoxelEditPlugin)
			.add(VoxelStreamingPlugin)
			.add(VoxelPhysicsPlugin);

		match self.mode {
			VoxelEngineMode::Server => group.add_group(VoxelLightyearPlugins {
				enable_client_chunk_source: false,
				enable_server_chunk_source: true,
			}),
			VoxelEngineMode::Client => group
				.add_group(VoxelLightyearPlugins {
					enable_client_chunk_source: true,
					enable_server_chunk_source: false,
				})
				.add_group(ClientRenderingPlugins),
			VoxelEngineMode::Host => group
				.add_group(VoxelLightyearPlugins {
					enable_client_chunk_source: false,
					enable_server_chunk_source: true,
				})
				.add_group(ClientRenderingPlugins),
		}
	}
}
