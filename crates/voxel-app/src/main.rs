use bevy::prelude::*;

use tracy_client::{Client};

#[tokio::main]
async fn main() {
	App::new()
		.add_plugins(DefaultPlugins)
		.add_plugin(voxel_data::VoxelDataPlugin)
		.add_plugin(voxel_physics::VoxelPhysicsPlugin)
		.add_plugin(voxel_renderer::VoxelRendererPlugin)
		.run();
}
