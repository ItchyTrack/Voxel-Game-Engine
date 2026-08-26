pub mod aabb;
pub mod bvh;
pub mod compressed_voxels;
pub mod grid;
pub mod grid_tree;
pub mod region;
pub mod sdf;
pub mod signed_grid_tree;
pub mod voxel_grid_tree;
pub mod voxels;

use bevy::prelude::*;

use voxel_tasks::VoxelTasksPlugin;

#[derive(Default)]
pub struct VoxelDataPlugin;

impl Plugin for VoxelDataPlugin {
	fn build(&self, app: &mut App) {
		if !app.is_plugin_added::<VoxelTasksPlugin>() {
			app.add_plugins(VoxelTasksPlugin);
		}
	}
}
