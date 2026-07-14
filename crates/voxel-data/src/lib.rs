pub mod aabb;
pub mod bvh;
pub mod compressed_voxels;
pub mod grid;
pub mod grid_tree;
pub mod sdf;
pub mod signed_grid_tree;
pub mod splat;
pub mod subgrid;
pub mod transform_ext;
pub mod voxel_grid_tree;
pub mod voxels;
pub mod world_query;

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
