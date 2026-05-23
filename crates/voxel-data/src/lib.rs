mod grid;
mod grid_tree;
mod gpu_grid_tree;
mod matrix;
mod packed_buffer;
mod packed_dynamic_buffer;
mod voxel_tracker;
mod voxels;
mod bvh;
mod gpu_bvh;
mod world_gpu_data;
mod sub_grid_gpu_state;
mod task_queue;

use bevy::prelude::*;

use crate::world_gpu_data::WorldGpuData;

#[derive(Default)]
pub struct VoxelDataPlugin;

impl Plugin for VoxelDataPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<WorldGpuData>();
		app.add_systems(PreUpdate, sub_grid_gpu_state::request_gpu_state);
	}
}
