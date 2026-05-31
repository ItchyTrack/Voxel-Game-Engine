pub mod gpu_grid_tree;
pub mod packed_buffer;
pub mod packed_dynamic_buffer;
pub mod world_gpu_data;
pub mod sub_grid_gpu_state;
pub mod matrix;

use bevy::prelude::*;

use crate::{task_queue::{AsyncTaskPriorityQueueResource, TaskQueueResource}, world_gpu_data::WorldGpuData};

#[derive(Default)]
pub struct VoxelDataPlugin;

impl Plugin for VoxelDataPlugin {
	fn build(&self, app: &mut App) {
		app.add_systems(PreUpdate, sub_grid_gpu_state::request_gpu_state);
	}

	fn finish(&self, app: &mut App) {
		app.init_resource::<WorldGpuData>();
	}
}
