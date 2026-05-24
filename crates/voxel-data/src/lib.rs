pub mod voxels {
	pub mod grid_tree;
	pub mod gpu_grid_tree;
	pub mod voxels;
}
pub mod bvh {
	pub mod bvh;
	pub mod gpu_bvh;
}
pub mod buffers {
	pub mod packed_buffer;
	pub mod packed_dynamic_buffer;
}
pub mod grid;
pub mod world_gpu_data;
pub mod sub_grid_gpu_state;
pub mod matrix;
pub mod task_queue;
pub mod transform_ext;
pub mod task_system;

use bevy::prelude::*;

use crate::{task_queue::{AsyncTaskPriorityQueueResource, TaskQueueResource}, world_gpu_data::WorldGpuData};

#[derive(Default)]
pub struct VoxelDataPlugin;

impl Plugin for VoxelDataPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<TaskQueueResource>()
			.init_resource::<AsyncTaskPriorityQueueResource>();
		app.add_systems(PreUpdate, sub_grid_gpu_state::request_gpu_state);
		app.add_systems(Update, task_system::drain_task_queue);
	}

	fn finish(&self, app: &mut App) {
		// RenderDevice/RenderQueue are inserted by RenderPlugin::finish.
		app.init_resource::<WorldGpuData>();
	}
}
