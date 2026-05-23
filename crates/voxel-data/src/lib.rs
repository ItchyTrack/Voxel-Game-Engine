pub mod grid;
pub mod grid_tree;
pub mod gpu_grid_tree;
pub mod matrix;
pub mod packed_buffer;
pub mod packed_dynamic_buffer;
pub mod voxel_tracker;
pub mod voxels;
pub mod bvh;
pub mod gpu_bvh;
pub mod world_gpu_data;
pub mod sub_grid_gpu_state;
pub mod task_queue;

use bevy::prelude::*;

use crate::{task_queue::{AsyncTaskPriorityQueueResource, TaskQueueResource}, world_gpu_data::WorldGpuData};

#[derive(Default)]
pub struct VoxelDataPlugin;

impl Plugin for VoxelDataPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<TaskQueueResource>()
			.init_resource::<AsyncTaskPriorityQueueResource>();
		app.add_systems(PreUpdate, sub_grid_gpu_state::request_gpu_state);

		let render_app = app.sub_app_mut(bevy::render::RenderApp);
		render_app.init_resource::<WorldGpuData>(); // FromWorld runs here, RenderDevice exists
	}
}
