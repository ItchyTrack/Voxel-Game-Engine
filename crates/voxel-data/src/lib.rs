pub mod aabb;
pub mod bvh;
pub mod compressed_voxels;
pub mod grid;
pub mod grid_tree;
pub mod sdf;
pub mod signed_grid_tree;
pub mod splat;
pub mod subgrid;
pub mod task_queue;
pub mod task_system;
pub mod transform_ext;
pub mod voxel_grid_tree;
pub mod voxels;
pub mod world_query;

use bevy::prelude::*;

use crate::task_queue::{AsyncTaskPriorityQueueResource, TaskQueueResource};

#[derive(Default)]
pub struct VoxelDataPlugin;

impl Plugin for VoxelDataPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<TaskQueueResource>()
			.init_resource::<AsyncTaskPriorityQueueResource>();
		app.add_systems(Update, task_system::drain_task_queue);
	}
}
