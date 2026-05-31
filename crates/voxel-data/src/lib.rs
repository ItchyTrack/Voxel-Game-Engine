pub mod grid_tree;
pub mod voxels;
pub mod grid;
pub mod task_queue;
pub mod transform_ext;
pub mod task_system;

use bevy::prelude::*;

use crate::{task_queue::{AsyncTaskPriorityQueueResource, TaskQueueResource}};

#[derive(Default)]
pub struct VoxelDataPlugin;

impl Plugin for VoxelDataPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<TaskQueueResource>()
			.init_resource::<AsyncTaskPriorityQueueResource>();
		app.add_systems(Update, task_system::drain_task_queue);
	}
}
