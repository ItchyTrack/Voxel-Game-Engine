pub mod bvh;
pub mod gpu_bvh;

use bevy::prelude::*;

use crate::{task_queue::{AsyncTaskPriorityQueueResource, TaskQueueResource}, world_gpu_data::WorldGpuData};

#[derive(Default)]
pub struct VoxelBvhPlugin;

impl Plugin for VoxelBvhPlugin {
	fn build(&self, app: &mut App) {

	}
}

struct BvhResource {
	bvh: bvh::BVH<Entity>
}
