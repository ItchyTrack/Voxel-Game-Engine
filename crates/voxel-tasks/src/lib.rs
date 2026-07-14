mod cancellation;
pub mod priority_task_queue;
pub mod task_queue;

use bevy::prelude::*;

pub use cancellation::CancellationToken;
pub use priority_task_queue::{AsyncTaskPriorityQueueResource, AsyncTaskPusher, PriorityTask};
pub use task_queue::TaskQueueResource;

#[derive(Default)]
pub struct VoxelTasksPlugin;

impl Plugin for VoxelTasksPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<TaskQueueResource>()
			.init_resource::<AsyncTaskPriorityQueueResource>()
			.add_systems(Update, drain_task_queue);
	}
}

fn drain_task_queue(world: &mut World) {
	let queue = world.resource::<TaskQueueResource>().clone();
	queue.apply(world);
}
