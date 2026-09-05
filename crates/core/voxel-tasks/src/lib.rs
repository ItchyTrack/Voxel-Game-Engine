mod cancellation;
pub mod priority_task_queue;

use bevy::prelude::*;

pub use priority_task_queue::{AsyncPriorityTaskPool, PriorityTask};
pub use cancellation::CancellationToken;

#[derive(Default)]
pub struct VoxelTasksPlugin;

impl Plugin for VoxelTasksPlugin {
	fn build(&self, _app: &mut App) {
		AsyncPriorityTaskPool::get_or_init(|| AsyncPriorityTaskPool::default());
	}
}
