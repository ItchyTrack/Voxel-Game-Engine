use bevy::ecs::world::World;

use crate::task_queue::TaskQueueResource;

pub fn drain_task_queue(world: &mut World) {
	let queue = world.resource::<TaskQueueResource>().clone();
	while let Some(task) = queue.pop_front() {
		task.run(world);
	}
}
