use bevy::ecs::{resource::Resource, system::Command, world::{CommandQueue, World}};
use crossbeam_channel::{Receiver, Sender};

const MAX_TASK_QUEUE_APPLIES_PER_FRAME: usize = 32;

#[derive(Resource, Clone)]
pub struct TaskQueueResource {
	sender: Sender<CommandQueue>,
	receiver: Receiver<CommandQueue>,
}

impl TaskQueueResource {
	pub fn new() -> Self {
		let (sender, receiver) = crossbeam_channel::unbounded();
		Self { sender, receiver }
	}

	pub fn push(&self, command: impl Command<Out = ()>) {
		let mut queue = CommandQueue::default();
		queue.push(command);
		let _ = self.sender.send(queue);
	}

	pub fn apply(&self, world: &mut World) {
		for _ in 0..MAX_TASK_QUEUE_APPLIES_PER_FRAME {
			let Ok(mut queue) = self.receiver.try_recv() else { break };
			queue.apply(world);
		}
	}
}

impl Default for TaskQueueResource {
	fn default() -> Self { Self::new() }
}
