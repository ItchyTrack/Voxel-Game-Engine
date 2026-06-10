use std::{pin::Pin, sync::Arc};

use async_priority_queue::PriorityQueue;
use bevy::ecs::{resource::Resource, system::Command, world::{CommandQueue, World}};
use crossbeam_channel::{Receiver, Sender};

// --------------------- TaskQueue ---------------------

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

	pub fn push(&self, command: impl Command) {
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

// --------------------- AsyncTaskPriorityQueue ---------------------

pub struct PriorityTask {
	priority: f32,
	task_func: Pin<Box<dyn Future<Output = ()> + Send + 'static>>,
}

impl PriorityTask {
	pub fn new<F: Future<Output = ()> + Send + 'static>(priority: f32, function: F) -> Self {
		Self {
			priority,
			task_func: Box::pin(function),
		}
	}
	pub fn run(self) -> impl Future<Output = ()> {
		self.task_func
	}
}

impl PartialEq for PriorityTask {
	fn eq(&self, other: &Self) -> bool {
		self.priority == other.priority
	}
}

impl Eq for PriorityTask {}

impl PartialOrd for PriorityTask {
	fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
		Some(self.cmp(other))
	}
}

impl Ord for PriorityTask {
	fn cmp(&self, other: &Self) -> std::cmp::Ordering {
		self.priority.total_cmp(&other.priority)
	}
}

type AsyncTaskPriorityQueue = Arc<PriorityQueue<PriorityTask>>;

#[derive(Resource)]
pub struct AsyncTaskPriorityQueueResource {
	queue: AsyncTaskPriorityQueue,
	_threads: Vec<tokio::task::JoinHandle<()>>,
}

impl AsyncTaskPriorityQueueResource {
	pub fn new() -> Self {
		let async_task_priority_queue = Arc::new(PriorityQueue::<PriorityTask>::new());
		let async_task_priority_queue_threads = (0..8).map(|_| {
			let async_task_priority_queue = async_task_priority_queue.clone();
			tokio::spawn(async move {
				loop {
					let task = async_task_priority_queue.pop().await;
					task.run().await;
				}
			})
		}).collect();
		Self {
			queue: async_task_priority_queue,
			_threads: async_task_priority_queue_threads,
		}
	}
	pub fn push(&self, task: PriorityTask) {
		self.queue.push(task);
	}

	pub fn pusher(&self) -> AsyncTaskPusher {
		AsyncTaskPusher { queue: self.queue.clone() }
	}

	pub fn len(&self) -> usize {
		self.queue.len()
	}
}

#[derive(Clone)]
pub struct AsyncTaskPusher {
	queue: AsyncTaskPriorityQueue,
}

impl AsyncTaskPusher {
	pub fn push(&self, task: PriorityTask) {
		self.queue.push(task);
	}
}

impl Default for AsyncTaskPriorityQueueResource {
	fn default() -> Self { Self::new() }
}
