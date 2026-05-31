use std::{collections::VecDeque, pin::Pin, sync::{Arc, Mutex}};

use async_priority_queue::PriorityQueue;
use bevy::ecs::{resource::Resource, world::World};

// --------------------- TaskQueue ---------------------

pub struct Task {
	task_func: Box<dyn FnOnce(&mut World) + Send + 'static>,
}

impl Task {
	pub fn new<F: FnOnce(&mut World) + Send + 'static>(function: F) -> Self {
		Self {
			task_func: Box::new(function),
		}
	}
	pub fn run(self, world: &mut World) {
		(self.task_func)(world);
	}
}

type TaskQueue = Arc<Mutex<VecDeque<Task>>>;

#[derive(Resource, Clone)]
pub struct TaskQueueResource {
	queue: TaskQueue,
}

impl TaskQueueResource {
	pub fn new() -> Self {
		Self {
			queue: TaskQueue::new(Mutex::new(VecDeque::new())),
		}
	}

	pub fn push_back(&self, task: Task) {
		self.queue.lock().unwrap().push_back(task);
	}

	pub fn pop_front(&self) -> Option<Task> {
		self.queue.lock().unwrap().pop_front()
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
}

impl Default for AsyncTaskPriorityQueueResource {
	fn default() -> Self { Self::new() }
}
