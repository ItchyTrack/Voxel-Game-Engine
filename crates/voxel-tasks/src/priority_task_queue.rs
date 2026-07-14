use std::pin::Pin;
use std::sync::Arc;
#[cfg(target_arch = "wasm32")]
use std::sync::OnceLock;
#[cfg(not(target_arch = "wasm32"))]
use std::thread;
#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;
#[cfg(target_arch = "wasm32")]
use wasm_thread as thread;
#[cfg(not(target_arch = "wasm32"))]
use thread::JoinHandle;

use async_priority_queue::PriorityQueue;
use bevy::ecs::resource::Resource;

pub struct PriorityTask {
	priority: f32,
	task_func: Pin<Box<dyn Future<Output = ()> + Send + 'static>>,
}

impl PriorityTask {
	pub fn new<F: Future<Output = ()> + Send + 'static>(priority: f32, function: F) -> Self {
		Self { priority, task_func: Box::pin(function) }
	}

	pub fn run(self) -> impl Future<Output = ()> { self.task_func }
}

impl PartialEq for PriorityTask {
	fn eq(&self, other: &Self) -> bool { self.priority == other.priority }
}
impl Eq for PriorityTask {}
impl PartialOrd for PriorityTask {
	fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> { Some(self.cmp(other)) }
}
impl Ord for PriorityTask {
	fn cmp(&self, other: &Self) -> std::cmp::Ordering { self.priority.total_cmp(&other.priority) }
}

type AsyncTaskPriorityQueue = Arc<PriorityQueue<PriorityTask>>;

#[cfg(target_arch = "wasm32")]
static GLOBAL_ASYNC_TASK_QUEUE: OnceLock<AsyncTaskPriorityQueue> = OnceLock::new();

#[derive(Resource)]
pub struct AsyncTaskPriorityQueueResource {
	queue: AsyncTaskPriorityQueue,
	#[cfg(not(target_arch = "wasm32"))]
	_threads: Vec<JoinHandle<()>>,
}

impl AsyncTaskPriorityQueueResource {
	pub fn new() -> Self {
		let queue = {
			#[cfg(target_arch = "wasm32")]
			{
				GLOBAL_ASYNC_TASK_QUEUE.get_or_init(|| Arc::new(PriorityQueue::<PriorityTask>::new())).clone()
			}
			#[cfg(not(target_arch = "wasm32"))]
			{
				Arc::new(PriorityQueue::<PriorityTask>::new())
			}
		};

		#[cfg(not(target_arch = "wasm32"))]
		let threads = (0..8).map(|i| {
			let queue = queue.clone();
			thread::Builder::new()
				.name(format!("async-task-priority-queue-{i}"))
				.spawn(move || futures::executor::block_on(async move {
					loop {
						let task = queue.pop().await;
						task.run().await;
					}
				}))
				.unwrap()
		}).collect();

		Self {
			queue,
			#[cfg(not(target_arch = "wasm32"))]
			_threads: threads,
		}
	}

	pub fn push(&self, task: PriorityTask) { self.queue.push(task); }
	pub fn pusher(&self) -> AsyncTaskPusher { AsyncTaskPusher { queue: self.queue.clone() } }
	pub fn len(&self) -> usize { self.queue.len() }
	pub fn is_empty(&self) -> bool { self.len() == 0 }
}

impl Default for AsyncTaskPriorityQueueResource {
	fn default() -> Self { Self::new() }
}

#[derive(Clone)]
pub struct AsyncTaskPusher {
	queue: AsyncTaskPriorityQueue,
}

impl AsyncTaskPusher {
	pub fn push(&self, task: PriorityTask) { self.queue.push(task); }
}

// Keep the exported name stable for the existing web worker bootstrap script.
#[cfg(target_arch = "wasm32")]
#[wasm_bindgen]
pub fn voxel_data_async_worker_loop(_worker_id: u32) {
	let queue = loop {
		if let Some(queue) = GLOBAL_ASYNC_TASK_QUEUE.get() { break queue.clone(); }
	};
	futures::executor::block_on(async move {
		loop {
			let task = queue.pop().await;
			task.run().await;
		}
	});
}
