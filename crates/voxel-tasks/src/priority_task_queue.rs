use std::{
	future::Future,
	pin::Pin,
	sync::{Mutex, OnceLock},
};

use async_priority_queue::PriorityQueue;
use bevy::{log::tracing::Instrument, tasks::AsyncComputeTaskPool};

pub struct PriorityTask {
	priority: f32,
	task_func: Pin<Box<dyn Future<Output = ()> + Send + 'static>>,
}

impl PriorityTask {
	pub fn new<F>(priority: f32, function: F) -> Self
	where
		F: Future<Output = ()> + Send + 'static,
	{
		Self {
			priority,
			task_func: Box::pin(function),
		}
	}
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

struct State {
	queue: PriorityQueue<PriorityTask>,
	active_tasks: usize,
}

impl State {
	fn new() -> Self {
		Self {
			queue: PriorityQueue::new(),
			active_tasks: 0,
		}
	}
}

pub struct AsyncPriorityTaskPool {
	state: Mutex<State>,
	max_concurrency: usize,
}

static ASYNC_PRIORITY_TASK_POOL: OnceLock<AsyncPriorityTaskPool> = OnceLock::new();

impl AsyncPriorityTaskPool {
	/// Creates a new priority task pool.
	pub fn new(max_concurrency: usize) -> Self {
		assert!(max_concurrency > 0);

		Self {
			state: Mutex::new(State::new()),
			max_concurrency,
		}
	}

	/// Gets the global [`AsyncPriorityTaskPool`] instance, or initializes it with `f`.
	pub fn get_or_init(
		f: impl FnOnce() -> AsyncPriorityTaskPool,
	) -> &'static Self {
		ASYNC_PRIORITY_TASK_POOL.get_or_init(f)
	}

	/// Attempts to get the global [`AsyncPriorityTaskPool`] instance.
	pub fn try_get() -> Option<&'static Self> {
		ASYNC_PRIORITY_TASK_POOL.get()
	}

	/// Gets the global [`AsyncPriorityTaskPool`] instance.
	///
	/// # Panics
	///
	/// Panics if the global instance has not been initialized yet.
	pub fn get() -> &'static Self {
		ASYNC_PRIORITY_TASK_POOL.get().expect(
			"The AsyncPriorityTaskPool has not been initialized yet. \
			 Please call AsyncPriorityTaskPool::get_or_init beforehand.",
		)
	}

	/// Spawns a future with the given priority.
	///
	/// Higher priority values are executed before lower priority values.
	pub fn spawn<F>(&'static self, priority: f32, future: F)
	where
		F: Future<Output = ()> + Send + 'static,
	{
		let task = PriorityTask::new(priority, future);

		let task = {
			let mut state = self.state.lock().unwrap();

			if state.active_tasks < self.max_concurrency {
				state.active_tasks += 1;
				Some(task)
			} else {
				state.queue.push(task);
				None
			}
		};

		if let Some(task) = task {
			self.spawn_task(task);
		}
	}

	fn spawn_task(&'static self, task: PriorityTask) {
		AsyncComputeTaskPool::get()
			.spawn(async move {
				task.task_func.await;
				self.task_finished();
			}.instrument(bevy::log::info_span!("priority_task")))
			.detach();
	}

	fn task_finished(&'static self) {
		let next_task = {
			let mut state = self.state.lock().unwrap();

			match state.queue.try_pop() {
				Some(task) => {
					// Transfer this active slot directly to the next task.
					Some(task)
				}
				None => {
					state.active_tasks -= 1;
					None
				}
			}
		};

		if let Some(task) = next_task {
			self.spawn_task(task);
		}
	}

	pub fn active_tasks(&self) -> usize {
		self.state.lock().unwrap().active_tasks
	}

	pub fn queued_tasks(&self) -> usize {
		self.state.lock().unwrap().queue.len()
	}

	pub fn len(&self) -> usize {
		let state = self.state.lock().unwrap();
		state.active_tasks + state.queue.len()
	}

	pub fn is_empty(&self) -> bool {
		self.len() == 0
	}
}

impl Default for AsyncPriorityTaskPool {
	fn default() -> Self {
		Self::new(AsyncComputeTaskPool::get().thread_num())
	}
}