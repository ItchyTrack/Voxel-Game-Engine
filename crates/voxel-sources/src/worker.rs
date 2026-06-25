use std::sync::Arc;
#[cfg(target_arch = "wasm32")]
use std::sync::OnceLock;
#[cfg(not(target_arch = "wasm32"))]
use std::thread;
#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;
#[cfg(target_arch = "wasm32")]
use wasm_thread as thread;
use thread::JoinHandle;

use bevy::ecs::resource::Resource;

use crossbeam_channel::{Receiver, Sender};

use tracy_client::span;
use voxel_data::task_queue::{AsyncTaskPriorityQueueResource, AsyncTaskPusher, PriorityTask};

use crate::loader::{GeneratedChunkLoadRequest, GeneratedLodLoadRequest, PresenceLoadRequest, SourceRequest};

use crate::handle::{SourceLodResult, SourceMessage, SourceResult};
use crate::registry::{
	cheapest, lod_sources_with_any_chunks, LodRequestKey, PendingLod, PendingLodJob, SharedSource,
	SourceRegistry,
};

#[cfg(target_arch = "wasm32")]
static WASM_SOURCES: OnceLock<Arc<[SharedSource]>> = OnceLock::new();
#[cfg(target_arch = "wasm32")]
static WASM_PENDING_LOD: OnceLock<PendingLod> = OnceLock::new();
#[cfg(target_arch = "wasm32")]
static WASM_MESSAGE_TX: OnceLock<Sender<SourceMessage>> = OnceLock::new();
#[cfg(target_arch = "wasm32")]
static WASM_REQUEST_RX: OnceLock<Receiver<SourceRequest>> = OnceLock::new();
#[cfg(target_arch = "wasm32")]
static WASM_PUSHER: OnceLock<AsyncTaskPusher> = OnceLock::new();

/// Owns the serve worker threads so they live for the app's lifetime.
#[derive(Resource, Default)]
pub(crate) struct SourceWorkers {
	_threads: Vec<JoinHandle<()>>,
}

pub(crate) fn spawn_workers(
	mut commands: bevy::ecs::system::Commands,
	registry: bevy::ecs::system::Res<SourceRegistry>,
	async_queue: bevy::ecs::system::Res<AsyncTaskPriorityQueueResource>,
) {
	#[cfg(target_arch = "wasm32")]
	{
		let sources: Arc<[SharedSource]> = registry.sources.clone().into();
		let _ = WASM_SOURCES.set(sources);
		let _ = WASM_PENDING_LOD.set(registry.pending_lod.clone());
		let _ = WASM_MESSAGE_TX.set(registry.message_tx.clone());
		let _ = WASM_REQUEST_RX.set(registry.requests.receiver());
		let _ = WASM_PUSHER.set(async_queue.pusher());
		commands.insert_resource(SourceWorkers::default());
		return;
	}

	#[cfg(not(target_arch = "wasm32"))]
	{
		let sources: Arc<[SharedSource]> = registry.sources.clone().into();
		let pending_lod = registry.pending_lod.clone();
		let active_presence_loads = registry.active_presence_loads.clone();
		let pusher = async_queue.pusher();

		let requests = registry.requests.receiver();

		let message_tx = registry.message_tx.clone();

		let worker = thread::Builder::new()
			.name("serve-source-requests".into())
			.spawn(move || serve_requests(requests, sources, pending_lod, active_presence_loads, pusher, message_tx))
			.unwrap();

		commands.insert_resource(SourceWorkers { _threads: vec![worker] });
	}
}

fn handle_presence_request(
	request: PresenceLoadRequest,
	sources: &[SharedSource],
	active_presence_loads: &std::sync::Arc<std::sync::Mutex<std::collections::HashMap<voxel_data::grid::GridId, u32>>>,
	pusher: &AsyncTaskPusher,
) {
	let count = sources.len() as u32;
	if count == 0 { return; }
	*active_presence_loads.lock().unwrap().entry(request.grid).or_default() += count;
	for source in sources {
		let source = source.clone();
		let grid = request.grid;
		pusher.push(PriorityTask::new(0.0, async move {
			source.request_available_area(grid);
		}));
	}
}

fn handle_chunk_request(
	request: GeneratedChunkLoadRequest,
	sources: &[SharedSource],
	pusher: &AsyncTaskPusher,
	message_tx: &Sender<SourceMessage>,
) {
	if let Some(id) = cheapest(sources, request.request.grid, request.request.chunk) {
		let source = sources[id.0].clone();
		let grid = request.request.grid;
		let chunk = request.request.chunk;
		let generation = request.generation;
		pusher.push(PriorityTask::new(0.0, async move {
			let _zone = span!("source request_load chunk");
			source.request_load(grid, chunk, generation);
		}));
	} else {
		let _ = message_tx.send(SourceMessage::Chunk(SourceResult {
			grid: request.request.grid,
			chunk: request.request.chunk,
			generation: request.generation,
			voxels: None,
		}));
	}
}

fn serve_requests(
	requests: Receiver<SourceRequest>,
	sources: Arc<[SharedSource]>,
	pending_lod: PendingLod,
	active_presence_loads: std::sync::Arc<std::sync::Mutex<std::collections::HashMap<voxel_data::grid::GridId, u32>>>,
	pusher: AsyncTaskPusher,
	message_tx: Sender<SourceMessage>,
) {
	while let Ok(request) = requests.recv() {
		match request {
			SourceRequest::Presence(request) => handle_presence_request(request, &sources, &active_presence_loads, &pusher),
			SourceRequest::Chunk(request) => handle_chunk_request(request, &sources, &pusher, &message_tx),
			SourceRequest::Lod(request) => {
				handle_lod_request(request, &sources, &pending_lod, &pusher, &message_tx)
			}
		}
	}
}

fn handle_lod_request(
	request: GeneratedLodLoadRequest,
	sources: &[SharedSource],
	pending_lod: &PendingLod,
	pusher: &AsyncTaskPusher,
	message_tx: &Sender<SourceMessage>,
) {
	let source_ids =
		lod_sources_with_any_chunks(sources, request.request.grid, request.request.key.min, request.request.key.size, request.request.key.lod as f32);
	match source_ids.as_slice() {
		[] => {
			pending_lod.lock().unwrap().insert(
				LodRequestKey::new(request.request.grid, request.request.key.min, request.request.key.size, request.request.key.lod as f32),
				PendingLodJob::Direct { requests: vec![request] },
			);
			let _ = message_tx.send(SourceMessage::Lod(SourceLodResult {
				source: crate::source::SourceId(usize::MAX),
				grid: request.request.grid,
				min: request.request.key.min,
				size: request.request.key.size,
				lod: request.request.key.lod as f32,
				generation: request.generation,
				voxels: None,
			}));
		}
		[id] => {
			let lod_key = LodRequestKey::new(request.request.grid, request.request.key.min, request.request.key.size, request.request.key.lod as f32);
			let should_request = {
				let mut pending = pending_lod.lock().unwrap();
				match pending.get_mut(&lod_key) {
					Some(PendingLodJob::Direct { requests }) => {
						requests.push(request);
						false
					}
					Some(PendingLodJob::Composite { requests, .. }) => {
						requests.push(request);
						false
					}
					None => {
						pending.insert(lod_key, PendingLodJob::Direct { requests: vec![request.clone()] });
						true
					}
				}
			};
			if should_request {
				let source = sources[id.0].clone();
				let request = request.clone();
				let grid = request.request.grid;
				let generation = request.generation;
				pusher.push(PriorityTask::new(request.request.priority, async move {
					let _zone = span!("source request_load_lod direct");
					// tracy_client::plot!(
					// 	"source lod request volume chunks",
					// 	(request.request.key.size.x * request.request.key.size.y * request.request.key.size.z) as f64
					// );
					source.request_load_lod(grid, request.request.key.min, request.request.key.size, request.request.key.lod as f32, generation);
				}));
			}
		}
		_ => {
			let intermediate_lod = (request.request.key.lod as f32).min(6.0);
			let lod_key = LodRequestKey::new(request.request.grid, request.request.key.min, request.request.key.size, intermediate_lod);
			let should_request = {
				let mut pending = pending_lod.lock().unwrap();
				match pending.get_mut(&lod_key) {
					Some(PendingLodJob::Composite { requests, .. }) => {
						requests.push(request.clone());
						false
					}
					Some(PendingLodJob::Direct { requests }) => {
						requests.push(request.clone());
						false
					}
					None => {
						pending.insert(
							lod_key,
							PendingLodJob::Composite {
								requests: vec![request.clone()],
								expected: source_ids.iter().copied().collect(),
								received: Default::default(),
								final_lod: request.request.key.lod as f32,
								intermediate_lod,
							},
						);
						true
					}
				}
			};
			if should_request {
				for id in source_ids {
					let source = sources[id.0].clone();
					let request = request.clone();
					let grid = request.request.grid;
					let generation = request.generation;
					pusher.push(PriorityTask::new(request.request.priority, async move {
						let _zone = span!("source request_load_lod composite part");
						// tracy_client::plot!(
						// 	"source lod request volume chunks",
						// 	(request.request.key.size.x * request.request.key.size.y * request.request.key.size.z) as f64
						// );
						source.request_load_lod(grid, request.request.key.min, request.request.key.size, intermediate_lod, generation);
					}));
				}
			}
		}
	}
}

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen]
pub fn voxel_sources_request_worker_loop(_worker_id: u32) {
	let requests = loop {
		if let Some(rx) = WASM_REQUEST_RX.get() { break rx.clone(); }
	};
	let sources = loop {
		if let Some(s) = WASM_SOURCES.get() { break s.clone(); }
	};
	let pending_lod = loop {
		if let Some(v) = WASM_PENDING_LOD.get() { break v.clone(); }
	};
	let pusher = loop {
		if let Some(v) = WASM_PUSHER.get() { break v.clone(); }
	};
	let message_tx = loop {
		if let Some(v) = WASM_MESSAGE_TX.get() { break v.clone(); }
	};
	serve_requests(requests, sources, pending_lod, pusher, message_tx);
}
