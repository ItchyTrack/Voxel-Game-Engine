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

use crate::loader::{ChunkLoadRequest, LodLoadRequest, PresenceLoadRequest, SourceRequest};

use crate::handle::{SourceLodResult, SourceResult};
use crate::registry::{
	cheapest, lod_sources_with_any_chunks, LodRequestKey, PendingLod, PendingLodJob, SharedSource,
	SourceRegistry,
};

#[cfg(target_arch = "wasm32")]
static WASM_SOURCES: OnceLock<Arc<[SharedSource]>> = OnceLock::new();
#[cfg(target_arch = "wasm32")]
static WASM_PENDING_LOD: OnceLock<PendingLod> = OnceLock::new();
#[cfg(target_arch = "wasm32")]
static WASM_RESULT_TX: OnceLock<Sender<SourceResult>> = OnceLock::new();
#[cfg(target_arch = "wasm32")]
static WASM_LOD_RESULT_TX: OnceLock<Sender<SourceLodResult>> = OnceLock::new();
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
		let _ = WASM_RESULT_TX.set(registry.result_tx.clone());
		let _ = WASM_LOD_RESULT_TX.set(registry.lod_result_tx.clone());
		let _ = WASM_REQUEST_RX.set(registry.requests.receiver());
		let _ = WASM_PUSHER.set(async_queue.pusher());
		commands.insert_resource(SourceWorkers::default());
		return;
	}

	#[cfg(not(target_arch = "wasm32"))]
	{
		let sources: Arc<[SharedSource]> = registry.sources.clone().into();
		let pending_lod = registry.pending_lod.clone();
		let pusher = async_queue.pusher();

		let requests = registry.requests.receiver();

		let result_tx = registry.result_tx.clone();
		let lod_result_tx = registry.lod_result_tx.clone();

		let worker = thread::Builder::new()
			.name("serve-source-requests".into())
			.spawn(move || serve_requests(requests, sources, pending_lod, pusher, result_tx, lod_result_tx))
			.unwrap();

		commands.insert_resource(SourceWorkers { _threads: vec![worker] });
	}
}

fn handle_presence_request(
	request: PresenceLoadRequest,
	sources: &[SharedSource],
	pusher: &AsyncTaskPusher,
) {
	for source in sources {
		let source = source.clone();
		let grid = request.grid;
		pusher.push(PriorityTask::new(0.0, async move {
			source.request_available_area(grid);
		}));
	}
}

fn handle_chunk_request(
	request: ChunkLoadRequest,
	sources: &[SharedSource],
	pusher: &AsyncTaskPusher,
	result_tx: &Sender<SourceResult>,
) {
	if let Some(id) = cheapest(sources, request.grid, request.chunk) {
		let source = sources[id.0].clone();
		let grid = request.grid;
		pusher.push(PriorityTask::new(0.0, async move {
			let _zone = span!("source request_load chunk");
			source.request_load(grid, request.chunk);
		}));
	} else {
		let _ = result_tx.send(SourceResult { grid: request.grid, chunk: request.chunk, voxels: None });
	}
}

fn serve_requests(
	requests: Receiver<SourceRequest>,
	sources: Arc<[SharedSource]>,
	pending_lod: PendingLod,
	pusher: AsyncTaskPusher,
	result_tx: Sender<SourceResult>,
	lod_result_tx: Sender<SourceLodResult>,
) {
	while let Ok(request) = requests.recv() {
		match request {
			SourceRequest::Presence(request) => handle_presence_request(request, &sources, &pusher),
			SourceRequest::Chunk(request) => handle_chunk_request(request, &sources, &pusher, &result_tx),
			SourceRequest::Lod(request) => {
				handle_lod_request(request, &sources, &pending_lod, &pusher, &lod_result_tx)
			}
		}
	}
}

fn handle_lod_request(
	request: LodLoadRequest,
	sources: &[SharedSource],
	pending_lod: &PendingLod,
	pusher: &AsyncTaskPusher,
	lod_result_tx: &Sender<SourceLodResult>,
) {
	let source_ids =
		lod_sources_with_any_chunks(sources, request.grid, request.key.min, request.key.size, request.key.lod as f32);
	match source_ids.as_slice() {
		[] => {
			pending_lod.lock().unwrap().insert(
				LodRequestKey::new(request.grid, request.key.min, request.key.size, request.key.lod as f32),
				PendingLodJob::Direct { requests: vec![request] },
			);
			let _ = lod_result_tx.send(SourceLodResult {
				source: crate::source::SourceId(usize::MAX),
				grid: request.grid,
				min: request.key.min,
				size: request.key.size,
				lod: request.key.lod as f32,
				voxels: None,
			});
		}
		[id] => {
			let lod_key = LodRequestKey::new(request.grid, request.key.min, request.key.size, request.key.lod as f32);
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
				let grid = request.grid;
				pusher.push(PriorityTask::new(request.priority, async move {
					let _zone = span!("source request_load_lod direct");
					tracy_client::plot!(
						"source lod request volume chunks",
						(request.key.size.x * request.key.size.y * request.key.size.z) as f64
					);
					source.request_load_lod(grid, request.key.min, request.key.size, request.key.lod as f32);
				}));
			}
		}
		_ => {
			let intermediate_lod = (request.key.lod as f32).min(6.0);
			let lod_key = LodRequestKey::new(request.grid, request.key.min, request.key.size, intermediate_lod);
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
								final_lod: request.key.lod as f32,
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
					let grid = request.grid;
					pusher.push(PriorityTask::new(request.priority, async move {
						let _zone = span!("source request_load_lod composite part");
						tracy_client::plot!(
							"source lod request volume chunks",
							(request.key.size.x * request.key.size.y * request.key.size.z) as f64
						);
						source.request_load_lod(grid, request.key.min, request.key.size, intermediate_lod);
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
	let result_tx = loop {
		if let Some(v) = WASM_RESULT_TX.get() { break v.clone(); }
	};
	let lod_result_tx = loop {
		if let Some(v) = WASM_LOD_RESULT_TX.get() { break v.clone(); }
	};
	serve_requests(requests, sources, pending_lod, pusher, result_tx, lod_result_tx);
}
