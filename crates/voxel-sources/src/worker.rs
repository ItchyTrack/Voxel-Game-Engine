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
use voxel_tasks::{AsyncTaskPriorityQueueResource, AsyncTaskPusher, CancellationToken, PriorityTask};

use crate::loader::{GeneratedChunkLoadRequest, GeneratedLodLoadRequest, PresenceLoadRequest, SourceRequest};

use crate::handle::{SourceLodResult, SourceMessage, SourceChunkResult};
use crate::registry::{
	cheapest, lod_sources_with_any_chunks, ActivePresenceLoads, LodRequestKey, PendingLod,
	PendingLodJob, SharedSource, SourceRegistry,
};

#[cfg(target_arch = "wasm32")]
static WASM_SOURCES: OnceLock<Arc<[SharedSource]>> = OnceLock::new();
#[cfg(target_arch = "wasm32")]
static WASM_PENDING_LOD: OnceLock<PendingLod> = OnceLock::new();
#[cfg(target_arch = "wasm32")]
static WASM_ACTIVE_PRESENCE_LOADS: OnceLock<ActivePresenceLoads> = OnceLock::new();
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
		let _ = WASM_ACTIVE_PRESENCE_LOADS.set(registry.active_presence_loads.clone());
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
	active_presence_loads: &ActivePresenceLoads,
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
	if request.cancellation.is_cancelled() { return; }
	if let Some(id) = cheapest(sources, request.request.grid, request.request.chunk) {
		let source = sources[id.0].clone();
		let grid = request.request.grid;
		let chunk = request.request.chunk;
		let generation = request.generation;
		let cancellation = request.cancellation;
		pusher.push(PriorityTask::new(0.0, async move {
			if cancellation.is_cancelled() { return; }
			let _zone = span!("source request_load chunk");
			source.request_load(grid, chunk, generation, cancellation);
		}));
	} else if !request.cancellation.is_cancelled() {
		let _ = message_tx.send(SourceMessage::Chunk(SourceChunkResult {
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
	active_presence_loads: ActivePresenceLoads,
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
			SourceRequest::CancelLod { grid, key } => cancel_lod_request(grid, key, &pending_lod),
		}
	}
}

fn cancel_lod_request(grid: voxel_data::grid::GridId, key: crate::LodKey, pending_lod: &PendingLod) {
	let direct = LodRequestKey::new(grid, key.min, key.size, key.lod as f32);
	let intermediate = LodRequestKey::new(grid, key.min, key.size, (key.lod as f32).min(6.0));
	let mut pending = pending_lod.lock().unwrap();
	for candidate in [direct, intermediate] {
		let remove_job = if let Some(job) = pending.get_mut(&candidate) {
			let (requests, cancellation) = match job {
				PendingLodJob::Direct { requests, cancellation, .. }
				| PendingLodJob::Composite { requests, cancellation, .. } => (requests, cancellation),
			};
			requests.retain(|request| request.request.grid != grid || request.request.key != key);
			if requests.is_empty() {
				cancellation.cancel();
				true
			} else {
				false
			}
		} else {
			false
		};
		if remove_job {
			pending.remove(&candidate);
		}
		if candidate == intermediate { break; }
	}
}

fn handle_lod_request(
	request: GeneratedLodLoadRequest,
	sources: &[SharedSource],
	pending_lod: &PendingLod,
	pusher: &AsyncTaskPusher,
	message_tx: &Sender<SourceMessage>,
) {
	if request.cancellation.is_cancelled() { return; }
	let grid = request.request.grid;
	let min = request.request.key.min;
	let size = request.request.key.size;
	let lod = request.request.key.lod as f32;
	let priority = request.request.priority;
	let generation = request.generation;
	let source_ids = lod_sources_with_any_chunks(sources, grid, min, size, lod);
	match source_ids.as_slice() {
		[] => {
			let cancellation = CancellationToken::new();
			pending_lod.lock().unwrap().insert(
				LodRequestKey::new(grid, min, size, lod),
				PendingLodJob::Direct { requests: vec![request], generation, cancellation },
			);
			let _ = message_tx.send(SourceMessage::Lod(SourceLodResult {
				source: crate::source::SourceId(usize::MAX),
				grid,
				min,
				size,
				lod,
				generation,
				voxels: None,
			}));
		}
		[id] => {
			let lod_key = LodRequestKey::new(grid, min, size, lod);
			let (should_request, cancellation) = {
				let mut pending = pending_lod.lock().unwrap();
				match pending.get_mut(&lod_key) {
					Some(PendingLodJob::Direct { requests, cancellation, .. })
					| Some(PendingLodJob::Composite { requests, cancellation, .. }) => {
						requests.push(request);
						(false, cancellation.clone())
					}
					None => {
						let cancellation = CancellationToken::new();
						pending.insert(lod_key, PendingLodJob::Direct {
							requests: vec![request],
							generation,
							cancellation: cancellation.clone(),
						});
						(true, cancellation)
					}
				}
			};
			if should_request {
				let source = sources[id.0].clone();
				pusher.push(PriorityTask::new(priority, async move {
					if cancellation.is_cancelled() { return; }
					let _zone = span!("source request_load_lod direct");
					source.request_load_lod(grid, min, size, lod, generation, cancellation);
				}));
			}
		}
		_ => {
			let intermediate_lod = lod.min(6.0);
			let lod_key = LodRequestKey::new(grid, min, size, intermediate_lod);
			let (should_request, cancellation) = {
				let mut pending = pending_lod.lock().unwrap();
				match pending.get_mut(&lod_key) {
					Some(PendingLodJob::Composite { requests, cancellation, .. })
					| Some(PendingLodJob::Direct { requests, cancellation, .. }) => {
						requests.push(request);
						(false, cancellation.clone())
					}
					None => {
						let cancellation = CancellationToken::new();
						pending.insert(
							lod_key,
							PendingLodJob::Composite {
								requests: vec![request],
								expected: source_ids.iter().copied().collect(),
								received: Default::default(),
								final_lod: lod,
								intermediate_lod,
								generation,
								cancellation: cancellation.clone(),
							},
						);
						(true, cancellation)
					}
				}
			};
			if should_request {
				for id in source_ids {
					let source = sources[id.0].clone();
					let cancellation = cancellation.clone();
					pusher.push(PriorityTask::new(priority, async move {
						if cancellation.is_cancelled() { return; }
						let _zone = span!("source request_load_lod composite part");
						source.request_load_lod(grid, min, size, intermediate_lod, generation, cancellation);
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
	let active_presence_loads = loop {
		if let Some(v) = WASM_ACTIVE_PRESENCE_LOADS.get() { break v.clone(); }
	};
	let pusher = loop {
		if let Some(v) = WASM_PUSHER.get() { break v.clone(); }
	};
	let message_tx = loop {
		if let Some(v) = WASM_MESSAGE_TX.get() { break v.clone(); }
	};
	serve_requests(requests, sources, pending_lod, active_presence_loads, pusher, message_tx);
}
