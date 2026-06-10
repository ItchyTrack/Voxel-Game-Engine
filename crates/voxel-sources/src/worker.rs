use std::sync::Arc;
use std::thread::JoinHandle;

use bevy::ecs::resource::Resource;

use crossbeam_channel::Sender;

use tracy_client::span;
use voxel_data::task_queue::{AsyncTaskPriorityQueueResource, AsyncTaskPusher, PriorityTask};
use voxel_streaming::{ChunkRequestChannel, LodRequestChannel};

use crate::handle::{SourceLodResult, SourceResult};
use crate::registry::{cheapest, cheapest_lod, GridKeyMap, LodRequestKey, PendingLod, SharedSource, SourceRegistry};

/// Owns the serve worker threads so they live for the app's lifetime.
#[derive(Resource, Default)]
pub(crate) struct SourceWorkers {
	_threads: Vec<JoinHandle<()>>,
}

pub(crate) fn spawn_workers(
	mut commands: bevy::ecs::system::Commands,
	registry: bevy::ecs::system::Res<SourceRegistry>,
	chunk_requests: bevy::ecs::system::Res<ChunkRequestChannel>,
	lod_requests: bevy::ecs::system::Res<LodRequestChannel>,
	async_queue: bevy::ecs::system::Res<AsyncTaskPriorityQueueResource>,
) {
	let sources: Arc<[SharedSource]> = registry.sources.clone().into();
	let grid_keys = registry.grid_keys.clone();
	let pending_lod = registry.pending_lod.clone();
	let pusher = async_queue.pusher();

	let chunk_rx = chunk_requests.receiver();
	let lod_rx = lod_requests.receiver();

	let result_tx = registry.result_tx.clone();
	let lod_result_tx = registry.lod_result_tx.clone();

	let chunk_thread = {
		let sources = sources.clone();
		let grid_keys = grid_keys.clone();
		let pusher = pusher.clone();
		std::thread::Builder::new()
			.name("serve-requests".into())
			.spawn(move || serve_requests(chunk_rx, sources, grid_keys, pusher, result_tx))
			.unwrap()
	};

	let lod_thread = std::thread::Builder::new()
		.name("serve-lod-requests".into())
		.spawn(move || serve_lod_requests(lod_rx, sources, grid_keys, pending_lod, pusher, lod_result_tx))
		.unwrap();

	commands.insert_resource(SourceWorkers { _threads: vec![chunk_thread, lod_thread] });
}

fn serve_requests(
	requests: crossbeam_channel::Receiver<voxel_streaming::ChunkLoadRequest>,
	sources: Arc<[SharedSource]>,
	grid_keys: GridKeyMap,
	pusher: AsyncTaskPusher,
	result_tx: Sender<SourceResult>,
) {
	while let Ok(request) = requests.recv() {
		let Some(key) = grid_keys.read().unwrap().get(&request.grid).copied() else { continue };
		if let Some(id) = cheapest(&sources, key, request.chunk) {
			let source = sources[id.0].clone();
			pusher.push(PriorityTask::new(0.0, async move {
				let _zone = span!("source request_load chunk");
				source.request_load(key, request.chunk);
			}));
		} else {
			let _ = result_tx.send(SourceResult { grid: key, chunk: request.chunk, voxels: None });
		}
	}
}

fn serve_lod_requests(
	requests: crossbeam_channel::Receiver<voxel_streaming::LodLoadRequest>,
	sources: Arc<[SharedSource]>,
	grid_keys: GridKeyMap,
	pending_lod: PendingLod,
	pusher: AsyncTaskPusher,
	lod_result_tx: Sender<SourceLodResult>,
) {
	while let Ok(request) = requests.recv() {
		let Some(key) = grid_keys.read().unwrap().get(&request.grid).copied() else { continue };
		pending_lod
			.lock()
			.unwrap()
			.entry(LodRequestKey::new(key, request.min, request.size, request.lod))
			.or_default()
			.push(request);
		if let Some(id) = cheapest_lod(&sources, key, request.min, request.size, request.lod) {
			let source = sources[id.0].clone();
			pusher.push(PriorityTask::new(request.priority, async move {
				let _zone = span!("source request_load_lod");
				tracy_client::plot!("source lod request volume chunks", (request.size.x * request.size.y * request.size.z) as f64);
				source.request_load_lod(key, request.min, request.size, request.lod);
			}));
		} else {
			let _ = lod_result_tx.send(SourceLodResult {
				grid: key,
				min: request.min,
				size: request.size,
				lod: request.lod,
				voxels: None,
			});
		}
	}
}
