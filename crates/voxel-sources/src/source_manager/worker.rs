use std::sync::Arc;
#[cfg(target_arch = "wasm32")]
use std::sync::OnceLock;
#[cfg(not(target_arch = "wasm32"))]
use std::thread::{self, JoinHandle};

use bevy::ecs::resource::Resource;
use bevy::math::IVec3;
use crossbeam_channel::Receiver;
#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

use voxel_data::grid::GridId;
use voxel_data::voxels::VoxelTypeId;

use super::SourceId;
use voxel_tasks::CancellationToken;

pub(super) enum SourceWork {
	Presence {
		grid: GridId,
	},
	Chunk {
		grid: GridId,
		chunk: IVec3,
		edit_index: u64,
		cancellation: CancellationToken,
	},
	Borrow {
		request: u64,
		borrower: SourceId,
		grid: GridId,
		min: IVec3,
		size: IVec3,
		edit_index: u64,
		cancellation: CancellationToken,
	},
	Return {
		grid: GridId,
		min: IVec3,
		size: IVec3,
	},
	Voxels {
		grid: GridId,
		min: IVec3,
		size: IVec3,
		lod: f32,
		voxel_type: VoxelTypeId,
		priority: f32,
		edit_index: u64,
		cancellation: CancellationToken,
	},
}

type WorkHandler = Arc<dyn Fn(SourceWork) + Send + Sync>;

#[cfg(target_arch = "wasm32")]
static WASM_REQUEST_RX: OnceLock<Receiver<SourceWork>> = OnceLock::new();
#[cfg(target_arch = "wasm32")]
static WASM_HANDLER: OnceLock<WorkHandler> = OnceLock::new();

/// Owns source-manager worker threads for the app's lifetime.
#[derive(Resource, Default)]
pub(super) struct SourceWorkers {
	#[cfg(not(target_arch = "wasm32"))]
	_threads: Vec<JoinHandle<()>>,
}

pub(super) fn start(
	requests: Receiver<SourceWork>,
	handle: impl Fn(SourceWork) + Send + Sync + 'static,
) -> SourceWorkers {
	let handler: WorkHandler = Arc::new(handle);

	#[cfg(target_arch = "wasm32")]
	{
		let _ = WASM_REQUEST_RX.set(requests);
		let _ = WASM_HANDLER.set(handler);
		SourceWorkers::default()
	}

	#[cfg(not(target_arch = "wasm32"))]
	{
		let worker = thread::Builder::new()
			.name("serve-source-requests".into())
			.spawn(move || run(requests, handler))
			.unwrap();
		SourceWorkers { _threads: vec![worker] }
	}
}

fn run(requests: Receiver<SourceWork>, handler: WorkHandler) {
	while let Ok(request) = requests.recv() {
		handler(request);
	}
}

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen]
pub fn voxel_sources_request_worker_loop(_worker_id: u32) {
	let requests = loop {
		if let Some(receiver) = WASM_REQUEST_RX.get() { break receiver.clone(); }
	};
	let handler = loop {
		if let Some(handler) = WASM_HANDLER.get() { break handler.clone(); }
	};
	run(requests, handler);
}
