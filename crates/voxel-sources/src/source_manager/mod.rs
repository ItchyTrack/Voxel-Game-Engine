mod handle;
mod ownership;
mod request_processing;
mod source;
mod worker;

use std::collections::HashMap;
use std::sync::{Arc, Mutex, RwLock};
use std::sync::atomic::{AtomicU64, Ordering};

use bevy::ecs::resource::Resource;
use bevy::math::IVec3;
use crossbeam_channel::{Receiver, Sender, unbounded};
use rustc_hash::FxHashMap;

use voxel_data::grid::GridId;
use voxel_data::voxels::{VoxelTypeId, Voxels};
use voxel_tasks::{AsyncTaskPriorityQueueResource, CancellationToken};
use handle::{SourceEditEmitter, SourceMessage, VoxelLodGenerators};
pub use handle::{ChunkPresence, ChunksBorrowed, ChunksEdited, SourceHandle, SourceId, VoxelLodGenerator};
pub use ownership::LentChunks;
use request_processing::{RequestProcessor, RequestState};
pub use source::{ChunkSource, LendResult};
use source::SharedSource;
use worker::SourceWork;

#[derive(Debug, Clone)]
pub enum Completed {
	PresenceLoaded {
		grid: GridId,
	},
	ChunkLoaded {
		grid: GridId,
		chunk: IVec3,
		edit_index: u64,
		voxels: Option<Voxels>,
	},
	VoxelsLoaded {
		grid: GridId,
		min: IVec3,
		size: IVec3,
		lod: f32,
		voxel_type: VoxelTypeId,
		edit_index: u64,
		voxels: Option<Voxels>,
	},
}

#[derive(Resource)]
pub struct SourceManager {
	sources: Vec<SharedSource>,
	generators: VoxelLodGenerators,
	message_tx: Sender<SourceMessage>,
	message_rx: Receiver<SourceMessage>,
	request_state: RequestState,
	edit_indices: Arc<Mutex<HashMap<GridId, u64>>>,
	edit_events: SourceEditEmitter,
	next_borrow_request: AtomicU64,
	work_tx: Sender<SourceWork>,
	work_rx: Receiver<SourceWork>,
	completed: Vec<Completed>,
	source_presence: Vec<ChunkPresence>,
	edited: Vec<ChunksEdited>,
	borrowed: Vec<ChunksBorrowed>,
}

impl Default for SourceManager {
	fn default() -> Self {
		let (message_tx, message_rx) = unbounded();
		let edit_indices = Arc::new(Mutex::new(HashMap::new()));
		let edit_events = SourceEditEmitter::new(edit_indices.clone(), message_tx.clone());
		let (work_tx, work_rx) = unbounded();
		Self {
			sources: Vec::new(),
			generators: Arc::new(RwLock::new(FxHashMap::default())),
			message_tx,
			message_rx,
			request_state: RequestState::default(),
			edit_indices,
			edit_events,
			next_borrow_request: AtomicU64::new(1),
			work_tx,
			work_rx,
			completed: Vec::new(),
			source_presence: Vec::new(),
			edited: Vec::new(),
			borrowed: Vec::new(),
		}
	}
}

pub(crate) fn spawn_workers(
	mut commands: bevy::ecs::system::Commands,
	manager: bevy::ecs::system::Res<SourceManager>,
	async_queue: bevy::ecs::system::Res<AsyncTaskPriorityQueueResource>,
) {
	let processor = RequestProcessor::new(
		manager.sources.clone().into(),
		manager.request_state.clone(),
		async_queue.pusher(),
		manager.message_tx.clone(),
	);
	let requests = manager.work_rx.clone();
	commands.insert_resource(worker::start(requests, move |request| processor.process(request)));
}

impl SourceManager {
	pub(crate) fn push(&mut self, source: SharedSource) {
		self.sources.push(source);
	}

	pub(crate) fn register_lod_generator(&mut self, generator: Arc<dyn VoxelLodGenerator>) {
		self.generators.write().unwrap().insert((generator.input_type_id(), generator.output_type_id()), generator);
	}

	pub(crate) fn init_sources(&self) {
		for (i, source) in self.sources.iter().enumerate() {
			source.init(SourceHandle {
				id: SourceId(i),
				messages: self.message_tx.clone(),
				lod_generators: self.generators.clone(),
				edits: self.edit_events.clone(),
			});
		}
	}

	/// Ask every registered source to publish its available area for `grid`.
	pub fn request_presence(&self, grid: GridId) {
		let _ = self.work_tx.send(SourceWork::Presence { grid });
	}

	/// Start loading one chunk and return the source edit_index captured by the request.
	pub fn request_chunk(&self, grid: GridId, chunk: IVec3, cancellation: CancellationToken) -> u64 {
		let edit_index = self.current_edit_index(grid);
		let _ = self.work_tx.send(SourceWork::Chunk { grid, chunk, edit_index, cancellation });
		edit_index
	}

	/// Start loading voxel data for a chunk-space region.
	pub fn request_voxels(
		&self,
		grid: GridId,
		min: IVec3,
		size: IVec3,
		lod: f32,
		voxel_type: VoxelTypeId,
		priority: f32,
		cancellation: CancellationToken,
	) -> u64 {
		let edit_index = self.current_edit_index(grid);
		let _ = self.work_tx.send(SourceWork::Voxels {
			grid,
			min,
			size,
			lod,
			voxel_type,
			priority,
			edit_index,
			cancellation,
		});
		edit_index
	}

	/// Borrow a chunk-space area into `borrower`. The returned request ID is
	/// published in [`ChunksBorrowed`] when the complete area is ready.
	pub fn borrow_area(
		&self,
		borrower: SourceId,
		grid: GridId,
		min: IVec3,
		size: IVec3,
		cancellation: CancellationToken,
	) -> u64 {
		assert!(borrower.0 < self.sources.len(), "invalid borrower source ID");
		let request = self.next_borrow_request.fetch_add(1, Ordering::Relaxed);
		let edit_index = self.current_edit_index(grid);
		let _ = self.work_tx.send(SourceWork::Borrow { request, borrower, grid, min, size, edit_index, cancellation });
		request
	}

	/// Return a borrowed area without changing the retained owner data.
	pub fn return_area(&self, grid: GridId, min: IVec3, size: IVec3) {
		let _ = self.work_tx.send(SourceWork::Return { grid, min, size });
	}

	/// Persist an already-edited chunk. This transfers ownership but does not
	/// advance the edit index or emit an edit event.
	pub fn save_chunk(&self, grid: GridId, chunk: IVec3, edit_index: u64, voxels: &Voxels) {
		for (i, source) in self.sources.iter().enumerate() {
			if source.save(grid, chunk, edit_index, voxels) {
				for (j, other) in self.sources.iter().enumerate() {
					if j != i { other.forget(grid, chunk); }
				}
				return;
			}
		}
	}

	/// Drain all source operations completed since the previous call.
	pub fn get_completed(&mut self) -> Vec<Completed> {
		self.collect_messages();
		std::mem::take(&mut self.completed)
	}

	/// Drain source presence reports received since the previous call.
	pub fn get_source_presence(&mut self) -> Vec<ChunkPresence> {
		self.collect_messages();
		std::mem::take(&mut self.source_presence)
	}

	pub fn get_edited(&mut self) -> Vec<ChunksEdited> {
		self.collect_messages();
		std::mem::take(&mut self.edited)
	}

	pub fn get_borrowed(&mut self) -> Vec<ChunksBorrowed> {
		self.collect_messages();
		std::mem::take(&mut self.borrowed)
	}

	fn collect_messages(&mut self) {
		self.request_state.retain_active();
		while let Ok(message) = self.message_rx.try_recv() {
			match message {
				SourceMessage::Presence(presence) => self.source_presence.push(presence),
				SourceMessage::Edited(edited) => self.edited.push(edited),
				SourceMessage::Borrowed(borrowed) => self.borrowed.push(borrowed),
				SourceMessage::PresenceLoaded(grid) => {
					if self.request_state.finish_presence_load(grid) {
						self.completed.push(Completed::PresenceLoaded { grid });
					}
				}
				SourceMessage::Chunk(result) => self.completed.push(Completed::ChunkLoaded {
					grid: result.grid,
					chunk: result.chunk,
					edit_index: result.edit_index,
					voxels: result.voxels,
				}),
				SourceMessage::Voxels(result) => {
					let (grid, min, size, voxel_type) = (result.grid, result.region.min(), result.region.size().as_ivec3(), result.voxel_type);
					if let Some((lod, voxels, edit_index)) = self.request_state.take_voxels_completion(result) {
						self.completed.push(Completed::VoxelsLoaded {
							grid,
							min,
							size,
							lod,
							voxel_type,
							edit_index,
							voxels,
						});
					}
				}
			}
		}
	}

	pub fn current_edit_index(&self, grid: GridId) -> u64 {
		self.edit_indices.lock().unwrap().get(&grid).copied().unwrap_or(0)
	}
}
