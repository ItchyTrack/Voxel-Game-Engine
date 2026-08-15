mod handle;
mod request_processing;
mod source;
mod worker;

use std::sync::{Arc, RwLock};

use bevy::ecs::resource::Resource;
use bevy::math::IVec3;
use crossbeam_channel::{Receiver, Sender, unbounded};
use rustc_hash::FxHashMap;

use voxel_data::grid::GridId;
use voxel_data::voxels::{VoxelTypeId, Voxels};
use voxel_tasks::{AsyncTaskPriorityQueueResource, CancellationToken};
use handle::{SourceEditEmitter, SourceMessage, TakeContext, VoxelLodGenerators};
pub use handle::{ChunkPresence, ChunksEdited, SourceHandle, SourceId, VoxelLodGenerator};
use request_processing::{RequestProcessor, RequestState};
pub use source::{ChunkSource, SourceCoverage, TakeJob};
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
		generation: u64,
		voxels: Option<Voxels>,
	},
	VoxelsLoaded {
		grid: GridId,
		min: IVec3,
		size: IVec3,
		lod: f32,
		voxel_type: VoxelTypeId,
		generation: u64,
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
	edit_events: SourceEditEmitter,
	routing: Arc<RwLock<()>>,
	work_tx: Sender<SourceWork>,
	work_rx: Receiver<SourceWork>,
	completed: Vec<Completed>,
	source_presence: Vec<ChunkPresence>,
	edited: Vec<ChunksEdited>,
}

impl Default for SourceManager {
	fn default() -> Self {
		let (message_tx, message_rx) = unbounded();
		let edit_events = SourceEditEmitter::new(message_tx.clone());
		let (work_tx, work_rx) = unbounded();
		Self {
			sources: Vec::new(),
			generators: Arc::new(RwLock::new(FxHashMap::default())),
			message_tx,
			message_rx,
			request_state: RequestState::default(),
			edit_events,
			routing: Arc::new(RwLock::new(())),
			work_tx,
			work_rx,
			completed: Vec::new(),
			source_presence: Vec::new(),
			edited: Vec::new(),
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
		manager.routing.clone(),
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

	pub(crate) fn init_sources(&self, pusher: voxel_tasks::AsyncTaskPusher) {
		let take = TakeContext {
			sources: self.sources.clone().into(),
			routing: self.routing.clone(),
			pusher,
		};
		for (i, source) in self.sources.iter().enumerate() {
			source.init(SourceHandle {
				id: SourceId(i),
				messages: self.message_tx.clone(),
				lod_generators: self.generators.clone(),
				edits: self.edit_events.clone(),
				take: take.clone(),
			});
		}
	}

	/// Ask every registered source to publish its available area for `grid`.
	pub fn request_presence(&self, grid: GridId) {
		let _ = self.work_tx.send(SourceWork::Presence { grid });
	}

	/// Start loading one chunk and return the spatial generation captured by the request.
	pub fn request_chunk(&self, grid: GridId, chunk: IVec3, cancellation: CancellationToken) -> u64 {
		let generation = self.chunk_generation(grid, chunk);
		let _ = self.work_tx.send(SourceWork::Chunk { grid, chunk, generation, cancellation });
		generation
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
		let generation = self.region_generation(grid, min, size);
		let _ = self.work_tx.send(SourceWork::Voxels {
			grid,
			min,
			size,
			lod,
			voxel_type,
			priority,
			generation,
			cancellation,
		});
		generation
	}

	/// Persist an already-edited chunk. This transfers ownership but does not
	/// advance its generation or emit a command event.
	pub fn save_chunk(&self, grid: GridId, chunk: IVec3, generation: u64, voxels: &Voxels) {
		for (i, source) in self.sources.iter().enumerate() {
			if source.save(grid, chunk, generation, voxels) {
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


	fn collect_messages(&mut self) {
		self.request_state.retain_active();
		while let Ok(message) = self.message_rx.try_recv() {
			match message {
				SourceMessage::Presence(presence) => self.source_presence.push(presence),
				SourceMessage::Edited(edited) => self.edited.push(edited),
				SourceMessage::PresenceLoaded(grid) => {
					if self.request_state.finish_presence_load(grid) {
						self.completed.push(Completed::PresenceLoaded { grid });
					}
				}
				SourceMessage::Chunk(result) => self.completed.push(Completed::ChunkLoaded {
					grid: result.grid,
					chunk: result.chunk,
					generation: result.generation,
					voxels: result.voxels,
				}),
				SourceMessage::Transferred(result) => {
					let _ = (result.source, result.generation);
					self.sources[result.destination.0].receive_taken(result.grid, result.chunk, result.voxels);
				}
				SourceMessage::Voxels(result) => {
					let (grid, min, size, voxel_type) = (result.grid, result.region.min(), result.region.size().as_ivec3(), result.voxel_type);
					if let Some((lod, voxels, generation)) = self.request_state.take_voxels_completion(result) {
						self.completed.push(Completed::VoxelsLoaded {
							grid,
							min,
							size,
							lod,
							voxel_type,
							generation,
							voxels,
						});
					}
				}
			}
		}
	}


	pub fn current_generation(&self, grid: GridId) -> u64 {
		self.edit_events.current_generation(grid)
	}

	pub fn chunk_generation(&self, grid: GridId, chunk: IVec3) -> u64 {
		self.region_generation(grid, chunk, IVec3::ONE)
	}

	pub fn region_generation(&self, grid: GridId, min: IVec3, size: IVec3) -> u64 {
		self.edit_events.region_generation(grid, tile_data::ChunkRegion::new(min, size.as_uvec3()))
	}
}
