mod handle;
mod request_processing;
mod source;
mod worker;

use std::collections::HashMap;
use std::sync::{Arc, Mutex, RwLock};

use bevy::ecs::resource::Resource;
use bevy::math::IVec3;
use crossbeam_channel::{Receiver, Sender, unbounded};
use rustc_hash::FxHashMap;

use voxel_data::grid::GridId;
use voxel_data::voxels::{VoxelTypeId, Voxels};
use voxel_tasks::{AsyncTaskPriorityQueueResource, CancellationToken};

use handle::{SourceChangeEmitter, SourceMessage, VoxelLodGenerators};
pub use handle::{ChunkChangeKind, ChunkChanged, SourceHandle, SourceId, VoxelLodGenerator};
use request_processing::{RequestProcessor, RequestState};
pub use source::ChunkSource;
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
	changes: SourceChangeEmitter,
	request_state: RequestState,
	generations: Arc<Mutex<HashMap<GridId, u64>>>,
	work_tx: Sender<SourceWork>,
	work_rx: Receiver<SourceWork>,
	completed: Vec<Completed>,
	source_changes: Vec<ChunkChanged>,
}

impl Default for SourceManager {
	fn default() -> Self {
		let (message_tx, message_rx) = unbounded();
		let generations = Arc::new(Mutex::new(HashMap::new()));
		let (work_tx, work_rx) = unbounded();
		Self {
			sources: Vec::new(),
			generators: Arc::new(RwLock::new(FxHashMap::default())),
			changes: SourceChangeEmitter::new(generations.clone(), message_tx.clone()),
			message_tx,
			message_rx,
			request_state: RequestState::default(),
			generations,
			work_tx,
			work_rx,
			completed: Vec::new(),
			source_changes: Vec::new(),
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
				changes: self.changes.clone(),
			});
		}
	}

	/// Ask every registered source to publish its available area for `grid`.
	pub fn request_presence(&self, grid: GridId) {
		let _ = self.work_tx.send(SourceWork::Presence { grid });
	}

	/// Start loading one chunk and return the source generation captured by the request.
	pub fn request_chunk(&self, grid: GridId, chunk: IVec3, cancellation: CancellationToken) -> u64 {
		let generation = self.current_generation(grid);
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
		let generation = self.current_generation(grid);
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

	/// Save a chunk through the first writable source.
	pub fn save_chunk(&self, grid: GridId, chunk: IVec3, voxels: &Voxels) {
		if voxels.is_empty() {
			for source in &self.sources {
				source.forget(grid, chunk);
			}
			self.changes.removed(grid, chunk, IVec3::ONE, true);
			return;
		}
		for (i, source) in self.sources.iter().enumerate() {
			if source.save(grid, chunk, voxels) {
				self.forget_others(SourceId(i), grid, chunk, IVec3::ONE);
				self.changes.changed(grid, chunk, IVec3::ONE, true);
				break;
			}
		}
	}

	fn forget_others(&self, keep: SourceId, grid: GridId, min: IVec3, size: IVec3) {
		for (i, source) in self.sources.iter().enumerate() {
			if i != keep.0 {
				for x in min.x..min.x + size.x {
					for y in min.y..min.y + size.y {
						for z in min.z..min.z + size.z {
							source.forget(grid, IVec3::new(x, y, z));
						}
					}
				}
			}
		}
	}

	/// Drain all source operations completed since the previous call.
	pub fn get_completed(&mut self) -> Vec<Completed> {
		self.collect_messages();
		std::mem::take(&mut self.completed)
	}

	/// Drain all source-change notifications received since the previous call.
	pub fn get_source_changes(&mut self) -> Vec<ChunkChanged> {
		self.collect_messages();
		std::mem::take(&mut self.source_changes)
	}

	fn collect_messages(&mut self) {
		self.request_state.retain_active();
		while let Ok(message) = self.message_rx.try_recv() {
			match message {
				SourceMessage::Claim { source, grid, min, size, generation } => {
					self.forget_others(source, grid, min, size);
					self.source_changes.push(ChunkChanged {
						grid,
						min,
						size,
						kind: ChunkChangeKind::Changed { generation },
						from_save: false,
					});
				}
				SourceMessage::Unavailable { grid, min, size, generation } => {
					self.source_changes.push(ChunkChanged {
						grid,
						min,
						size,
						kind: ChunkChangeKind::Removed { generation },
						from_save: false,
					});
				}
				SourceMessage::Changed(change) => self.source_changes.push(change),
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
				SourceMessage::Voxels(result) => {
					let (grid, min, size, voxel_type) = (result.grid, result.min, result.size, result.voxel_type);
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

	fn current_generation(&self, grid: GridId) -> u64 {
		self.generations.lock().unwrap().get(&grid).copied().unwrap_or(0)
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn requests_capture_the_current_generation_without_advancing_it() {
		let manager = SourceManager::default();
		manager.request_chunk(GridId::PLACEHOLDER, IVec3::new(1, 2, 3), CancellationToken::new());
		manager.request_chunk(GridId::PLACEHOLDER, IVec3::new(4, 5, 6), CancellationToken::new());
		assert_eq!(manager.current_generation(GridId::PLACEHOLDER), 0);

		manager.changes.changed(GridId::PLACEHOLDER, IVec3::ZERO, IVec3::ONE, false);
		assert_eq!(manager.current_generation(GridId::PLACEHOLDER), 1);
	}
}
