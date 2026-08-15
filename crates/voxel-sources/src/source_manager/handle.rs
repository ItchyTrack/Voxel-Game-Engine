use std::collections::HashMap;
use std::sync::{Arc, Mutex, RwLock};

use bevy::ecs::message::Message;
use bevy::math::IVec3;
use crossbeam_channel::Sender;
use rustc_hash::FxHashMap;
use voxel_edit::GridEdit;
use tile_data::ChunkRegion;
use crate::ChunkGenerationIndex;

use voxel_data::grid::GridId;
use voxel_data::voxels::{VoxelTypeId, Voxels};
use voxel_tasks::AsyncTaskPusher;

use super::source::SharedSource;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct SourceId(pub usize);

pub trait VoxelLodGenerator: Send + Sync {
	fn input_type_id(&self) -> VoxelTypeId;
	fn output_type_id(&self) -> VoxelTypeId;

	fn generate(
		&self,
		min: IVec3,
		size: IVec3,
		lod: f32,
		fetch: &dyn Fn(IVec3) -> Option<Voxels>,
	) -> Option<Voxels>;
}

pub(super) type VoxelLodGenerators = Arc<RwLock<FxHashMap<(VoxelTypeId, VoxelTypeId), Arc<dyn VoxelLodGenerator>>>>;

#[derive(Message, Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChunkPresence {
	pub grid: GridId,
	pub region: ChunkRegion,
}

/// One authoritative command, published once the owning source has recorded
/// it. Resident chunks may already contain it; unloaded chunks may replay it
/// when their data arrives.
#[derive(Message, Clone)]
pub struct ChunksEdited {
	pub source: SourceId,
	pub grid: GridId,
	pub region: ChunkRegion,
	pub generation: u64,
	pub edit: GridEdit,
}

pub(super) struct SourceChunkResult {
	pub grid: GridId,
	pub chunk: IVec3,
	pub generation: u64,
	pub voxels: Option<Voxels>,
}

pub(super) struct SourceTransferResult {
	pub source: SourceId,
	pub destination: SourceId,
	pub grid: GridId,
	pub chunk: IVec3,
	pub generation: u64,
	pub voxels: Option<Voxels>,
}

pub(super) struct SourceVoxelsResult {
	pub source: SourceId,
	pub grid: GridId,
	pub region: ChunkRegion,
	pub lod: f32,
	pub voxel_type: VoxelTypeId,
	pub generation: u64,
	pub voxels: Option<Voxels>,
}

pub(super) enum SourceMessage {
	Presence(ChunkPresence),
	Edited(ChunksEdited),
	PresenceLoaded(GridId),
	Chunk(SourceChunkResult),
	Transferred(SourceTransferResult),
	Voxels(SourceVoxelsResult),
}

#[derive(Default)]
struct SourceVersions {
	grid_generations: HashMap<GridId, u64>,
	chunk_generations: HashMap<GridId, ChunkGenerationIndex>,
}

#[derive(Clone)]
pub(super) struct SourceEditEmitter {
	versions: Arc<Mutex<SourceVersions>>,
	messages: Sender<SourceMessage>,
	emission_lock: Arc<Mutex<()>>,
}

impl SourceEditEmitter {
	pub(super) fn new(messages: Sender<SourceMessage>) -> Self {
		Self {
			versions: Arc::new(Mutex::new(SourceVersions::default())),
			messages,
			emission_lock: Arc::new(Mutex::new(())),
		}
	}

	pub(super) fn current_generation(&self, grid: GridId) -> u64 {
		self.versions.lock().unwrap().grid_generations.get(&grid).copied().unwrap_or(0)
	}

	pub(super) fn region_generation(&self, grid: GridId, region: ChunkRegion) -> u64 {
		self.versions.lock().unwrap().chunk_generations.get(&grid).map_or(0, |generations| generations.last_changed(region))
	}

	pub(super) fn synchronize_generation(&self, grid: GridId, generation: u64) {
		let _emission = self.emission_lock.lock().unwrap();
		let mut versions = self.versions.lock().unwrap();
		let current = versions.grid_generations.entry(grid).or_default();
		*current = (*current).max(generation);
	}

	pub(super) fn synchronize_region_generation(&self, grid: GridId, region: ChunkRegion, generation: u64) {
		let _emission = self.emission_lock.lock().unwrap();
		let mut versions = self.versions.lock().unwrap();
		let current = versions.grid_generations.entry(grid).or_default();
		*current = (*current).max(generation);
		versions.chunk_generations.entry(grid).or_default().set_region(region, generation);
	}

	fn edited(&self, source: SourceId, grid: GridId, min: IVec3, size: IVec3, edit: GridEdit) -> u64 {
		let _emission = self.emission_lock.lock().unwrap();
		let region = ChunkRegion::new(min, size.as_uvec3());
		let generation = {
			let mut versions = self.versions.lock().unwrap();
			let generation = versions.grid_generations.entry(grid).or_default();
			*generation += 1;
			let generation = *generation;
			versions.chunk_generations.entry(grid).or_default().set_region(region, generation);
			generation
		};
		let _ = self.messages.send(SourceMessage::Edited(ChunksEdited { source, grid, region, generation, edit }));
		generation
	}
}

#[derive(Clone)]
pub(super) struct TakeContext {
	pub sources: Arc<[SharedSource]>,
	pub routing: Arc<RwLock<()>>,
	pub pusher: AsyncTaskPusher,
}

#[derive(Clone)]
pub struct SourceHandle {
	pub(super) id: SourceId,
	pub(super) messages: Sender<SourceMessage>,
	pub(super) lod_generators: VoxelLodGenerators,
	pub(super) edits: SourceEditEmitter,
	pub(super) take: TakeContext,
}

impl SourceHandle {
	pub fn id(&self) -> SourceId { self.id }

	pub fn voxel_lod_generator(&self, input: VoxelTypeId, output: VoxelTypeId) -> Option<Arc<dyn VoxelLodGenerator>> {
		self.lod_generators.read().unwrap().get(&(input, output)).cloned()
	}

	/// Permanently take ownership of a chunk-space area.
	///
	/// Ownership moves to this source as soon as the request begins. The
	/// previous owner materializes each chunk and transfers its data
	/// asynchronously.
	pub fn take(&self, grid: GridId, min: IVec3, size: IVec3) {
		assert!(size.cmpgt(IVec3::ZERO).all(), "taken area size must be positive");
		let generation = self.region_generation(grid, min, size);
		super::request_processing::take_area(
			self.id,
			grid,
			min,
			size,
			generation,
			&self.take.sources,
			&self.take.routing,
			&self.take.pusher,
		);
	}

	pub fn current_generation(&self, grid: GridId) -> u64 { self.edits.current_generation(grid) }

	pub fn region_generation(&self, grid: GridId, min: IVec3, size: IVec3) -> u64 {
		self.edits.region_generation(grid, ChunkRegion::new(min, size.as_uvec3()))
	}

	pub fn synchronize_generation(&self, grid: GridId, generation: u64) {
		self.edits.synchronize_generation(grid, generation);
	}

	pub fn synchronize_region_generation(&self, grid: GridId, region: ChunkRegion, generation: u64) {
		self.edits.synchronize_region_generation(grid, region, generation);
	}

	pub fn edited(&self, grid: GridId, min: IVec3, size: IVec3, edit: GridEdit) -> u64 {
		self.edits.edited(self.id, grid, min, size, edit)
	}

	pub fn loaded(&self, grid: GridId, chunk: IVec3, generation: u64, voxels: Option<Voxels>) {
		let _ = self.messages.send(SourceMessage::Chunk(SourceChunkResult { grid, chunk, generation, voxels }));
	}

	pub fn transferred(&self, destination: SourceId, grid: GridId, chunk: IVec3, generation: u64, voxels: Option<Voxels>) {
		let _ = self.messages.send(SourceMessage::Transferred(SourceTransferResult {
			source: self.id,
			destination,
			grid,
			chunk,
			generation,
			voxels,
		}));
	}

	pub fn voxels_loaded(
		&self,
		grid: GridId,
		min: IVec3,
		size: IVec3,
		lod: f32,
		voxel_type: VoxelTypeId,
		generation: u64,
		voxels: Option<Voxels>,
	) {
		let _ = self.messages.send(SourceMessage::Voxels(SourceVoxelsResult {
			source: self.id,
			grid,
			region: ChunkRegion::new(min, size.as_uvec3()),
			lod,
			voxel_type,
			generation,
			voxels,
		}));
	}

	pub fn presence(&self, grid: GridId, min: IVec3, size: IVec3) {
		let _ = self.messages.send(SourceMessage::Presence(ChunkPresence { grid, region: ChunkRegion::new(min, size.as_uvec3()) }));
	}

	pub fn presence_loaded(&self, grid: GridId) {
		let _ = self.messages.send(SourceMessage::PresenceLoaded(grid));
	}
}
