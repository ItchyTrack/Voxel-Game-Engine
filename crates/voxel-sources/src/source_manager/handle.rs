use std::collections::HashMap;
use std::sync::{Arc, Mutex, RwLock};

use bevy::ecs::message::Message;
use bevy::math::IVec3;
use crossbeam_channel::Sender;
use rustc_hash::FxHashMap;

use voxel_data::grid::GridId;
use voxel_data::voxels::{VoxelTypeId, Voxels};

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChunkChangeKind {
	Changed { generation: u64 },
	Removed { generation: u64 },
}

#[derive(Message, Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChunkChanged {
	pub grid: GridId,
	pub min: IVec3,
	pub size: IVec3,
	pub kind: ChunkChangeKind,
	pub from_save: bool,
}

pub(super) struct SourceChunkResult {
	pub grid: GridId,
	pub chunk: IVec3,
	pub generation: u64,
	pub voxels: Option<Voxels>,
}

pub(super) struct SourceVoxelsResult {
	pub source: SourceId,
	pub grid: GridId,
	pub min: IVec3,
	pub size: IVec3,
	pub lod: f32,
	pub voxel_type: VoxelTypeId,
	pub generation: u64,
	pub voxels: Option<Voxels>,
}

pub(super) enum SourceMessage {
	Claim { source: SourceId, grid: GridId, min: IVec3, size: IVec3, generation: u64 },
	Unavailable { grid: GridId, min: IVec3, size: IVec3, generation: u64 },
	Changed(ChunkChanged),
	PresenceLoaded(GridId),
	Chunk(SourceChunkResult),
	Voxels(SourceVoxelsResult),
}

#[derive(Clone)]
pub(super) struct SourceChangeEmitter {
	generations: Arc<Mutex<HashMap<GridId, u64>>>,
	messages: Sender<SourceMessage>,
	emission_lock: Arc<Mutex<()>>,
}

impl SourceChangeEmitter {
	pub(super) fn new(generations: Arc<Mutex<HashMap<GridId, u64>>>, messages: Sender<SourceMessage>) -> Self {
		Self { generations, messages, emission_lock: Arc::new(Mutex::new(())) }
	}

	fn emit(&self, grid: GridId, message: impl FnOnce(u64) -> SourceMessage) -> u64 {
		let _emission = self.emission_lock.lock().unwrap();
		let generation = {
			let mut generations = self.generations.lock().unwrap();
			let generation = generations.entry(grid).or_default();
			*generation += 1;
			*generation
		};
		let _ = self.messages.send(message(generation));
		generation
	}

	fn claim(&self, source: SourceId, grid: GridId, min: IVec3, size: IVec3) -> u64 {
		self.emit(grid, |generation| SourceMessage::Claim { source, grid, min, size, generation })
	}

	fn unavailable(&self, grid: GridId, min: IVec3, size: IVec3) -> u64 {
		self.emit(grid, |generation| SourceMessage::Unavailable { grid, min, size, generation })
	}

	pub(super) fn changed(&self, grid: GridId, min: IVec3, size: IVec3, from_save: bool) -> u64 {
		self.emit(grid, |generation| SourceMessage::Changed(ChunkChanged {
			grid,
			min,
			size,
			kind: ChunkChangeKind::Changed { generation },
			from_save,
		}))
	}

	pub(super) fn removed(&self, grid: GridId, min: IVec3, size: IVec3, from_save: bool) -> u64 {
		self.emit(grid, |generation| SourceMessage::Changed(ChunkChanged {
			grid,
			min,
			size,
			kind: ChunkChangeKind::Removed { generation },
			from_save,
		}))
	}
}

#[derive(Clone)]
pub struct SourceHandle {
	pub(super) id: SourceId,
	pub(super) messages: Sender<SourceMessage>,
	pub(super) lod_generators: VoxelLodGenerators,
	pub(super) changes: SourceChangeEmitter,
}

impl SourceHandle {
	pub fn id(&self) -> SourceId {
		self.id
	}

	pub fn voxel_lod_generator(&self, input: VoxelTypeId, output: VoxelTypeId) -> Option<Arc<dyn VoxelLodGenerator>> {
		self.lod_generators.read().unwrap().get(&(input, output)).cloned()
	}

	pub fn loaded(&self, grid: GridId, chunk: IVec3, generation: u64, voxels: Option<Voxels>) {
		let _ = self.messages.send(SourceMessage::Chunk(SourceChunkResult { grid, chunk, generation, voxels }));
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
			min,
			size,
			lod,
			voxel_type,
			generation,
			voxels,
		}));
	}

	pub fn claim(&self, grid: GridId, min: IVec3, size: IVec3) -> u64 {
		self.changes.claim(self.id, grid, min, size)
	}

	pub fn unavailable(&self, grid: GridId, min: IVec3, size: IVec3) -> u64 {
		self.changes.unavailable(grid, min, size)
	}

	pub fn presence_loaded(&self, grid: GridId) {
		let _ = self.messages.send(SourceMessage::PresenceLoaded(grid));
	}
}
