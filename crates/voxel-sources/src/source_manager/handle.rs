use std::collections::HashMap;
use std::sync::{Arc, Mutex, RwLock};

use bevy::ecs::message::Message;
use bevy::math::IVec3;
use crossbeam_channel::Sender;
use rustc_hash::FxHashMap;
use voxel_edit::GridEdit;
use tile_data::ChunkRegion;

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

#[derive(Message, Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChunkPresence {
	pub grid: GridId,
	pub region: ChunkRegion,
}

/// One authoritative edit, published after every chunk in the area has been
/// mutated by the source that owns or borrows it.
#[derive(Message, Clone)]
pub struct ChunksEdited {
	pub source: SourceId,
	pub grid: GridId,
	pub region: ChunkRegion,
	pub edit_index: u64,
	pub edit: GridEdit,
}

#[derive(Message, Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChunksBorrowed {
	pub request: u64,
	pub borrower: SourceId,
	pub grid: GridId,
	pub region: ChunkRegion,
	pub edit_index: u64,
	pub success: bool,
}

pub(super) struct SourceChunkResult {
	pub grid: GridId,
	pub chunk: IVec3,
	pub edit_index: u64,
	pub voxels: Option<Voxels>,
}

pub(super) struct SourceVoxelsResult {
	pub source: SourceId,
	pub grid: GridId,
	pub region: ChunkRegion,
	pub lod: f32,
	pub voxel_type: VoxelTypeId,
	pub edit_index: u64,
	pub voxels: Option<Voxels>,
}

pub(super) enum SourceMessage {
	Presence(ChunkPresence),
	Edited(ChunksEdited),
	Borrowed(ChunksBorrowed),
	PresenceLoaded(GridId),
	Chunk(SourceChunkResult),
	Voxels(SourceVoxelsResult),
}

#[derive(Clone)]
pub(super) struct SourceEditEmitter {
	edit_indices: Arc<Mutex<HashMap<GridId, u64>>>,
	messages: Sender<SourceMessage>,
	emission_lock: Arc<Mutex<()>>,
}

impl SourceEditEmitter {
	pub(super) fn new(edit_indices: Arc<Mutex<HashMap<GridId, u64>>>, messages: Sender<SourceMessage>) -> Self {
		Self { edit_indices, messages, emission_lock: Arc::new(Mutex::new(())) }
	}

	fn current(&self, grid: GridId) -> u64 {
		self.edit_indices.lock().unwrap().get(&grid).copied().unwrap_or(0)
	}

	fn synchronize(&self, grid: GridId, edit_index: u64) {
		let _emission = self.emission_lock.lock().unwrap();
		let mut edit_indices = self.edit_indices.lock().unwrap();
		let current = edit_indices.entry(grid).or_default();
		*current = (*current).max(edit_index);
	}

	fn edited(&self, source: SourceId, grid: GridId, min: IVec3, size: IVec3, edit: GridEdit) -> u64 {
		let _emission = self.emission_lock.lock().unwrap();
		let edit_index = {
			let mut edit_indices = self.edit_indices.lock().unwrap();
			let edit_index = edit_indices.entry(grid).or_default();
			*edit_index += 1;
			*edit_index
		};
		let _ = self.messages.send(SourceMessage::Edited(ChunksEdited { source, grid, region: ChunkRegion::new(min, size.as_uvec3()), edit_index, edit }));
		edit_index
	}
}

#[derive(Clone)]
pub struct SourceHandle {
	pub(super) id: SourceId,
	pub(super) messages: Sender<SourceMessage>,
	pub(super) lod_generators: VoxelLodGenerators,
	pub(super) edits: SourceEditEmitter,
}

impl SourceHandle {
	pub fn id(&self) -> SourceId { self.id }

	pub fn voxel_lod_generator(&self, input: VoxelTypeId, output: VoxelTypeId) -> Option<Arc<dyn VoxelLodGenerator>> {
		self.lod_generators.read().unwrap().get(&(input, output)).cloned()
	}

	pub fn current_edit_index(&self, grid: GridId) -> u64 { self.edits.current(grid) }

	pub fn synchronize_edit_index(&self, grid: GridId, edit_index: u64) {
		self.edits.synchronize(grid, edit_index);
	}

	pub fn edited(&self, grid: GridId, min: IVec3, size: IVec3, edit: GridEdit) -> u64 {
		self.edits.edited(self.id, grid, min, size, edit)
	}

	pub fn loaded(&self, grid: GridId, chunk: IVec3, edit_index: u64, voxels: Option<Voxels>) {
		let _ = self.messages.send(SourceMessage::Chunk(SourceChunkResult { grid, chunk, edit_index, voxels }));
	}

	pub fn voxels_loaded(
		&self,
		grid: GridId,
		min: IVec3,
		size: IVec3,
		lod: f32,
		voxel_type: VoxelTypeId,
		edit_index: u64,
		voxels: Option<Voxels>,
	) {
		let _ = self.messages.send(SourceMessage::Voxels(SourceVoxelsResult {
			source: self.id,
			grid,
			region: ChunkRegion::new(min, size.as_uvec3()),
			lod,
			voxel_type,
			edit_index,
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
