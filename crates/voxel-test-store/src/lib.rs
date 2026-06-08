use std::collections::HashMap;

use bevy::prelude::*;

use voxel_data::voxels::Voxels;
use voxel_sources::{ChunkSource, GridKey, SourceHandle, VoxelSourcesAppExt};

const LOAD_COST: u32 = 0;

#[derive(Default)]
struct MemoryStore {
	chunks: HashMap<(GridKey, IVec3), Voxels>,
	handle: Option<SourceHandle>,
}

impl ChunkSource for MemoryStore {
	fn init(&mut self, handle: SourceHandle) {
		self.handle = Some(handle);
	}

	fn cost(&self, grid: GridKey, chunk: IVec3) -> Option<u32> {
		self.chunks.contains_key(&(grid, chunk)).then_some(LOAD_COST)
	}

	fn request_load(&mut self, grid: GridKey, chunk: IVec3) {
		let voxels = self.chunks.get(&(grid, chunk)).cloned();
		if let Some(handle) = &self.handle {
			handle.loaded(grid, chunk, voxels);
		}
	}

	fn can_save(&self) -> bool {
		true
	}

	fn save(&mut self, grid: GridKey, chunk: IVec3, voxels: &Voxels) {
		let key = (grid, chunk);
		if voxels.is_empty() {
			self.chunks.remove(&key);
		} else {
			self.chunks.insert(key, voxels.clone());
		}
	}

	fn forget(&mut self, grid: GridKey, chunk: IVec3) {
		self.chunks.remove(&(grid, chunk));
	}
}

#[derive(Default)]
pub struct MemoryStorePlugin;

impl Plugin for MemoryStorePlugin {
	fn build(&self, app: &mut App) {
		app.register_source(MemoryStore::default());
	}
}
