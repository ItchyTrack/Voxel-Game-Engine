use std::collections::HashMap;
use std::sync::{Mutex, OnceLock};

use bevy::prelude::*;

use voxel_data::voxels::Voxels;
use voxel_sources::{ChunkSource, GridKey, SourceHandle, VoxelSourcesAppExt};

use crate::lod_downsample::downsample_region;

const LOAD_COST: u32 = 0;

#[derive(Default)]
struct MemoryStore {
	chunks: Mutex<HashMap<(GridKey, IVec3), Voxels>>,
	handle: OnceLock<SourceHandle>,
}

impl ChunkSource for MemoryStore {
	fn init(&self, handle: SourceHandle) {
		let _ = self.handle.set(handle);
	}

	fn cost(&self, grid: GridKey, chunk: IVec3) -> Option<u32> {
		self.chunks.lock().unwrap().contains_key(&(grid, chunk)).then_some(LOAD_COST)
	}

	fn request_load(&self, grid: GridKey, chunk: IVec3) {
		let voxels = self.chunks.lock().unwrap().get(&(grid, chunk)).cloned();
		if let Some(handle) = self.handle.get() {
			handle.loaded(grid, chunk, voxels);
		}
	}

	fn cost_lod(&self, grid: GridKey, min: IVec3, size: IVec3, _lod: f32) -> Option<u32> {
		let chunks = self.chunks.lock().unwrap();
		let region_has_data = (0..size.z).any(|z| (0..size.y).any(|y| (0..size.x).any(|x| {
			chunks.contains_key(&(grid, min + IVec3::new(x, y, z)))
		})));
		region_has_data.then_some(LOAD_COST)
	}

	fn request_load_lod(&self, grid: GridKey, min: IVec3, size: IVec3, lod: f32) {
		let region = downsample_region(min, size, lod, |chunk| {
			self.chunks.lock().unwrap().get(&(grid, chunk)).cloned()
		});
		let voxels = (!region.is_empty()).then_some(region);
		if let Some(handle) = self.handle.get() {
			handle.loaded_lod(grid, min, size, lod, voxels);
		}
	}

	fn can_save(&self) -> bool {
		true
	}

	fn save(&self, grid: GridKey, chunk: IVec3, voxels: &Voxels) {
		let key = (grid, chunk);
		if voxels.is_empty() {
			self.chunks.lock().unwrap().remove(&key);
		} else {
			self.chunks.lock().unwrap().insert(key, voxels.clone());
		}
	}

	fn forget(&self, grid: GridKey, chunk: IVec3) {
		self.chunks.lock().unwrap().remove(&(grid, chunk));
	}
}

#[derive(Default)]
pub struct MemoryStorePlugin;

impl Plugin for MemoryStorePlugin {
	fn build(&self, app: &mut App) {
		app.register_source(MemoryStore::default());
	}
}
