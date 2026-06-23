use std::collections::HashMap;
use std::sync::{Mutex, OnceLock};

use bevy::prelude::*;

use voxel_data::grid::GridId;
use voxel_data::voxels::Voxels;
use voxel_sources::{ChunkSource, SourceHandle, VoxelSourcesAppExt};

use crate::voxel::lod_downsample::downsample_region;

const LOAD_COST: u32 = 0;

#[derive(Default)]
struct MemoryStore {
	chunks: Mutex<HashMap<(GridId, IVec3), Voxels>>,
	handle: OnceLock<SourceHandle>,
}

impl MemoryStore {
	fn available_area(&self, grid: GridId) -> Option<(IVec3, IVec3)> {
		let chunks = self.chunks.lock().unwrap();
		let mut iter = chunks.keys().filter_map(|(key, chunk)| (*key == grid).then_some(*chunk));
		let first = iter.next()?;
		let (mut min, mut max) = (first, first);
		for chunk in iter {
			min = min.min(chunk);
			max = max.max(chunk);
		}
		Some((min, max - min + IVec3::ONE))
	}
}

impl ChunkSource for MemoryStore {
	fn init(&self, handle: SourceHandle) {
		let _ = self.handle.set(handle);
	}

	fn cost(&self, grid: GridId, chunk: IVec3) -> Option<u32> {
		self.chunks.lock().unwrap().contains_key(&(grid, chunk)).then_some(LOAD_COST)
	}

	fn request_load(&self, grid: GridId, chunk: IVec3, generation: u64) {
		let voxels = self.chunks.lock().unwrap().get(&(grid, chunk)).cloned();
		if let Some(handle) = self.handle.get() {
			handle.loaded(grid, chunk, generation, voxels);
		}
	}

	fn cost_lod(&self, grid: GridId, min: IVec3, size: IVec3, _lod: f32) -> Option<u32> {
		let chunks = self.chunks.lock().unwrap();
		let region_has_data = (0..size.z).any(|z| (0..size.y).any(|y| (0..size.x).any(|x| {
			chunks.contains_key(&(grid, min + IVec3::new(x, y, z)))
		})));
		region_has_data.then_some(LOAD_COST)
	}

	fn request_load_lod(&self, grid: GridId, min: IVec3, size: IVec3, lod: f32, generation: u64) {
		let region = downsample_region(min, size, lod, |chunk| {
			self.chunks.lock().unwrap().get(&(grid, chunk)).cloned()
		});
		let voxels = (!region.is_empty()).then_some(region);
		if let Some(handle) = self.handle.get() {
			handle.loaded_lod(grid, min, size, lod, generation, voxels);
		}
	}

	fn request_available_area(&self, grid: GridId) {
		let Some(handle) = self.handle.get() else { return };
		if let Some((min, size)) = self.available_area(grid) {
			handle.claim(grid, min, size);
		}
		handle.presence_loaded(grid);
	}

	fn can_save(&self) -> bool {
		true
	}

	fn save(&self, grid: GridId, chunk: IVec3, voxels: &Voxels) {
		let key = (grid, chunk);
		if voxels.is_empty() {
			self.chunks.lock().unwrap().remove(&key);
		} else {
			self.chunks.lock().unwrap().insert(key, voxels.clone());
		}
	}

	fn forget(&self, grid: GridId, chunk: IVec3) {
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
