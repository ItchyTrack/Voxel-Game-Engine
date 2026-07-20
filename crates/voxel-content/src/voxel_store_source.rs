use std::collections::HashMap;
use std::sync::{Arc, OnceLock, RwLock};

use bevy::prelude::*;

use voxel_data::grid::GridId;
use voxel_data::voxels::Voxels;
use voxel_sources::{CancellationToken, ChunkSource, SourceHandle, VoxelSourcesAppExt};

use crate::GridStore;

#[derive(Default)]
struct VoxelStoreSourceInner {
	grids: RwLock<HashMap<GridId, GridStore>>,
	handle: OnceLock<SourceHandle>,
}

#[derive(Resource, Clone, Default)]
pub struct VoxelStoreSource {
	inner: Arc<VoxelStoreSourceInner>,
}

impl VoxelStoreSource {
	pub fn insert_chunk_data(&self, grid: GridId, chunk_data: HashMap<IVec3, Voxels>) {
		let mut grids = self.inner.grids.write().unwrap();
		let store = grids.entry(grid).or_default();
		for (chunk, voxels) in chunk_data {
			store.save_chunk(chunk, &voxels);
		}
	}

	fn grid_available_area(&self, grid: GridId) -> Option<(IVec3, IVec3)> {
		self.inner.grids.read().unwrap().get(&grid)?.available_area()
	}
}

const LOAD_COST: u32 = 0;


impl ChunkSource for VoxelStoreSource {
	fn init(&self, handle: SourceHandle) {
		let _ = self.inner.handle.set(handle);
	}

	fn cost(&self, grid: GridId, chunk: IVec3) -> Option<u32> {
		self.inner.grids.read().unwrap().get(&grid)?.contains_chunk(chunk).then_some(LOAD_COST)
	}

	fn request_load(&self, grid: GridId, chunk: IVec3, generation: u64, cancellation: CancellationToken) {
		if cancellation.is_cancelled() { return; }
		let voxels = self.inner.grids.read().unwrap().get(&grid).and_then(|store| store.load_chunk(chunk));
		if cancellation.is_cancelled() { return; }
		if let Some(handle) = self.inner.handle.get() {
			handle.loaded(grid, chunk, generation, voxels);
		}
	}

	fn cost_lod(&self, grid: GridId, min: IVec3, size: IVec3, _lod: f32) -> Option<u32> {
		self.inner.grids.read().unwrap().get(&grid)?.has_any_in_region(min, size).then_some(LOAD_COST)
	}

	fn request_load_lod(&self, grid: GridId, min: IVec3, size: IVec3, lod: f32, generation: u64, cancellation: CancellationToken) {
		if cancellation.is_cancelled() { return; }
		let voxels = self.inner.handle.get().and_then(|handle| {
			let grids = self.inner.grids.read().unwrap();
			let store = grids.get(&grid)?;
			if lod <= 0.0 {
				return store.load_lod_region(min, size, lod);
			}
			let type_id = store.voxel_type_id_in_region(min, size)?;
			let generator = handle.voxel_lod_generator(type_id)?;
			generator.generate(min, size, lod, &|chunk| store.load_chunk(chunk))
		});
		if cancellation.is_cancelled() { return; }
		if let Some(handle) = self.inner.handle.get() {
			handle.loaded_lod(grid, min, size, lod, generation, voxels);
		}
	}

	fn request_available_area(&self, grid: GridId) {
		let Some(handle) = self.inner.handle.get() else { return };
		if let Some((min, size)) = self.grid_available_area(grid) {
			handle.claim(grid, min, size);
		}
		handle.presence_loaded(grid);
	}

	fn can_save(&self) -> bool {
		true
	}

	fn save(&self, grid: GridId, chunk: IVec3, voxels: &Voxels) {
		self.inner.grids.write().unwrap().entry(grid).or_default().save_chunk(chunk, voxels);
	}

	fn forget(&self, grid: GridId, chunk: IVec3) {
		if let Some(store) = self.inner.grids.write().unwrap().get_mut(&grid) {
			store.forget_chunk(chunk);
		}
	}
}

#[derive(Default)]
pub struct VoxelStoreSourcePlugin;

impl Plugin for VoxelStoreSourcePlugin {
	fn build(&self, app: &mut App) {
		let source = VoxelStoreSource::default();
		app.insert_resource(source.clone());
		app.register_voxel_source(source);
	}
}
