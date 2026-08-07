use std::collections::HashMap;
use std::sync::{Arc, OnceLock, RwLock};

use bevy::prelude::*;

use voxel_data::grid::GridId;
use voxel_data::voxels::{VoxelTypeId, Voxels};
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

	fn cost_tile_voxels(&self, grid: GridId, min: IVec3, size: IVec3, lod: f32, voxel_type: VoxelTypeId) -> Option<u32> {
		let grids = self.inner.grids.read().unwrap();
		let store = grids.get(&grid)?;
		if !store.has_any_in_region(min, size) { return None; }
		let input = store.voxel_type_id_in_region(min, size)?;
		if lod <= 0.0 && input == voxel_type
			|| self.inner.handle.get()?.voxel_lod_generator(input, voxel_type).is_some()
		{
			Some(LOAD_COST)
		} else {
			None
		}
	}

	fn request_tile_voxels(
		&self,
		grid: GridId,
		min: IVec3,
		size: IVec3,
		lod: f32,
		voxel_type: VoxelTypeId,
		generation: u64,
		cancellation: CancellationToken,
	) {
		if cancellation.is_cancelled() { return; }
		let voxels = self.inner.handle.get().and_then(|handle| {
			let grids = self.inner.grids.read().unwrap();
			let store = grids.get(&grid)?;
			let input = store.voxel_type_id_in_region(min, size)?;
			if lod <= 0.0 && input == voxel_type {
				return store.load_lod_region(min, size, lod);
			}
			let generator = handle.voxel_lod_generator(input, voxel_type)?;
			generator.generate(min, size, lod, &|chunk| store.load_chunk(chunk))
		});
		if cancellation.is_cancelled() { return; }
		if let Some(handle) = self.inner.handle.get() {
			handle.loaded_tile_voxels(grid, min, size, lod, voxel_type, generation, voxels);
		}
	}

	fn request_available_area(&self, grid: GridId) {
		let Some(handle) = self.inner.handle.get() else { return };
		if let Some((min, size)) = self.grid_available_area(grid) {
			handle.claim(grid, min, size);
		}
		handle.presence_loaded(grid);
	}

	fn save(&self, grid: GridId, chunk: IVec3, voxels: &Voxels) -> bool {
		self.inner.grids.write().unwrap().entry(grid).or_default().save_chunk(chunk, voxels);
		true
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
