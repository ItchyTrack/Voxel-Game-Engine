use std::collections::HashMap;
use std::sync::{Arc, OnceLock, RwLock};

use bevy::prelude::*;

use voxel_data::grid::GridId;
use voxel_data::voxels::{VoxelTypeId, Voxels};
use voxel_sources::{CancellationToken, ChunkSource, LendResult, LentChunks, SourceHandle, VoxelSourcesAppExt};

use crate::GridStore;

#[derive(Default)]
struct VoxelStoreSourceInner {
	grids: RwLock<HashMap<GridId, GridStore>>,
	handle: OnceLock<SourceHandle>,
	lent: LentChunks,
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

	fn grid_available_area(&self, grid: GridId) -> Option<tile_data::ChunkRegion> {
		self.inner.grids.read().unwrap().get(&grid)?.available_area()
	}
}

const LOAD_COST: u32 = 0;


impl ChunkSource for VoxelStoreSource {
	fn init(&self, handle: SourceHandle) {
		let _ = self.inner.handle.set(handle);
	}

	fn cost(&self, grid: GridId, chunk: IVec3) -> Option<u32> {
		(!self.inner.lent.contains(grid, chunk) && self.inner.grids.read().unwrap().get(&grid)?.contains_chunk(chunk)).then_some(LOAD_COST)
	}

	fn request_load(&self, grid: GridId, chunk: IVec3, edit_index: u64, cancellation: CancellationToken) -> bool {
		if cancellation.is_cancelled() || self.inner.lent.contains(grid, chunk) { return false; }
		let voxels = self.inner.grids.read().unwrap().get(&grid).and_then(|store| store.load_chunk(chunk));
		if cancellation.is_cancelled() || self.inner.lent.contains(grid, chunk) { return false; }
		if let Some(handle) = self.inner.handle.get() { handle.loaded(grid, chunk, edit_index, voxels); }
		true
	}

	fn cost_voxels(&self, grid: GridId, min: IVec3, size: IVec3, lod: f32, voxel_type: VoxelTypeId) -> Option<u32> {
		let grids = self.inner.grids.read().unwrap();
		let store = grids.get(&grid)?;
		let Some(input) = store.voxel_type_id() else { return None };
		assert!(
			lod <= 0.0 && input == voxel_type
				|| self.inner.handle.get().and_then(|handle| handle.voxel_lod_generator(input, voxel_type)).is_some(),
			"voxel store source does not support requested voxel type or LOD",
		);
		let any_available = (0..size.z).any(|z| (0..size.y).any(|y| (0..size.x).any(|x| {
			let chunk = min + IVec3::new(x, y, z);
			!self.inner.lent.contains(grid, chunk) && store.contains_chunk(chunk)
		})));
		any_available.then_some(LOAD_COST)
	}

	fn request_voxel_area(
		&self,
		grid: GridId,
		min: IVec3,
		size: IVec3,
		lod: f32,
		voxel_type: VoxelTypeId,
		edit_index: u64,
		cancellation: CancellationToken,
	) -> bool {
		if cancellation.is_cancelled() || !self.inner.lent.any_available_in(grid, min, size) { return false; }
		let voxels = self.inner.handle.get().and_then(|handle| {
			let grids = self.inner.grids.read().unwrap();
			let store = grids.get(&grid)?;
			let input = (0..size.z).find_map(|z| (0..size.y).find_map(|y| (0..size.x).find_map(|x| {
				let chunk = min + IVec3::new(x, y, z);
				(!self.inner.lent.contains(grid, chunk)).then(|| store.load_chunk(chunk)).flatten().map(|voxels| voxels.voxel_type_id())
			})))?;
			if lod <= 0.0 && input == voxel_type {
				let mut out: Option<Voxels> = None;
				for z in 0..size.z { for y in 0..size.y { for x in 0..size.x {
					let chunk = min + IVec3::new(x, y, z);
					if self.inner.lent.contains(grid, chunk) { continue; }
					let Some(voxels) = store.load_chunk(chunk) else { continue };
					out.get_or_insert_with(|| Voxels::new_with_type(voxels.voxel_type_info()))
						.merge_from(&voxels, IVec3::new(x, y, z) * tile_data::CHUNK_SIZE);
				}}}
				return out.filter(|voxels| !voxels.is_empty());
			}
			let generator = handle.voxel_lod_generator(input, voxel_type)?;
			generator.generate(min, size, lod, &|chunk| {
				(!self.inner.lent.contains(grid, chunk)).then(|| store.load_chunk(chunk)).flatten()
			})
		});
		if cancellation.is_cancelled() { return false; }
		if let Some(handle) = self.inner.handle.get() { handle.voxels_loaded(grid, min, size, lod, voxel_type, edit_index, voxels); }
		true
	}

	fn request_available_area(&self, grid: GridId) {
		let Some(handle) = self.inner.handle.get() else { return };
		if let Some(area) = self.grid_available_area(grid) {
			handle.presence(grid, area.min(), area.size().as_ivec3());
		}
		handle.presence_loaded(grid);
	}

	fn lend(&self, grid: GridId, chunk: IVec3, _cancellation: CancellationToken) -> LendResult {
		if !self.inner.lent.begin(grid, chunk) { return LendResult::Unavailable; }
		let voxels = self.inner.grids.read().unwrap().get(&grid).and_then(|store| store.load_chunk(chunk));
		if voxels.is_none() {
			self.inner.lent.end(grid, chunk);
			return LendResult::Unavailable;
		}
		LendResult::Borrowed(voxels)
	}


	fn return_area(&self, grid: GridId, min: IVec3, size: IVec3) { self.inner.lent.end_area(grid, min, size); }

	fn save(&self, grid: GridId, chunk: IVec3, _edit_index: u64, voxels: &Voxels) -> bool {
		self.inner.grids.write().unwrap().entry(grid).or_default().save_chunk(chunk, voxels);
		self.inner.lent.end(grid, chunk);
		true
	}

	fn forget(&self, grid: GridId, chunk: IVec3) {
		if let Some(store) = self.inner.grids.write().unwrap().get_mut(&grid) { store.forget_chunk(chunk); }
		self.inner.lent.end(grid, chunk);
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
