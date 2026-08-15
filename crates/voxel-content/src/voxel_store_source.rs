use std::collections::HashMap;
use std::sync::{Arc, OnceLock, RwLock};

use bevy::prelude::*;

use voxel_data::grid::GridId;
use voxel_data::voxels::{VoxelTypeId, Voxels};
use voxel_sources::{CancellationToken, ChunkSource, SourceCoverage, SourceHandle, TakeJob, VoxelSourcesAppExt};

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
			store.save_chunk(chunk, 0, &voxels);
		}
	}

	fn grid_available_area(&self, grid: GridId) -> Option<tile_data::ChunkRegion> {
		self.inner.grids.read().unwrap().get(&grid)?.available_area()
	}
}

impl ChunkSource for VoxelStoreSource {
	fn init(&self, handle: SourceHandle) {
		let _ = self.inner.handle.set(handle);
	}

	fn request_load(&self, grid: GridId, chunk: IVec3, _generation: u64, cancellation: CancellationToken) -> SourceCoverage {
		if cancellation.is_cancelled() { return SourceCoverage::None; }
		let Some((actual, voxels)) = self.inner.grids.read().unwrap().get(&grid)
			.filter(|store| store.contains_chunk(chunk))
			.map(|store| (store.chunk_generation(chunk), store.load_chunk(chunk))) else { return SourceCoverage::None };
		if cancellation.is_cancelled() { return SourceCoverage::None; }
		if let Some(handle) = self.inner.handle.get() {
			handle.loaded(grid, chunk, actual, voxels);
		}
		SourceCoverage::All
	}

	fn request_voxel_area(
		&self,
		grid: GridId,
		min: IVec3,
		size: IVec3,
		lod: f32,
		voxel_type: VoxelTypeId,
		generation: u64,
		cancellation: CancellationToken,
	) -> SourceCoverage {
		if cancellation.is_cancelled() { return SourceCoverage::None; }
		let (coverage, input) = {
			let grids = self.inner.grids.read().unwrap();
			let Some(store) = grids.get(&grid) else { return SourceCoverage::None };
			let mut owned = 0;
			for z in 0..size.z { for y in 0..size.y { for x in 0..size.x {
				owned += store.contains_chunk(min + IVec3::new(x, y, z)) as usize;
			}}}
			(SourceCoverage::from_count(owned, (size.x * size.y * size.z) as usize), store.voxel_type_id())
		};
		if coverage == SourceCoverage::None { return coverage; }
		let input = input.expect("owned voxel-store chunks have no voxel type");
		assert!(
			lod <= 0.0 && input == voxel_type
				|| self.inner.handle.get().and_then(|handle| handle.voxel_lod_generator(input, voxel_type)).is_some(),
			"voxel store source does not support requested voxel type or LOD",
		);
		let voxels = self.inner.handle.get().and_then(|handle| {
			let grids = self.inner.grids.read().unwrap();
			let store = grids.get(&grid)?;
			let input = (0..size.z).find_map(|z| (0..size.y).find_map(|y| (0..size.x).find_map(|x| {
				let chunk = min + IVec3::new(x, y, z);
				store.load_chunk(chunk).map(|voxels| voxels.voxel_type_id())
			})))?;
			if lod <= 0.0 && input == voxel_type {
				let mut out: Option<Voxels> = None;
				for z in 0..size.z { for y in 0..size.y { for x in 0..size.x {
					let chunk = min + IVec3::new(x, y, z);
					let Some(voxels) = store.load_chunk(chunk) else { continue };
					out.get_or_insert_with(|| Voxels::new_with_type(voxels.voxel_type_info()))
						.merge_from(&voxels, IVec3::new(x, y, z) * tile_data::CHUNK_SIZE);
				}}}
				return out.filter(|voxels| !voxels.is_empty());
			}
			let generator = handle.voxel_lod_generator(input, voxel_type)?;
			generator.generate(min, size, lod, &|chunk| {
				store.load_chunk(chunk)
			})
		});
		if cancellation.is_cancelled() { return SourceCoverage::None; }
		if let Some(handle) = self.inner.handle.get() {
			let actual = self.inner.grids.read().unwrap().get(&grid).map_or(generation, |store| {
				let mut latest = 0;
				for z in 0..size.z { for y in 0..size.y { for x in 0..size.x {
					latest = latest.max(store.chunk_generation(min + IVec3::new(x, y, z)));
				}}}
				latest
			});
			handle.voxels_loaded(grid, min, size, lod, voxel_type, actual, voxels);
		}
		coverage
	}

	fn request_available_area(&self, grid: GridId) {
		let Some(handle) = self.inner.handle.get() else { return };
		if let Some(area) = self.grid_available_area(grid) {
			handle.presence(grid, area.min(), area.size().as_ivec3());
		}
		handle.presence_loaded(grid);
	}

	fn take(&self, destination: voxel_sources::SourceId, grid: GridId, min: IVec3, size: IVec3, _generation: u64) -> Vec<TakeJob> {
		let taken = {
			let mut grids = self.inner.grids.write().unwrap();
			let Some(store) = grids.get_mut(&grid) else { return Vec::new() };
			let mut taken = Vec::new();
			for z in min.z..min.z + size.z { for y in min.y..min.y + size.y { for x in min.x..min.x + size.x {
				let chunk = IVec3::new(x, y, z);
				if let Some((generation, voxels)) = store.take_chunk(chunk) { taken.push((chunk, generation, voxels)); }
			}}}
			taken
		};
		let handle = self.inner.handle.get().expect("voxel store source was not initialized").clone();
		taken.into_iter().map(|(chunk, generation, compressed)| TakeJob::new(chunk, {
			let handle = handle.clone();
			move || {
				let voxels = compressed.and_then(|voxels| voxels.decompress().ok());
				handle.transferred(destination, grid, chunk, generation, voxels);
			}
		})).collect()
	}

	fn save(&self, grid: GridId, chunk: IVec3, generation: u64, voxels: &Voxels) -> bool {
		let saved = self.inner.grids.write().unwrap().entry(grid).or_default().save_chunk(chunk, generation, voxels);
		if saved {
			if let Some(handle) = self.inner.handle.get() {
				handle.synchronize_region_generation(grid, tile_data::ChunkRegion::new(chunk, UVec3::ONE), generation);
			}
		}
		saved
	}

	fn forget(&self, grid: GridId, chunk: IVec3) {
		if let Some(store) = self.inner.grids.write().unwrap().get_mut(&grid) { store.forget_chunk(chunk); }
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
