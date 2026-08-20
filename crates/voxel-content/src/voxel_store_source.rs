use std::collections::HashMap;
use std::sync::{Arc, OnceLock, RwLock};

use bevy::prelude::*;

use tile_data::NonZeroChunkRegion;
use voxel_data::grid::GridId;
use voxel_data::voxels::{VoxelTypeId, Voxels};
use voxel_sources::{ChunkSource, RequestId, SourceCoverage, SourceHandle, VoxelSourcesAppExt};
use voxel_tasks::{AsyncPriorityTaskPool, CancellationToken};

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

	fn grid_available_area(&self, grid: GridId) -> Option<tile_data::NonZeroChunkRegion> {
		self.inner.grids.read().unwrap().get(&grid)?.available_area()
	}
}

impl ChunkSource for VoxelStoreSource {
	fn init(&self, handle: SourceHandle) {
		let _ = self.inner.handle.set(handle);
	}

	fn request_voxels(
		&self,
		request_id: RequestId,
		cancellation: &CancellationToken,
		grid: GridId,
		region: NonZeroChunkRegion,
		_lod: u8,
		_voxel_type: Option<VoxelTypeId>,
	) -> SourceCoverage {
		if cancellation.is_cancelled() {
			return SourceCoverage::None;
		}
		let owned = {
			let grids = self.inner.grids.read().unwrap();
			let Some(store) = grids.get(&grid) else { return SourceCoverage::None };
			let mut owned = 0;
			for z in region.min().z..region.end().z {
				for y in region.min().y..region.end().y {
					for x in region.min().x..region.end().x {
						owned += store.contains_chunk(IVec3::new(x, y, z)) as u32;
					}
				}
			}
			owned
		};
		let coverage = match owned {
			0 => SourceCoverage::None,
			owned if owned == region.area() => SourceCoverage::All,
			_ => SourceCoverage::Some,
		};
		if coverage == SourceCoverage::None {
			return coverage;
		}

		let source = self.clone();
		let handle = self.inner.handle.get().expect("voxel store source was not initialized").clone();
		let cancellation = cancellation.clone();
		AsyncPriorityTaskPool::get().spawn(1.0, async move {
			let _span = bevy::log::info_span!("VoxelStoreSource build").entered();
			let grids = source.inner.grids.read().unwrap();
			if let Some(store) = grids.get(&grid) {
				'chunks: for z in region.min().z..region.end().z {
					for y in region.min().y..region.end().y {
						for x in region.min().x..region.end().x {
							if cancellation.is_cancelled() {
								break 'chunks;
							}
							let chunk = IVec3::new(x, y, z);
							if let Some(voxels) = store.load_chunk(chunk) {
								handle.voxels(
									request_id,
									grid,
									NonZeroChunkRegion::from_single(chunk),
									0,
									store.chunk_generation(chunk),
									voxels,
								);
							}
						}
					}
				}
			}
			handle.voxels_loaded(request_id);
		});

		coverage
	}

	fn request_presence(
		&self,
		request_id: RequestId,
		_cancellation: CancellationToken,
		grid: GridId,
	) {
		let handle = self.inner.handle.get().expect("voxel store source was not initialized");
		if let Some(region) = self.grid_available_area(grid) {
			handle.presence(request_id, grid, region);
		}
		handle.presence_loaded(request_id);
	}

	fn take_ownership(
		&self,
		grid: GridId,
		region: NonZeroChunkRegion
	) {
		if let Some(store) = self.inner.grids.write().unwrap().get_mut(&grid) {
			for x in region.min().x..region.end().x {
				for y in region.min().y..region.end().y {
					for z in region.min().z..region.end().z {
						store.forget_chunk(IVec3::new(x, y, z));
					}
				}
			}
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
