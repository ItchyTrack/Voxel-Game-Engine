use std::collections::HashMap;
use std::sync::{Arc, OnceLock, RwLock};

use bevy::prelude::*;

use tile_data::NonZeroChunkRegion;
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
		lod: u8,
		_voxel_type: Option<VoxelTypeId>,
	) -> Option<(SourceCoverage, Option<impl AsyncFnOnce() + Send + 'static>)> {
		let grids = self.inner.grids.read().unwrap();
		let Some(store) = grids.get(&grid) else { return SourceCoverage::None };
		let mut has_one = None;
		let mut source_coverage = SourceCoverage::All;
		'outer: for z in region.min().z..region.end().z {
			for y in region.min().y..region.end().y {
				for x in region.min().x..region.end().x {
					if store.contains_chunk(IVec3::new(x, y, z)) {
						if let Some(has_one_val) = has_one {
							if has_one_val {
								source_coverage = SourceCoverage::Some;
								break 'outer;
							}
						} else {
							has_one = Some(true)
						}
					} else {
						if let Some(has_one_val) = has_one {
							if has_one_val {
								source_coverage = SourceCoverage::Some;
								break 'outer;
							}
						} else {
							has_one = Some(false)
						}
					}
				}
			}
		}
		assert!(has_one.is_some()); // has to happen as region is non zero
		if !has_one.unwrap() {
			return None;
		}

		let async_fn = async move || {
			if cancellation.is_cancelled() { return; }
			let grids = self.inner.grids.read().unwrap();
			let Some(store) = grids.get(&grid) else { return; };
			let input = store.voxel_type_id().expect("owned voxel-store chunks have no voxel type");
			let handle = self.inner.handle.get().unwrap();
			for x in region.min().x..region.end().x {
				for y in region.min().y..region.end().y {
					for z in region.min().z..region.end().z {
						let chunk = IVec3::new(x, y, z);
						if let Some(voxels) = store.load_chunk(chunk) {
							if cancellation.is_cancelled() { return; }
							handle.voxels(request_id, grid, NonZeroChunkRegion::from_single(chunk), lod, generation, voxels);
						}
					}
				}
			}
		};

		Some((source_coverage, Some(async_fn)))
	}

	fn request_presence(
		&self,
		request_id: RequestId,
		cancellation: CancellationToken,
		grid: GridId,
	) -> Option<impl AsyncFnOnce() + Send + 'static> {
		let Some(handle) = self.inner.handle.get() else { return None };
		if let Some(region) = self.grid_available_area(grid).and_then(|area| area.try_into().ok()) {
			handle.presence(request_id, grid, region);
		}
		handle.presence_loaded(region);
		None
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
