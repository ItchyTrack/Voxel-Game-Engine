use std::sync::RwLock;

use bevy::math::IVec3;
use rustc_hash::FxHashMap;
use tile_data::NonZeroChunkRegion;
use voxel_data::{
	grid::GridId,
	grid_tree::U16Cell,
	region::NonZeroVoxelRegion,
	signed_grid_tree::SignedGridTree,
};

// Just a tmp helper for voxel source impls to use. Will remove proabaly

type ForgottenChunkTree = SignedGridTree<U16Cell>;

#[derive(Default)]
pub struct ForgottenChunks {
	grids: RwLock<FxHashMap<GridId, ForgottenChunkTree>>,
}

impl ForgottenChunks {
	pub fn forget(&self, grid: GridId, chunk: IVec3) {
		self.grids.write().unwrap().entry(grid).or_default().insert(chunk, 0u16);
	}

	pub fn remember(&self, grid: GridId, chunk: IVec3) {
		if let Some(chunks) = self.grids.write().unwrap().get_mut(&grid) {
			chunks.remove(chunk);
		}
	}

	pub fn remember_area(&self, grid: GridId, region: NonZeroChunkRegion) {
		if let Some(forgotten) = self.grids.write().unwrap().get_mut(&grid) {
			forgotten.remove_area(chunk_region(region));
		}
	}

	pub fn forget_area(&self, grid: GridId, region: NonZeroChunkRegion) {
		self.grids.write().unwrap().entry(grid).or_default().add_area(chunk_region(region), 0u16);
	}

	pub fn contains(&self, grid: GridId, chunk: IVec3) -> bool {
		self.grids.read().unwrap().get(&grid).is_some_and(|chunks| chunks.get(chunk).is_some())
	}

	pub fn any_remembered_in(&self, grid: GridId, region: NonZeroChunkRegion) -> bool {
		let grids = self.grids.read().unwrap();
		let Some(chunks) = grids.get(&grid) else { return true };
		!chunks.is_area_filled(chunk_region(region))
	}
}

fn chunk_region(region: NonZeroChunkRegion) -> NonZeroVoxelRegion {
	NonZeroVoxelRegion::new(region.min(), region.size()).unwrap()
}
