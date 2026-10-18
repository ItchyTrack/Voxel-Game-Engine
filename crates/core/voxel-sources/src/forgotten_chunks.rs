use std::sync::RwLock;

use bevy::math::IVec3;
use rustc_hash::FxHashMap;
use bile_math::NonZeroRegion;
use tile_data::{chunk_origin, chunk_region};
use voxel_trees::{
	grid_tree::U16Cell,
	signed_grid_tree::SignedGridTree,
};
use voxel_data::grid::GridId;

// Just a tmp helper for voxel source impls to use. Will remove proabaly

#[derive(Default)]
pub struct ForgottenChunks {
	grids: RwLock<FxHashMap<GridId, SignedGridTree<U16Cell>>>,
}

impl ForgottenChunks {
	pub fn forget(&self, grid: GridId, chunk: IVec3) {
		self.grids.write().unwrap().entry(grid).or_default().add_area(chunk_region(chunk), 0u16);
	}

	pub fn remember(&self, grid: GridId, chunk: IVec3) {
		if let Some(chunks) = self.grids.write().unwrap().get_mut(&grid) {
			chunks.remove_area(chunk_region(chunk));
		}
	}

	pub fn remember_area(&self, grid: GridId, region: NonZeroRegion) {
		if let Some(forgotten) = self.grids.write().unwrap().get_mut(&grid) {
			forgotten.remove_area(region);
		}
	}

	pub fn forget_area(&self, grid: GridId, region: NonZeroRegion) {
		self.grids.write().unwrap().entry(grid).or_default().add_area(region, 0u16);
	}

	pub fn contains(&self, grid: GridId, chunk: IVec3) -> bool {
		self.grids.read().unwrap().get(&grid).is_some_and(|chunks| chunks.get(chunk_origin(chunk)).is_some())
	}

	pub fn any_remembered_in(&self, grid: GridId, region: NonZeroRegion) -> bool {
		let grids = self.grids.read().unwrap();
		let Some(chunks) = grids.get(&grid) else { return true };
		!chunks.is_area_filled(region)
	}
}
