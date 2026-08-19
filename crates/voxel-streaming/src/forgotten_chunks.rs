use std::sync::RwLock;

use bevy::math::IVec3;
use rustc_hash::{FxHashMap, FxHashSet};
use voxel_data::grid::GridId;
use tile_data::ChunkRegion;


#[derive(Default)]
pub struct ForgottenChunks {
	grids: RwLock<FxHashMap<GridId, FxHashSet<IVec3>>>,
}

impl ForgottenChunks {
	pub fn forget(&self, grid: GridId, chunk: IVec3) {
		self.grids.write().unwrap().entry(grid).or_default().insert(chunk);
	}

	pub fn forget_area_where(
		&self,
		grid: GridId,
		region: ChunkRegion,
		mut owns: impl FnMut(IVec3) -> bool,
	) -> Vec<IVec3> {
		let mut grids = self.grids.write().unwrap();
		let forgotten = grids.entry(grid).or_default();
		let mut taken = Vec::new();
		for z in region.min().z..region.end().z {
			for y in region.min().y..region.end().y {
				for x in region.min().x..region.end().x {
					let chunk = IVec3::new(x, y, z);
					if !forgotten.contains(&chunk) && owns(chunk) {
						forgotten.insert(chunk);
						taken.push(chunk);
					}
				}
			}
		}
		taken
	}

	pub fn contains(&self, grid: GridId, chunk: IVec3) -> bool {
		self.grids.read().unwrap().get(&grid).is_some_and(|chunks| chunks.contains(&chunk))
	}

	pub fn any_remembered_in(&self, grid: GridId, min: IVec3, size: IVec3) -> bool {
		let grids = self.grids.read().unwrap();
		let Some(chunks) = grids.get(&grid) else { return true };
		(0..size.z).any(|z| {
			(0..size.y).any(|y| (0..size.x).any(|x| !chunks.contains(&(min + IVec3::new(x, y, z)))))
		})
	}
}
