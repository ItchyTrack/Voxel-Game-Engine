use std::collections::HashMap;
use std::sync::RwLock;

use bevy::math::IVec3;
use rustc_hash::FxHashSet;
use voxel_data::grid::GridId;


#[derive(Default)]
pub struct ForgottenChunks {
	grids: RwLock<HashMap<GridId, FxHashSet<IVec3>>>,
}

impl ForgottenChunks {
	pub fn forget(&self, grid: GridId, chunk: IVec3) {
		self.grids.write().unwrap().entry(grid).or_default().insert(chunk);
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
