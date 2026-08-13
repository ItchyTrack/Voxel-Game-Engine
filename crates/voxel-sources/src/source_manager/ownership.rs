use std::collections::{HashMap, HashSet};
use std::sync::RwLock;

use bevy::math::IVec3;
use voxel_data::grid::GridId;

/// Compact shared helper for sources that retain owned data while another
/// source temporarily serves it.
#[derive(Default)]
pub struct LentChunks {
	chunks: RwLock<HashMap<GridId, HashSet<IVec3>>>,
}

impl LentChunks {
	/// Atomically marks an owned chunk as lent. Returns false if it was already lent.
	pub fn begin(&self, grid: GridId, chunk: IVec3) -> bool {
		self.chunks.write().unwrap().entry(grid).or_default().insert(chunk)
	}

	pub fn end(&self, grid: GridId, chunk: IVec3) {
		self.end_area(grid, chunk, IVec3::ONE);
	}

	pub fn end_area(&self, grid: GridId, min: IVec3, size: IVec3) {
		let mut chunks = self.chunks.write().unwrap();
		if let Some(grid_chunks) = chunks.get_mut(&grid) {
			for z in min.z..min.z + size.z { for y in min.y..min.y + size.y { for x in min.x..min.x + size.x {
				grid_chunks.remove(&IVec3::new(x, y, z));
			}}}
			if grid_chunks.is_empty() { chunks.remove(&grid); }
		}
	}

	pub fn contains(&self, grid: GridId, chunk: IVec3) -> bool {
		self.chunks.read().unwrap().get(&grid).is_some_and(|chunks| chunks.contains(&chunk))
	}

	pub fn any_available_in(&self, grid: GridId, min: IVec3, size: IVec3) -> bool {
		let chunks = self.chunks.read().unwrap();
		let lent = chunks.get(&grid);
		(0..size.z).any(|z| (0..size.y).any(|y| (0..size.x).any(|x| {
			!lent.is_some_and(|lent| lent.contains(&(min + IVec3::new(x, y, z))))
		})))
	}
}
