use bevy::prelude::*;

use voxel_data::voxels::Voxels;

use crate::handle::SourceHandle;

#[derive(Component, Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct GridKey(pub u64);

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct SourceId(pub usize);

pub trait ChunkSource: Send + Sync {
	fn init(&mut self, handle: SourceHandle) {
		let _ = handle;
	}

	/// Cost of serving `chunk`, or `None` if this source can't. Lowest wins.
	fn cost(&self, grid: GridKey, chunk: IVec3) -> Option<u32>;

	fn request_load(&mut self, grid: GridKey, chunk: IVec3);

	fn can_save(&self) -> bool {
		false
	}

	fn save(&mut self, grid: GridKey, chunk: IVec3, voxels: &Voxels) {
		let _ = (grid, chunk, voxels);
	}

	fn forget(&mut self, grid: GridKey, chunk: IVec3) {
		let _ = (grid, chunk);
	}
}
