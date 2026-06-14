use bevy::prelude::*;

use voxel_data::voxels::Voxels;

use crate::handle::SourceHandle;

#[derive(Component, Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct GridKey(pub u64);

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct SourceId(pub usize);

pub trait VoxelLodGenerator: Send + Sync {
	fn generate(&self, voxels: &Voxels, lod: f32) -> Option<Voxels>;
}

/// A source of voxel chunks. Methods take `&self` and are called concurrently from the
/// async load workers, so mutable state must use interior mutability. This lets many
/// loads from one source run in parallel instead of serializing on an outer lock.
pub trait ChunkSource: Send + Sync {
	fn init(&self, handle: SourceHandle) {
		let _ = handle;
	}

	/// Cost of serving `chunk`, or `None` if this source can't. Lowest wins.
	fn cost(&self, grid: GridKey, chunk: IVec3) -> Option<u32>;

	fn request_load(&self, grid: GridKey, chunk: IVec3);

	fn cost_lod(&self, grid: GridKey, min: IVec3, size: IVec3, lod: f32) -> Option<u32>;

	fn has_any_chunks_in_area(&self, grid: GridKey, min: IVec3, size: IVec3) -> bool {
		(0..size.z).any(|z| (0..size.y).any(|y| (0..size.x).any(|x| self.cost(grid, min + IVec3::new(x, y, z)).is_some())))
	}

	fn request_load_lod(&self, grid: GridKey, min: IVec3, size: IVec3, lod: f32);

	fn can_save(&self) -> bool {
		false
	}

	fn save(&self, grid: GridKey, chunk: IVec3, voxels: &Voxels) {
		let _ = (grid, chunk, voxels);
	}

	fn forget(&self, grid: GridKey, chunk: IVec3) {
		let _ = (grid, chunk);
	}
}
