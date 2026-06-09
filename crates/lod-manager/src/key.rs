use bevy::prelude::*;
use voxel_data::grid::GridId;

/// Unique identity for one streamed LOD chunk.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct LodKey {
	pub grid: GridId,
	pub min: IVec3,
	pub size: IVec3,
	pub level: u32,
}

impl LodKey {
	pub fn new(grid: GridId, min: IVec3, size: IVec3, lod: f32) -> Self {
		Self { grid, min, size, level: lod.max(0.0).floor() as u32 }
	}

	pub fn from_level(grid: GridId, min: IVec3, size: IVec3, level: u32) -> Self {
		Self { grid, min, size, level }
	}

	pub fn lod(&self) -> f32 {
		self.level as f32
	}
}

/// Where the manager should put a completed request.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
pub enum LodDestination {
	/// Keep the request in the manager cache only. This is useful for warm-up or CPU-side users.
	CacheOnly,
	/// Spawn a [`gpu_voxel_data::LodVoxels`] entity so upload/residency systems can move it to the GPU.
	#[default]
	Gpu,
}

impl LodDestination {
	pub fn wants_gpu(self) -> bool {
		matches!(self, Self::Gpu)
	}
}
