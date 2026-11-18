use std::sync::Arc;

use bile_math::NonZeroRegion;
use voxel_data::voxels::Voxels;

#[derive(Debug)]
pub struct VoxelRegionResult {
	pub area: NonZeroRegion,
	pub lod: u8,
	pub voxels: Voxels,
}

/// A shared [`VoxelRegionResult`], as voxel sources stream them.
pub type VoxelRegion = Arc<VoxelRegionResult>;
