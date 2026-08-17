use crate::{grid::GridId, region::NonZeroVoxelRegion};

pub struct VoxelsLocation { // TODO: rename this!
	pub grid: GridId,
	pub region: NonZeroVoxelRegion,
	pub scale: u8, // voxel_size = 2^scale
}