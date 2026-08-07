use bevy::prelude::*;
use voxel_data::{
	grid::GridId,
	voxels::{VoxelTypeId, Voxels},
};

use crate::{TileData, TileKey};

pub struct TileGeneratorInput {
	pub grid: GridId,
	pub key: TileKey,
	pub voxel_lod: u8,
	pub voxels: Voxels,
}

pub trait TileGenerator: Send + Sync + 'static {
	fn voxel_type(&self) -> VoxelTypeId;
	fn lod_levels(&self) -> u8;
	fn generate(&self, input: TileGeneratorInput) -> Option<Box<dyn TileData>>;
}
