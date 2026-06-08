use bevy::prelude::*;

use voxel_data::grid::GridId;
use voxel_data::voxels::Voxels;

#[derive(Component)]
pub struct LodVoxels {
	pub voxels: Voxels,
	pub grid: GridId,
	pub min: IVec3,
	pub size: IVec3,
	pub lod: f32,
	pub priority: f32,
}
