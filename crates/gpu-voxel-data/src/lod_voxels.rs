use bevy::prelude::*;

use voxel_data::voxels::Voxels;

#[derive(Component)]
pub struct LodVoxels {
	pub voxels: Voxels,
	pub world_transform: Transform,
	pub lod: f32,
	pub priority: f32,
}
