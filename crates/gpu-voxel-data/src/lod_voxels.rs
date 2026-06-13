use bevy::prelude::Component;

use voxel_data::voxels::Voxels;

#[derive(Component)]
pub struct LodVoxels {
	pub voxels: Voxels,
	pub lod: f32,
	pub priority: f32,
}
