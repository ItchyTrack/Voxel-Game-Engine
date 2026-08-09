mod generator;
mod voxel;

pub use generator::{downsample_region, BasicVoxelLodGenerator, LodVoxelLodGenerator};
pub use voxel::{BasicVoxel, LodVoxel};

use bevy::prelude::*;
use voxel_physics::VoxelPhysicsAppExt;
use voxel_sources::VoxelSourcesAppExt;
use voxel_gpu::VoxelGpuAppExt;

pub struct BasicVoxelPlugin;

impl Plugin for BasicVoxelPlugin {
	fn build(&self, app: &mut App) {
		bevy::asset::embedded_asset!(app, "shaders/basic_voxel.slang");
		bevy::asset::embedded_asset!(app, "shaders/lod_voxel.slang");
		app
			.register_voxel_lod_generator(BasicVoxelLodGenerator)
			.register_voxel_lod_generator(LodVoxelLodGenerator)
			.register_voxel_mass::<BasicVoxel>()
			.register_voxel_gpu_data::<BasicVoxel>()
			.register_voxel_gpu_data::<LodVoxel>();
	}
}
