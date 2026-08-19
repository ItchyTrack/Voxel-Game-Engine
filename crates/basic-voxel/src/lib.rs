mod generator;
mod voxel;

pub use generator::downsample_region;
use generator::{BasicToLodVoxelReducer, BasicToMarchingVoxelReducer, LodToLodVoxelReducer, MarchingToMarchingVoxelReducer};
pub use voxel::{BasicVoxel, LodVoxel, MarchingVoxel};

use bevy::prelude::*;
use tile_data::TileAppExt;
use voxel_data::voxels::VoxelType;
use voxel_gpu::VoxelGpuAppExt;
use voxel_physics::VoxelPhysicsAppExt;
use voxel_raster_renderer::{VoxelRasterRendererPlugin, VoxelRasterTileAppExt};
use voxel_ray_renderer::{VoxelRayRendererPlugin, VoxelRayTileAppExt};

pub struct BasicVoxelPlugin;

impl Plugin for BasicVoxelPlugin {
	fn build(&self, app: &mut App) {
		bevy::asset::embedded_asset!(app, "shaders/basic_voxel.slang");
		bevy::asset::embedded_asset!(app, "shaders/lod_voxel.slang");
		app
			.register_tile_voxel_reducer(BasicToLodVoxelReducer)
			.register_tile_voxel_reducer(LodToLodVoxelReducer)
			.register_tile_voxel_reducer(BasicToMarchingVoxelReducer)
			.register_tile_voxel_reducer(MarchingToMarchingVoxelReducer)
			.register_voxel_mass::<BasicVoxel>()
			.register_voxel_mass::<MarchingVoxel>()
			.register_voxel_gpu_data::<BasicVoxel>()
			.register_voxel_gpu_data::<LodVoxel>();

		let voxel_type = LodVoxel::TYPE_INFO.id;
		if app.is_plugin_added::<VoxelRayRendererPlugin>() {
			app.register_voxel_ray_generator(voxel_type, 0);
		}
		if app.is_plugin_added::<VoxelRasterRendererPlugin>() {
			app.register_voxel_raster_generator(voxel_type, 0);
		}
	}
}
