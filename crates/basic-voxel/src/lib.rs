mod generator;
mod voxel;

pub use generator::{downsample_region, BasicVoxelLodGenerator, LodVoxelLodGenerator, MarchingVoxelLodGenerator};
pub use voxel::{BasicVoxel, LodVoxel, MarchingVoxel};

use bevy::prelude::*;
use tile_data::TileAppExt;
use voxel_data::voxels::VoxelType;
use voxel_gpu::{
	RenderingGeneratorRegistry, RenderingTileClass, RenderingTileGenerator, VoxelGpuAppExt,
};
use voxel_physics::VoxelPhysicsAppExt;
use voxel_raster_renderer::{VoxelRasterRendererPlugin, VoxelRasterTileAppExt};
use voxel_ray_renderer::{VoxelRayRendererPlugin, VoxelRayTileAppExt};
use voxel_sources::VoxelSourcesAppExt;

pub struct BasicVoxelPlugin;

impl Plugin for BasicVoxelPlugin {
	fn build(&self, app: &mut App) {
		bevy::asset::embedded_asset!(app, "shaders/basic_voxel.slang");
		bevy::asset::embedded_asset!(app, "shaders/lod_voxel.slang");
		app
			.register_voxel_lod_generator(BasicVoxelLodGenerator)
			.register_voxel_lod_generator(LodVoxelLodGenerator)
			.register_voxel_lod_generator(MarchingVoxelLodGenerator)
			.register_voxel_mass::<BasicVoxel>()
			.register_voxel_mass::<MarchingVoxel>()
			.register_voxel_gpu_data::<BasicVoxel>()
			.register_voxel_gpu_data::<LodVoxel>();

		let Some(rendering_class) = app.world().get_resource::<RenderingTileClass>().copied() else { return };
		let source_voxel_type = BasicVoxel::TYPE_INFO.id;
		let voxel_type = LodVoxel::TYPE_INFO.id;
		if app.is_plugin_added::<VoxelRayRendererPlugin>() {
			app.register_voxel_ray_generator(source_voxel_type, voxel_type, 0);
		}
		if app.is_plugin_added::<VoxelRasterRendererPlugin>() {
			app.register_voxel_raster_generator(source_voxel_type, voxel_type, 0);
		}
		let generators = app.world().resource::<RenderingGeneratorRegistry>().clone();
		app.register_tile_generator(
			rendering_class.0,
			source_voxel_type,
			RenderingTileGenerator::new(source_voxel_type, generators),
		);
	}
}
