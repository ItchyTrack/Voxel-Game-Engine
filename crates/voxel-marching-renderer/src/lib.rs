pub mod gpu_data;
pub mod marching_cubes;
pub mod tile_data;
pub mod voxel_camera;

mod extract;
mod model;
mod render_node;
mod renderer_resource;
mod shader_sources;

use basic_voxel::MarchingVoxel;
use bevy::app::{App, Plugin};
use bevy::core_pipeline::{Core3d, Core3dSystems, core_3d::main_opaque_pass_3d};
use bevy::prelude::*;
use bevy::render::{ExtractSchedule, Render, RenderApp, RenderSystems};
use ::tile_data::{TileAppExt, TileCapabilityRegistry, TileData};
use voxel_data::voxels::VoxelType;
use voxel_gpu::{
	RenderingGeneratorAppExt, RenderingGeneratorRegistry, RenderingTileClass,
	RenderingTileGenerator, RenderingType, SlangShader, SlangShaderSettings,
};

#[derive(Component)]
pub struct MarchingShader;

#[derive(Resource, Default)]
pub struct MarchingTileCapabilityRegistry(TileCapabilityRegistry<tile_data::MarchingTileCapabilityData>);

impl MarchingTileCapabilityRegistry {
	pub fn read(&self, data: &dyn TileData) -> Option<tile_data::MarchingTileCapabilityData> { self.0.read(data) }
	pub fn register<T: tile_data::MarchingTileCapability>(&mut self) {
		self.0.register::<T>(|data| data.marching_tile_data());
	}
}

#[derive(Default)]
pub struct VoxelMarchingRendererPlugin;

impl Plugin for VoxelMarchingRendererPlugin {
	fn build(&self, app: &mut App) {
		bevy::asset::embedded_asset!(app, "shaders/marching_cubes.slang");
		let settings = shader_sources::asset_settings();
		let shader = app.world().resource::<AssetServer>()
			.load_builder()
			.with_settings(move |current: &mut SlangShaderSettings| *current = settings.clone())
			.load(shader_sources::ROOT_SHADER_ASSET);
		app.init_resource::<gpu_data::MarchingWorldGpuData>()
			.init_resource::<MarchingTileCapabilityRegistry>();
		app.world_mut().resource_mut::<MarchingTileCapabilityRegistry>().register::<tile_data::MarchingTileData>();

		let gpu = app.world().resource::<gpu_data::MarchingWorldGpuData>().clone();
		app.register_rendering_generator(
			RenderingType::Raster,
			MarchingVoxel::TYPE_INFO.id,
			tile_data::MarchingTileGenerator { gpu },
		);
		let Some(rendering_class) = app.world().get_resource::<RenderingTileClass>().copied() else {
			panic!("VoxelMarchingRendererPlugin requires RenderingGenerationPlugin");
		};
		let generators = app.world().resource::<RenderingGeneratorRegistry>().clone();
		app.register_tile_generator(
			rendering_class.0,
			MarchingVoxel::TYPE_INFO.id,
			RenderingTileGenerator::new(MarchingVoxel::TYPE_INFO.id, generators),
		);

		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return };
		render_app.world_mut().spawn((MarchingShader, SlangShader::new(shader)));
		render_app
			.add_systems(ExtractSchedule, extract::extract_marching_scene)
			.add_systems(Render, render_node::prepare_marching_view_bind_groups.in_set(RenderSystems::PrepareBindGroups))
			.add_systems(Core3d, render_node::marching_render_pass.in_set(Core3dSystems::MainPass).before(main_opaque_pass_3d));
	}

	fn finish(&self, app: &mut App) {
		let gpu = app.world().resource::<gpu_data::MarchingWorldGpuData>().clone();
		gpu.initialize(app.world().resource());
		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return };
		render_app.init_resource::<renderer_resource::MarchingRendererResource>();
	}
}
