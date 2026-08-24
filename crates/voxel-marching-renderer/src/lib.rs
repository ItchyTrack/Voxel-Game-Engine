pub mod gpu_data;
pub mod marching_cubes;
pub mod tile_data;
pub mod voxel_camera;

mod extract;
mod mc33_table;
mod model;
mod render_node;
mod renderer_resource;
mod shader_sources;

use bevy::app::{App, Plugin};
use bevy::core_pipeline::core_3d::Opaque3d;
use bevy::prelude::*;
use bevy::render::{
	ExtractSchedule, Render, RenderApp, RenderSystems,
	render_phase::AddRenderCommand,
};
use ::tile_data::{TileCapabilityRegistry, TileData, TileVoxelReducerRegistry};
use voxel_gpu::{RenderingBuilderAppExt, RenderingType, SlangShader, SlangShaderSettings};

#[derive(Component)]
pub struct MarchingShader;

#[derive(Resource, Clone, Copy, Debug, PartialEq, Eq)]
pub struct MarchingRenderingType(pub RenderingType);

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
			.init_resource::<MarchingTileCapabilityRegistry>()
			.init_resource::<TileVoxelReducerRegistry>()
			.add_systems(Update, gpu_data::collect_marching_gpu_garbage);
		app.world_mut().resource_mut::<MarchingTileCapabilityRegistry>().register::<tile_data::MarchingTileData>();

		let gpu = app.world().resource::<gpu_data::MarchingWorldGpuData>().clone();
		let reducers = app.world().resource::<TileVoxelReducerRegistry>().clone();
		let rendering_type = app.register_rendering_builder(tile_data::MarchingTileBuilder { gpu, reducers });
		app.insert_resource(MarchingRenderingType(rendering_type));

		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return };
		render_app.world_mut().spawn((MarchingShader, SlangShader::new(shader)));
		render_app
			.add_render_command::<Opaque3d, render_node::DrawMarchingCommands>()
			.add_systems(ExtractSchedule, extract::extract_marching_scene)
			.add_systems(
				Render,
				(
					render_node::queue_marching_phase.in_set(RenderSystems::Queue),
					render_node::prepare_marching_view_bind_groups.in_set(RenderSystems::PrepareBindGroups),
				),
			);
	}

	fn finish(&self, app: &mut App) {
		let gpu = app.world().resource::<gpu_data::MarchingWorldGpuData>().clone();
		gpu.initialize(app.world().resource(), app.world().resource());
		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return };
		render_app.init_resource::<renderer_resource::MarchingRendererResource>();
	}
}
