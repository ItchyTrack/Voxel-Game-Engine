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
use ::tile_data::TileVoxelReducerRegistry;
use voxel_gpu::{SlangShader, SlangShaderSettings};
use voxel_streaming::TileBuilderAppExt;

#[derive(Component)]
pub struct MarchingShader;

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
			.init_resource::<TileVoxelReducerRegistry>()
			.register_tile_class_component::<tile_data::MarchingTileClass, tile_data::MarchingTileData>()
			.add_systems(Update, gpu_data::collect_marching_gpu_garbage);

		let gpu = app.world().resource::<gpu_data::MarchingWorldGpuData>().clone();
		let reducers = app.world().resource::<TileVoxelReducerRegistry>().clone();
		app.register_tile_builder(tile_data::MarchingTileBuilder { gpu, reducers });

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
