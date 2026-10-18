pub mod render_node;
pub mod gpu_data;
pub mod gpu_raster_mesh;
pub mod voxel_camera;
pub mod tile_data;

mod extract;
mod model;
mod shader_sources;
mod voxel_raster_renderer;
mod voxel_raster_renderer_resource;

use bevy::app::{App, Plugin};
use bevy::prelude::Component;
use bevy::ecs::schedule::IntoScheduleConfigs;
use bevy::core_pipeline::core_3d::Opaque3d;
use bevy::render::{
	ExtractSchedule, Render, RenderApp, RenderSystems,
	render_phase::AddRenderCommand,
};

use ::tile_data::TileVoxelReducerRegistry;
use voxel_data::{VoxelDataPlugin, voxels::VoxelTypeId};
use voxel_streaming::TileBuilderAppExt;
use voxel_gpu::{
	GpuVoxelDataPlugin, SlangShader,
	VoxelGpuDataReaders, VoxelGpuShaderTypes,
};

#[derive(Component)]
pub struct VoxelRasterShader;

pub trait VoxelRasterTileAppExt {
	fn register_voxel_raster_builder(&mut self, voxel_type: VoxelTypeId, lod_levels: u8) -> &mut Self;
}

impl VoxelRasterTileAppExt for App {
	fn register_voxel_raster_builder(&mut self, voxel_type: VoxelTypeId, lod_levels: u8) -> &mut Self {
		self.init_resource::<TileVoxelReducerRegistry>();
		let builder = tile_data::VoxelRasterTileBuilder {
			voxel_type,
			lod_levels,
			gpu: self.world().resource::<gpu_data::RasterWorldGpuData>().clone(),
			readers: self.world().resource::<VoxelGpuDataReaders>().clone(),
			reducers: self.world().resource::<TileVoxelReducerRegistry>().clone(),
		};
		self.register_tile_builder(builder)
	}
}

#[derive(Default)]
pub struct VoxelRasterRendererPlugin;

impl Plugin for VoxelRasterRendererPlugin {
	fn build(&self, app: &mut App) {
		bevy::asset::embedded_asset!(app, "shaders/raster_voxel.slang");
		if !app.is_plugin_added::<VoxelDataPlugin>() {
			app.add_plugins(VoxelDataPlugin);
		}
		if !app.is_plugin_added::<GpuVoxelDataPlugin>() {
			app.add_plugins(GpuVoxelDataPlugin);
		}
		app.init_resource::<gpu_data::RasterWorldGpuData>()
			.register_tile_class_component::<tile_data::RasterTileClass, tile_data::RasterTileData>();
		app.add_systems(bevy::prelude::Update, (
			gpu_data::collect_raster_gpu_garbage,
			shader_sources::reload_shader_on_type_change,
		));

		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return; };
		render_app
			.add_render_command::<Opaque3d, render_node::DrawVoxelRasterCommands>()
			.add_systems(ExtractSchedule, extract::extract_raster_scene)
			.add_systems(
				Render,
				(
					render_node::queue_raster_phase.in_set(RenderSystems::Queue),
					render_node::prepare_raster_view_bind_groups.in_set(RenderSystems::PrepareBindGroups),
				),
			);
	}

	fn finish(&self, app: &mut App) {
		let gpu = app.world().resource::<gpu_data::RasterWorldGpuData>().clone();
		gpu.initialize(app.world().resource(), app.world().resource());
		let shader = shader_sources::load_shader(
			app.world().resource(),
			app.world().resource::<VoxelGpuShaderTypes>(),
		);
		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return; };
		render_app.world_mut().spawn((VoxelRasterShader, SlangShader::new(shader)));
		render_app.init_resource::<voxel_raster_renderer_resource::VoxelRasterRendererResource>();
	}
}
