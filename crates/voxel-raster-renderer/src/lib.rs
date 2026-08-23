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
use bevy::prelude::{Component, Resource};
use bevy::ecs::schedule::IntoScheduleConfigs;
use bevy::core_pipeline::core_3d::Opaque3d;
use bevy::render::{
	ExtractSchedule, Render, RenderApp, RenderSystems,
	render_phase::AddRenderCommand,
};

use ::tile_data::{TileCapabilityRegistry, TileData, TileVoxelReducerRegistry};
use voxel_data::{VoxelDataPlugin, voxels::VoxelTypeId};
use voxel_gpu::{
	GpuVoxelDataPlugin, RenderingBuilderAppExt, RenderingType, SlangShader,
	SlangShaderSettings, VoxelGpuDataReaders,
};

#[derive(Component)]
pub struct VoxelRasterShader;

#[derive(Resource, Clone, Copy, Debug, PartialEq, Eq)]
pub struct RasterRenderingType(pub RenderingType);

#[derive(Resource, Default)]
pub struct RasterTileCapabilityRegistry(TileCapabilityRegistry<tile_data::RasterTileCapabilityData>);

impl RasterTileCapabilityRegistry {
	pub fn read(&self, data: &dyn TileData) -> Option<tile_data::RasterTileCapabilityData> { self.0.read(data) }

	pub fn register<T: tile_data::RasterTileCapability>(&mut self) {
		self.0.register::<T>(|data| data.raster_tile_data());
	}
}

pub trait VoxelRasterTileAppExt {
	fn register_raster_tile_data<T: tile_data::RasterTileCapability>(&mut self) -> &mut Self;
	fn register_voxel_raster_generator(
		&mut self,
		voxel_type: VoxelTypeId,
		lod_levels: u8,
	) -> RenderingType;
}

impl VoxelRasterTileAppExt for App {
	fn register_raster_tile_data<T: tile_data::RasterTileCapability>(&mut self) -> &mut Self {
		self.init_resource::<RasterTileCapabilityRegistry>();
		self.world_mut().resource_mut::<RasterTileCapabilityRegistry>().register::<T>();
		self
	}

	fn register_voxel_raster_generator(
		&mut self,
		voxel_type: VoxelTypeId,
		lod_levels: u8,
	) -> RenderingType {
		self.init_resource::<TileVoxelReducerRegistry>();
		let generator = tile_data::VoxelRasterTileGenerator {
			voxel_type,
			lod_levels,
			gpu: self.world().resource::<gpu_data::RasterWorldGpuData>().clone(),
			readers: self.world().resource::<VoxelGpuDataReaders>().clone(),
			reducers: self.world().resource::<TileVoxelReducerRegistry>().clone(),
		};
		let rendering_type = self.register_rendering_builder(generator);
		self.insert_resource(RasterRenderingType(rendering_type));
		rendering_type
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
		let settings = shader_sources::asset_settings();
		let shader = app.world().resource::<bevy::asset::AssetServer>()
			.load_builder()
			.with_settings(move |current: &mut SlangShaderSettings| *current = settings.clone())
			.load(shader_sources::ROOT_SHADER_ASSET);
		app.init_resource::<gpu_data::RasterWorldGpuData>()
			.init_resource::<RasterTileCapabilityRegistry>();
		app.register_raster_tile_data::<tile_data::RasterTileData>();
		app.add_systems(bevy::prelude::Update, gpu_data::collect_raster_gpu_garbage);

		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return; };
		render_app.world_mut().spawn((VoxelRasterShader, SlangShader::new(shader)));
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
		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return; };
		render_app.init_resource::<voxel_raster_renderer_resource::VoxelRasterRendererResource>();
	}
}
