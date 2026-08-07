pub mod camera;
pub mod render_node;
pub mod gpu_data;
pub mod gpu_raster_mesh;
pub mod voxel_camera;
pub mod tile_data;

mod extract;
mod model;
mod residency;
#[cfg(all(feature = "shader_hot_reload", not(target_arch = "wasm32")))]
mod shader_hot_reload;
mod shader_sources;
mod voxel_raster_renderer;
mod voxel_raster_renderer_resource;

use bevy::app::{App, Plugin};
use bevy::prelude::Resource;
use bevy::core_pipeline::core_3d::main_opaque_pass_3d;
use bevy::core_pipeline::{Core3d, Core3dSystems};
use bevy::ecs::schedule::IntoScheduleConfigs;
use bevy::render::{ExtractSchedule, Render, RenderApp, RenderSystems};

use ::tile_data::{TileCapabilityRegistry, TileData};
use voxel_data::VoxelDataPlugin;
use voxel_gpu::GpuVoxelDataPlugin;

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
}

impl VoxelRasterTileAppExt for App {
	fn register_raster_tile_data<T: tile_data::RasterTileCapability>(&mut self) -> &mut Self {
		if !self.world().contains_resource::<RasterTileCapabilityRegistry>() {
			self.init_resource::<RasterTileCapabilityRegistry>();
		}
		self.world_mut().resource_mut::<RasterTileCapabilityRegistry>().register::<T>();
		self
	}
}

#[derive(Default)]
pub struct VoxelRasterRendererPlugin;

impl Plugin for VoxelRasterRendererPlugin {
	fn build(&self, app: &mut App) {
		if !app.is_plugin_added::<VoxelDataPlugin>() {
			app.add_plugins(VoxelDataPlugin);
		}
		if !app.is_plugin_added::<GpuVoxelDataPlugin>() {
			app.add_plugins(GpuVoxelDataPlugin);
		}
		app.init_resource::<gpu_data::RasterWorldGpuData>()
			.init_resource::<RasterTileCapabilityRegistry>()
			.add_systems(bevy::prelude::Update, gpu_data::collect_raster_gpu_garbage);

		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return; };
		render_app
			.init_resource::<extract::ExtractedRasterScene>()
			.add_systems(ExtractSchedule, extract::extract_raster_scene)
			.add_systems(
				Render,
				render_node::prepare_raster_view_bind_groups.in_set(RenderSystems::PrepareBindGroups),
			)
			.add_systems(
				Core3d,
				render_node::voxel_raster_render_pass
					.in_set(Core3dSystems::MainPass)
					.before(main_opaque_pass_3d),
			);
	}

	fn finish(&self, app: &mut App) {
		let gpu = app.world().resource::<gpu_data::RasterWorldGpuData>().clone();
		gpu.initialize(app.world().resource(), app.world().resource());
		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return; };
		render_app
			.init_resource::<residency::RasterResidencyBuffers>()
			.init_resource::<voxel_raster_renderer_resource::VoxelRasterRendererResource>();
	}
}
