pub mod camera;
pub mod graphics_settings;
pub mod gpu_bvh;
pub mod gpu_data;
pub mod gpu_grid_tree;
pub mod residency;
pub mod direction_feedback;
pub mod incoming_ray_directions;
pub mod voxel_renderer;
pub mod voxel_camera;
pub mod tile_data;
pub mod voxel_renderer_resource;

mod extract;
pub mod render_node;
#[cfg(all(feature = "shader_hot_reload", not(target_arch = "wasm32")))]
mod shader_hot_reload;
mod shader_sources;

use bevy::app::{App, Plugin};
use bevy::prelude::Resource;
use bevy::core_pipeline::core_3d::main_opaque_pass_3d;
use bevy::core_pipeline::{Core3d, Core3dSystems};
use bevy::ecs::schedule::IntoScheduleConfigs;
use bevy::render::extract_resource::ExtractResourcePlugin;
use bevy::render::{ExtractSchedule, Render, RenderApp, RenderSystems};

use ::tile_data::{TileCapabilityRegistry, TileData};
use voxel_data::VoxelDataPlugin;
use voxel_gpu::{GpuVoxelDataPlugin};

use graphics_settings::GraphicsSettings;
use direction_feedback::{DirectionFeedback, LastGpuBvh, RenderStats};

#[derive(Resource, Default)]
pub struct RayTileCapabilityRegistry(TileCapabilityRegistry<tile_data::RayTileCapabilityData>);

impl RayTileCapabilityRegistry {
	pub fn read(&self, data: &dyn TileData) -> Option<tile_data::RayTileCapabilityData> { self.0.read(data) }

	pub fn register<T: tile_data::RayTileCapability>(&mut self) {
		self.0.register::<T>(|data| data.ray_tile_data());
	}
}

pub trait VoxelRayTileAppExt {
	fn register_ray_tile_data<T: tile_data::RayTileCapability>(&mut self) -> &mut Self;
}

impl VoxelRayTileAppExt for App {
	fn register_ray_tile_data<T: tile_data::RayTileCapability>(&mut self) -> &mut Self {
		if !self.world().contains_resource::<RayTileCapabilityRegistry>() {
			self.init_resource::<RayTileCapabilityRegistry>();
		}
		self.world_mut().resource_mut::<RayTileCapabilityRegistry>().register::<T>();
		self
	}
}

#[derive(Default)]
pub struct VoxelRayRendererPlugin;

impl Plugin for VoxelRayRendererPlugin {
	fn build(&self, app: &mut App) {
		if !app.is_plugin_added::<VoxelDataPlugin>() {
			app.add_plugins(VoxelDataPlugin);
		}
		if !app.is_plugin_added::<GpuVoxelDataPlugin>() {
			app.add_plugins(GpuVoxelDataPlugin);
		}
		let render_stats = RenderStats::default();
		app.init_resource::<gpu_data::RayWorldGpuData>()
			.init_resource::<RayTileCapabilityRegistry>()
			.insert_resource(render_stats.clone())
			.init_resource::<GraphicsSettings>()
			.add_systems(bevy::prelude::Update, gpu_data::collect_ray_gpu_garbage)
			.add_plugins(ExtractResourcePlugin::<GraphicsSettings>::default());

		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return };
		render_app
			.insert_resource(render_stats)
			.init_resource::<DirectionFeedback>()
			.init_resource::<LastGpuBvh>()
			.init_resource::<extract::ExtractedVoxelScene>()
			.init_resource::<render_node::VoxelViewBindGroups>()
			.add_systems(ExtractSchedule, extract::extract_voxel_scene)
			.add_systems(
				Render,
				direction_feedback::read_back_direction_masks
					.in_set(RenderSystems::PrepareResources),
			)
			.add_systems(
				Render,
				render_node::prepare_voxel_view_bind_groups
					.in_set(RenderSystems::PrepareBindGroups),
			)
			.add_systems(
				Core3d,
				render_node::voxel_render_pass
					.in_set(Core3dSystems::MainPass)
					.before(main_opaque_pass_3d),
			);
	}

	fn finish(&self, app: &mut App) {
		let gpu = app.world().resource::<gpu_data::RayWorldGpuData>().clone();
		gpu.initialize(app.world().resource(), app.world().resource());
		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return };
		render_app
			.init_resource::<residency::ResidencyBuffers>()
			.init_resource::<voxel_renderer_resource::VoxelRendererResource>();
	}
}
