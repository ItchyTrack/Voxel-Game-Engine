pub mod camera;
pub mod render_node;
pub mod voxel_camera;

mod extract;
mod model;
mod residency;
mod voxel_raster_renderer;
mod voxel_raster_renderer_resource;

use bevy::app::{App, Plugin};
use bevy::core_pipeline::core_3d::main_opaque_pass_3d;
use bevy::core_pipeline::{Core3d, Core3dSystems};
use bevy::ecs::schedule::IntoScheduleConfigs;
use bevy::render::{ExtractSchedule, Render, RenderApp, RenderSystems};

use voxel_data::VoxelDataPlugin;
use voxel_gpu::GpuVoxelDataPlugin;

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
		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return; };
		render_app
			.init_resource::<residency::RasterResidencyBuffers>()
			.init_resource::<voxel_raster_renderer_resource::VoxelRasterRendererResource>();
	}
}
