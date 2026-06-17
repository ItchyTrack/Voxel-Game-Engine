pub mod camera;
pub mod graphics_settings;
pub mod gpu_bvh;
pub mod hit_count_feedback;
pub mod voxel_renderer;
pub mod voxel_camera;
pub mod voxel_renderer_resource;

mod extract;
mod render_node;

pub use render_node::VoxelRenderLabel;

use bevy::app::{App, Plugin};
use bevy::core_pipeline::core_3d::graph::{Core3d, Node3d};
use bevy::ecs::schedule::IntoScheduleConfigs;
use bevy::render::extract_resource::ExtractResourcePlugin;
use bevy::render::{ExtractSchedule, Render, RenderApp, RenderSystems};
use bevy::render::render_graph::{RenderGraphExt, ViewNodeRunner};

use voxel_data::VoxelDataPlugin;
use voxel_gpu::{GpuVoxelDataPlugin};

use graphics_settings::GraphicsSettings;
use hit_count_feedback::{HitCountFeedback, LastGpuBvh, RenderStats};

#[derive(Default)]
pub struct VoxelRendererPlugin;

impl Plugin for VoxelRendererPlugin {
	fn build(&self, app: &mut App) {
		if !app.is_plugin_added::<VoxelDataPlugin>() {
			app.add_plugins(VoxelDataPlugin);
		}
		if !app.is_plugin_added::<GpuVoxelDataPlugin>() {
			app.add_plugins(GpuVoxelDataPlugin);
		}
		let render_stats = RenderStats::default();
		app.insert_resource(render_stats.clone())
			.init_resource::<GraphicsSettings>()
			.add_plugins(ExtractResourcePlugin::<GraphicsSettings>::default());

		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return };
		render_app
			.insert_resource(render_stats)
			.init_resource::<HitCountFeedback>()
			.init_resource::<LastGpuBvh>()
			.init_resource::<extract::ExtractedVoxelScene>()
			.init_resource::<render_node::VoxelViewBindGroups>()
			.add_systems(ExtractSchedule, extract::extract_voxel_scene)
			.add_systems(
				Render,
				hit_count_feedback::read_back_hit_counts
					.in_set(RenderSystems::PrepareResources),
			)
			.add_systems(
				Render,
				render_node::prepare_voxel_view_bind_groups
					.in_set(RenderSystems::PrepareBindGroups),
			)
			.add_render_graph_node::<ViewNodeRunner<render_node::VoxelRenderNode>>(
				Core3d,
				render_node::VoxelRenderLabel,
			)
			.add_render_graph_edges(
				Core3d,
				(
					Node3d::StartMainPass,
					render_node::VoxelRenderLabel,
					Node3d::MainOpaquePass,
				),
			);
	}

	fn finish(&self, app: &mut App) {
		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return };
		render_app.init_resource::<voxel_renderer_resource::VoxelRendererResource>();
	}
}
