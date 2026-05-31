pub mod camera;
pub mod crosshair_renderer;
pub mod graphics_settings;
pub mod hit_count_feedback;
pub mod scene;
pub mod voxel_renderer;
pub mod voxel_renderer_resource;

mod crosshair_node;
mod extract;
mod render_node;

use bevy::app::{App, Plugin, Update};
use bevy::core_pipeline::core_3d::graph::{Core3d, Node3d};
use bevy::ecs::schedule::IntoScheduleConfigs;
use bevy::render::extract_resource::ExtractResourcePlugin;
use bevy::render::{ExtractSchedule, Render, RenderApp, RenderSystems};
use bevy::render::render_graph::{RenderGraphExt, ViewNodeRunner};

use voxel_data::VoxelDataPlugin;
use gpu_voxel_data::{GpuUploadSet, GpuVoxelDataAppExt, GpuVoxelDataPlugin};

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
		app.register_lod_requester::<scene::RenderLod>();

		let hit_count_feedback = HitCountFeedback::default();
		let render_stats = RenderStats::default();
		app.insert_resource(hit_count_feedback.clone())
			.insert_resource(render_stats.clone())
			.init_resource::<GraphicsSettings>()
			.add_plugins(ExtractResourcePlugin::<GraphicsSettings>::default())
			.add_systems(Update, scene::update_render_lod.before(GpuUploadSet::Collect));

		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return };
		render_app
			.insert_resource(hit_count_feedback)
			.insert_resource(render_stats)
			.init_resource::<LastGpuBvh>()
			.init_resource::<extract::ExtractedVoxelScene>()
			.init_resource::<render_node::VoxelViewBindGroups>()
			.init_resource::<crosshair_renderer::CrosshairResource>()
			.add_systems(ExtractSchedule, extract::extract_voxel_scene)
			.add_systems(
				Render,
				hit_count_feedback::read_back_hit_counts
					.in_set(RenderSystems::PrepareResources),
			)
			.add_systems(
				Render,
				(
					render_node::prepare_voxel_view_bind_groups,
					crosshair_node::prepare_crosshair,
				).in_set(RenderSystems::PrepareBindGroups),
			)
			.add_render_graph_node::<ViewNodeRunner<render_node::VoxelRenderNode>>(
				Core3d,
				render_node::VoxelRenderLabel,
			)
			.add_render_graph_node::<ViewNodeRunner<crosshair_node::CrosshairRenderNode>>(
				Core3d,
				crosshair_node::CrosshairRenderLabel,
			)
			.add_render_graph_edges(
				Core3d,
				(
					Node3d::StartMainPass,
					render_node::VoxelRenderLabel,
					Node3d::MainOpaquePass,
				),
			)
			.add_render_graph_edges(
				Core3d,
				(
					Node3d::Tonemapping,
					crosshair_node::CrosshairRenderLabel,
					Node3d::EndMainPassPostProcessing,
				),
			);
	}

	fn finish(&self, app: &mut App) {
		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return };
		render_app.init_resource::<voxel_renderer_resource::VoxelRendererResource>();
	}
}
