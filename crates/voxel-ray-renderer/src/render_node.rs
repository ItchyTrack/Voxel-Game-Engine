use bevy::ecs::system::{Commands, Query, Res, ResMut};
use bevy::render::render_asset::RenderAssets;
use bevy::render::renderer::{RenderContext, RenderDevice, RenderQueue, ViewQuery};
use bevy::render::storage::GpuShaderBuffer;
use bevy::render::view::{ExtractedView, ViewTarget, ViewUniformOffset, ViewUniforms};

use crate::camera::CameraUniform;
use crate::direction_feedback::RenderStats;
use crate::extract::ExtractedVoxelScene;
use crate::graphics_settings::{GraphicsSettings, RenderSettingsUniform};
use crate::voxel_renderer_resource::{VoxelRendererResource, VoxelViewResources};
use crate::VoxelRayShader;
use voxel_gpu::SlangShaderParam;

pub fn prepare_voxel_view_bind_groups(
	mut commands: Commands,
	render_device: Res<RenderDevice>,
	render_queue: Res<RenderQueue>,
	view_uniforms: Res<ViewUniforms>,
	graphics_settings: Res<GraphicsSettings>,
	mut shader: SlangShaderParam<VoxelRayShader>,
	mut voxel_resource: ResMut<VoxelRendererResource>,
	mut views: Query<(
		bevy::ecs::entity::Entity,
		&ExtractedView,
		&ViewTarget,
		&ViewUniformOffset,
		Option<&mut VoxelViewResources>,
	), bevy::ecs::query::With<ExtractedVoxelScene>>,
) {
	let view_uniforms_ready = view_uniforms.uniforms.binding().is_some();
	let settings = RenderSettingsUniform::from_graphics_settings(&graphics_settings);
	voxel_resource.render_settings_buffer.set(settings);
	voxel_resource.render_settings_buffer.write_buffer(&render_device, &render_queue);
	let shader = shader.get();

	for (entity, extracted_view, view_target, _view_uniform_offset, prepared) in &mut views {
		let Some(shader) = shader.as_ref() else {
			if let Some(mut prepared) = prepared { prepared.ready = false; }
			continue;
		};
		if !view_uniforms_ready {
			if let Some(mut prepared) = prepared { prepared.ready = false; }
			continue;
		}
		let Ok(camera_uniform) = CameraUniform::from_view(extracted_view) else {
			log::warn!("voxel renderer: unsupported camera projection (expected perspective)");
			if let Some(mut prepared) = prepared { prepared.ready = false; }
			continue;
		};
		let main_texture = view_target.main_texture();
		let width = main_texture.width();
		let height = main_texture.height();
		let format = view_target.main_texture_format();
		let device = render_device.wgpu_device();

		if let Some(mut prepared) = prepared {
			prepared.ready = false;
			prepared.camera_buffer.set(camera_uniform);
			prepared.camera_buffer.write_buffer(&render_device, &render_queue);
			prepared.ensure(
				device,
				width,
				height,
				format,
				&voxel_resource.camera_bind_group_layout,
				shader,
			);
			prepared.ready = prepared.voxel_renderer.is_some();
		} else {
			let mut prepared = VoxelViewResources::new(device, &render_device, &render_queue, &voxel_resource);
			prepared.camera_buffer.set(camera_uniform);
			prepared.camera_buffer.write_buffer(&render_device, &render_queue);
			prepared.ensure(
				device,
				width,
				height,
				format,
				&voxel_resource.camera_bind_group_layout,
				shader,
			);
			prepared.ready = prepared.voxel_renderer.is_some();
			commands.entity(entity).insert(prepared);
		}
	}
}

pub fn voxel_render_pass(
	render_stats: Res<RenderStats>,
	shader_buffers: Res<RenderAssets<GpuShaderBuffer>>,
	view: ViewQuery<(
		&ViewTarget,
		&ExtractedVoxelScene,
		&VoxelViewResources,
	)>,
	mut render_context: RenderContext,
) {
	let (view_target, extracted, prepared) = view.into_inner();
	if !prepared.ready { return; }
	let Some(voxel_renderer) = prepared.voxel_renderer.as_ref() else { return };
	let Some(bvh) = extracted.bvh.as_ref() else { return };
	let Some(direction_mask_handle) = extracted.direction_mask_buffer.as_ref() else { return };
	let Some(direction_mask_buffer) = shader_buffers.get(direction_mask_handle) else { return };

	let device = render_context.render_device().wgpu_device().clone();
	let encoder = render_context.command_encoder();
	let main_texture = view_target.main_texture();
	let color_attachment = view_target.get_color_attachment();

	let gpu_bvh = voxel_renderer.render(
		&device,
		encoder,
		main_texture.width(),
		main_texture.height(),
		&prepared.camera_bind_group,
		bvh,
		&extracted.bvh_item_data,
		&extracted.tree_buffer,
		&extracted.voxel_buffer,
		&extracted.main_tree_buffer,
		&extracted.main_voxel_buffer,
		&direction_mask_buffer.buffer,
		color_attachment,
	);

	if let Ok(mut stats) = render_stats.inner.lock() {
		stats.bvh_bytes = gpu_bvh.bvh_buffer.size();
		stats.bvh_leaf_bytes = gpu_bvh.items_buffer.size();
	}
}
