use bevy::ecs::system::{Commands, Query, Res, ResMut};
use bevy::render::renderer::{RenderContext, RenderDevice, RenderQueue, ViewQuery, WgpuWrapper};
use bevy::render::view::{ExtractedView, ViewDepthTexture, ViewTarget, ViewUniformOffset, ViewUniforms};

use crate::direction_feedback::{DirectionFeedback, RenderStats};
use crate::extract::ExtractedVoxelScenes;
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
	extracted_scenes: Res<ExtractedVoxelScenes>,
	mut shader: SlangShaderParam<VoxelRayShader>,
	mut voxel_resource: ResMut<VoxelRendererResource>,
	mut views: Query<(
		bevy::ecs::entity::Entity,
		&ExtractedView,
		&ViewTarget,
		&ViewUniformOffset,
		Option<&mut VoxelViewResources>,
	)>,
) {
	let settings = RenderSettingsUniform::from_graphics_settings(&graphics_settings);
	voxel_resource.render_settings_buffer.set(settings);
	voxel_resource.render_settings_buffer.write_buffer(&render_device, &render_queue);

	let Some(view_binding) = view_uniforms.uniforms.binding() else {
		voxel_resource.view_bind_group = None;
		for (_, _, _, _, prepared) in &mut views {
			if let Some(mut prepared) = prepared { prepared.ready = false; }
		}
		return;
	};
	let device = render_device.wgpu_device();
	voxel_resource.view_bind_group = Some(WgpuWrapper::new(device.create_bind_group(&wgpu::BindGroupDescriptor {
		layout: &voxel_resource.view_bind_group_layout,
		entries: &[
			wgpu::BindGroupEntry { binding: 0, resource: view_binding },
			wgpu::BindGroupEntry {
				binding: 1,
				resource: voxel_resource.render_settings_buffer.binding().expect("render settings uniform must be initialized"),
			},
		],
		label: Some("voxel_view_bind_group"),
	})));
	let shader = shader.get();

	for (entity, extracted_view, view_target, view_offset, prepared) in &mut views {
		if !extracted_scenes.0.contains_key(&entity) {
			if let Some(mut prepared) = prepared { prepared.ready = false; }
			continue;
		}
		let Some(shader) = shader.as_ref() else {
			if let Some(mut prepared) = prepared { prepared.ready = false; }
			continue;
		};
		if extracted_view.clip_from_view.w_axis.w != 0.0 {
			log::warn!("voxel renderer: unsupported camera projection (expected perspective)");
			if let Some(mut prepared) = prepared { prepared.ready = false; }
			continue;
		}
		let main_texture = view_target.main_texture();
		let width = main_texture.width();
		let height = main_texture.height();
		let format = view_target.main_texture_format();

		if let Some(mut prepared) = prepared {
			prepared.ready = false;
			prepared.view_uniform_offset = view_offset.offset;
			prepared.ensure(device, width, height, format, &voxel_resource.view_bind_group_layout, shader);
			prepared.ready = prepared.voxel_renderer.is_some();
		} else {
			let mut prepared = VoxelViewResources::new();
			prepared.view_uniform_offset = view_offset.offset;
			prepared.ensure(device, width, height, format, &voxel_resource.view_bind_group_layout, shader);
			prepared.ready = prepared.voxel_renderer.is_some();
			commands.entity(entity).insert(prepared);
		}
	}
}

pub fn voxel_render_pass(
	voxel_resource: Res<VoxelRendererResource>,
	render_stats: Res<RenderStats>,
	extracted_scenes: Res<ExtractedVoxelScenes>,
	view: ViewQuery<(
		bevy::ecs::entity::Entity,
		&ViewTarget,
		&ViewDepthTexture,
		&VoxelViewResources,
		&mut DirectionFeedback,
	)>,
	mut render_context: RenderContext,
) {
	let (entity, view_target, view_depth, prepared, mut feedback) = view.into_inner();
	if !prepared.ready { return; }
	let Some(voxel_renderer) = prepared.voxel_renderer.as_ref() else { return };
	let Some(view_bind_group) = voxel_resource.view_bind_group.as_ref() else { return };
	let Some(extracted) = extracted_scenes.0.get(&entity) else { return };
	let Some(bvh) = extracted.bvh.as_ref() else { return };

	let device = render_context.render_device().wgpu_device().clone();
	let encoder = render_context.command_encoder();
	let color_attachment = view_target.get_color_attachment();
	let depth_sample_view = view_depth.texture.create_view(&wgpu::TextureViewDescriptor {
		label: Some("voxel_raycast_depth_sample_view"),
		format: None,
		dimension: None,
		usage: Some(wgpu::TextureUsages::TEXTURE_BINDING),
		aspect: wgpu::TextureAspect::DepthOnly,
		base_mip_level: 0,
		mip_level_count: None,
		base_array_layer: 0,
		array_layer_count: None,
	});

	let gpu_bvh = voxel_renderer.render(
		&device,
		encoder,
		view_bind_group,
		prepared.view_uniform_offset,
		bvh,
		&extracted.bvh_item_data,
		&extracted.tree_buffer,
		&extracted.voxel_buffer,
		&extracted.main_tree_buffer,
		&extracted.main_voxel_buffer,
		&depth_sample_view,
		color_attachment,
	);

	if let Ok(mut stats) = render_stats.inner.lock() {
		stats.bvh_bytes = gpu_bvh.bvh_buffer.size();
		stats.bvh_leaf_bytes = gpu_bvh.items_buffer.size();
	}
	feedback.push_rendered(gpu_bvh);
}
