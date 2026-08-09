use bevy::ecs::system::{Commands, Query, Res, ResMut};
use bevy::math::Mat4;
use bevy::render::render_resource::{BindGroupEntries, PipelineCache, StoreOp};
use bevy::render::renderer::{RenderContext, RenderDevice, RenderQueue, ViewQuery};
use bevy::render::view::{ExtractedView, ViewDepthTexture, ViewTarget, ViewUniformOffset, ViewUniforms};

use crate::camera::CameraUniform;
use crate::extract::ExtractedRasterScene;
use crate::model::ModelUniform;
use crate::voxel_raster_renderer_resource::{RasterViewResources, VoxelRasterRendererResource};
use crate::VoxelRasterShader;
use voxel_gpu::SlangShaderParam;

pub fn prepare_raster_view_bind_groups(
	mut commands: Commands,
	render_device: Res<RenderDevice>,
	render_queue: Res<RenderQueue>,
	pipeline_cache: Res<PipelineCache>,
	view_uniforms: Res<ViewUniforms>,
	mut shader: SlangShaderParam<VoxelRasterShader>,
	mut raster_resource: ResMut<VoxelRasterRendererResource>,
	mut views: Query<(
		bevy::ecs::entity::Entity,
		&ExtractedView,
		&ViewTarget,
		&ViewUniformOffset,
		&ExtractedRasterScene,
		Option<&mut RasterViewResources>,
	)>,
) {
	let view_uniforms_ready = view_uniforms.uniforms.binding().is_some();
	let shader = shader.get().and_then(|shader| shader.bevy_shader().cloned());
	for (entity, extracted_view, view_target, _view_uniform_offset, extracted, prepared) in &mut views {
		let Some(shader) = shader.as_ref() else {
			if let Some(mut prepared) = prepared { prepared.pipeline = None; }
			continue;
		};
		if !view_uniforms_ready {
			if let Some(mut prepared) = prepared { prepared.pipeline = None; }
			continue;
		}

		let pipeline = raster_resource.pipeline(
			&pipeline_cache,
			view_target.main_texture_format(),
			shader,
		);
		if let Some(mut prepared) = prepared {
			prepare_view(
				&mut prepared,
				extracted_view,
				extracted,
				pipeline,
				&render_device,
				&render_queue,
				&pipeline_cache,
				&raster_resource,
			);
		} else {
			let mut prepared = RasterViewResources::new(
				&render_device,
				&render_queue,
				&pipeline_cache,
				&raster_resource,
			);
			prepare_view(
				&mut prepared,
				extracted_view,
				extracted,
				pipeline,
				&render_device,
				&render_queue,
				&pipeline_cache,
				&raster_resource,
			);
			commands.entity(entity).insert(prepared);
		}
	}
}

fn prepare_view(
	prepared: &mut RasterViewResources,
	extracted_view: &ExtractedView,
	extracted: &ExtractedRasterScene,
	pipeline: bevy::render::render_resource::CachedRenderPipelineId,
	render_device: &RenderDevice,
	render_queue: &RenderQueue,
	pipeline_cache: &PipelineCache,
	shared: &VoxelRasterRendererResource,
) {
	prepared.camera_buffer.set(CameraUniform::from_view(extracted_view));
	prepared.camera_buffer.write_buffer(render_device, render_queue);
	let models = extracted.items.iter()
		.map(|item| {
			let model = Mat4::from_scale_rotation_translation(
				item.transform.scale,
				item.transform.rotation,
				item.transform.translation,
			);
			ModelUniform::from_mat4(model, item.palette_offset)
		})
		.collect();
	prepared.write_models(models, render_device, render_queue, pipeline_cache, shared);
	prepared.pipeline = Some(pipeline);
}

pub fn voxel_raster_render_pass(
	raster_resource: Res<VoxelRasterRendererResource>,
	pipeline_cache: Res<PipelineCache>,
	view: ViewQuery<(
		&ViewTarget,
		&ViewDepthTexture,
		&ExtractedRasterScene,
		&RasterViewResources,
	)>,
	mut render_context: RenderContext,
) {
	let (view_target, view_depth, extracted, prepared) = view.into_inner();
	let Some(pipeline_id) = prepared.pipeline else { return };
	let Some(pipeline) = pipeline_cache.get_render_pipeline(pipeline_id) else { return };
	if extracted.items.is_empty() { return; }

	let render_device = render_context.render_device();
	let face_bind_group = render_device.create_bind_group(
		"raster_face_bind_group",
		&pipeline_cache.get_bind_group_layout(&raster_resource.face_bind_group_layout),
		&BindGroupEntries::sequential((
			wgpu::BufferBinding {
				buffer: &extracted.face_buffer,
				offset: 0,
				size: None,
			},
			wgpu::BufferBinding {
				buffer: &extracted.palette_buffer,
				offset: 0,
				size: None,
			},
		)),
	);

	let color_attachment = view_target.get_color_attachment();
	let encoder = render_context.command_encoder();
	let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
		label: Some("Voxel Raster Render Pass"),
		color_attachments: &[Some(color_attachment)],
		depth_stencil_attachment: Some(view_depth.get_attachment(StoreOp::Store)),
		occlusion_query_set: None,
		timestamp_writes: None,
		multiview_mask: None,
	});
	pass.set_pipeline(pipeline);
	pass.set_bind_group(0, &*prepared.camera_bind_group, &[]);
	pass.set_bind_group(1, &*prepared.model_bind_group, &[]);
	pass.set_bind_group(2, &*face_bind_group, &[]);
	for (index, item) in extracted.items.iter().enumerate() {
		let instance = index as u32;
		pass.draw(
			item.face_offset * 6..(item.face_offset + item.face_count) * 6,
			instance..instance + 1,
		);
	}
}
