use bevy::ecs::system::{Commands, Query, Res};
use bevy::math::Mat4;
use bevy::render::render_resource::StoreOp;
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
	view_uniforms: Res<ViewUniforms>,
	mut shader: SlangShaderParam<VoxelRasterShader>,
	raster_resource: Res<VoxelRasterRendererResource>,
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
	let shader = shader.get();
	for (entity, extracted_view, view_target, _view_uniform_offset, extracted, prepared) in &mut views {
		let Some(shader) = shader.as_ref() else {
			if let Some(mut prepared) = prepared { prepared.ready = false; }
			continue;
		};
		if !view_uniforms_ready {
			if let Some(mut prepared) = prepared { prepared.ready = false; }
			continue;
		}
		let format = view_target.main_texture_format();
		let device = render_device.wgpu_device();
		let camera_uniform = CameraUniform::from_view(extracted_view);

		if let Some(mut prepared) = prepared {
			prepared.ready = false;
			prepared.camera_buffer.set(camera_uniform);
			prepared.camera_buffer.write_buffer(&render_device, &render_queue);
			prepare_models(
				&mut prepared,
				extracted,
				device,
				&render_device,
				&render_queue,
				&raster_resource,
			);
			prepared.ensure(
				device,
				format,
				&raster_resource.camera_bind_group_layout,
				&raster_resource.model_bind_group_layout,
				shader,
			);
			prepared.ready = prepared.renderer.is_some();
		} else {
			let mut prepared = RasterViewResources::new(device, &render_device, &render_queue, &raster_resource);
			prepared.camera_buffer.set(camera_uniform);
			prepared.camera_buffer.write_buffer(&render_device, &render_queue);
			prepare_models(
				&mut prepared,
				extracted,
				device,
				&render_device,
				&render_queue,
				&raster_resource,
			);
			prepared.ensure(
				device,
				format,
				&raster_resource.camera_bind_group_layout,
				&raster_resource.model_bind_group_layout,
				shader,
			);
			prepared.ready = prepared.renderer.is_some();
			commands.entity(entity).insert(prepared);
		}
	}
}

fn prepare_models(
	prepared: &mut RasterViewResources,
	extracted: &ExtractedRasterScene,
	device: &wgpu::Device,
	render_device: &RenderDevice,
	render_queue: &RenderQueue,
	shared: &VoxelRasterRendererResource,
) {
	prepared.model_buffer.clear();
	prepared.model_buffer.push(&ModelUniform::default());
	prepared.model_offsets.clear();
	for item in &extracted.items {
		let model = Mat4::from_scale_rotation_translation(item.transform.scale, item.transform.rotation, item.transform.translation);
		let offset = prepared.model_buffer.push(&ModelUniform::from_mat4(&model, item.palette_offset));
		prepared.model_offsets.push(offset);
	}
	prepared.model_buffer.write_buffer(render_device, render_queue);
	prepared.model_bind_group = ModelUniform::get_bind_group(
		device,
		&shared.model_bind_group_layout,
		&prepared.model_buffer,
		0,
	);
}

pub fn voxel_raster_render_pass(
	view: ViewQuery<(
		&ViewTarget,
		&ViewDepthTexture,
		&ExtractedRasterScene,
		&RasterViewResources,
	)>,
	mut render_context: RenderContext,
) {
	let (view_target, view_depth, extracted, prepared) = view.into_inner();
	if !prepared.ready { return; }
	let Some(renderer) = prepared.renderer.as_ref() else { return };
	if extracted.items.is_empty() { return; }

	let face_bind_group = render_context.render_device().wgpu_device().create_bind_group(&wgpu::BindGroupDescriptor {
		layout: &renderer.face_bind_group_layout,
		entries: &[
			wgpu::BindGroupEntry {
				binding: 0,
				resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
					buffer: &extracted.face_buffer,
					offset: 0,
					size: None,
				}),
			},
			wgpu::BindGroupEntry {
				binding: 1,
				resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
					buffer: &extracted.palette_buffer,
					offset: 0,
					size: None,
				}),
			},
		],
		label: Some("raster_face_bind_group"),
	});

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
	pass.set_pipeline(&renderer.pipeline);
	pass.set_bind_group(0, &*prepared.camera_bind_group, &[]);
	pass.set_bind_group(2, &face_bind_group, &[]);
	for (item, offset) in extracted.items.iter().zip(&prepared.model_offsets) {
		pass.set_bind_group(1, &*prepared.model_bind_group, &[*offset]);
		pass.draw(item.face_offset * 6..(item.face_offset + item.face_count) * 6, 0..1);
	}
}
