use bevy::ecs::system::{Res, ResMut};
use bevy::ecs::world::World;
use bevy::math::Mat4;
use bevy::render::renderer::{RenderContext, RenderDevice, RenderQueue, ViewQuery, WgpuWrapper};
use bevy::render::render_resource::StoreOp;
use bevy::render::view::{ViewDepthTexture, ViewTarget};

use crate::camera::CameraUniform;
use crate::extract::ExtractedRasterScene;
use crate::model::ModelUniform;
use crate::voxel_raster_renderer_resource::VoxelRasterRendererResource;

pub fn prepare_raster_view_bind_groups(
	render_device: Res<RenderDevice>,
	render_queue: Res<RenderQueue>,
	extracted: Res<ExtractedRasterScene>,
	mut raster_resource: ResMut<VoxelRasterRendererResource>,
	views: bevy::ecs::system::Query<&ViewTarget>,
) {
	let Some(view_target) = views.iter().next() else { return; };
	let main_texture = view_target.main_texture();
	let width = main_texture.width();
	let height = main_texture.height();
	let format = view_target.main_texture_format();

	raster_resource.ensure(render_device.wgpu_device(), width, height, format);
	if raster_resource.renderer.is_none() { return; }

	if !extracted.has_camera {
		raster_resource.face_bind_group = None;
		return;
	}

	let Some(face_buffer) = extracted.face_buffer.as_ref() else {
		raster_resource.face_bind_group = None;
		return;
	};
	let Some(palette_buffer) = extracted.palette_buffer.as_ref() else {
		raster_resource.face_bind_group = None;
		return;
	};

	let camera_uniform = CameraUniform::from_camera(
		&extracted.camera_transform,
		extracted.camera_projection.as_ref().expect("camera projection missing"),
	);
	render_queue.write_buffer(&raster_resource.camera_buffer, 0, camera_uniform.as_bytes());

	raster_resource.ensure_model_buffer_capacity(render_device.wgpu_device(), extracted.items.len());
	let mut model_bytes = Vec::with_capacity((raster_resource.model_stride as usize) * extracted.items.len().max(1));
	for item in &extracted.items {
		let start = model_bytes.len();
		let model = Mat4::from_scale_rotation_translation(item.transform.scale, item.transform.rotation, item.transform.translation);
		model_bytes.extend_from_slice(bytemuck::bytes_of(&ModelUniform::from_mat4(&model, item.palette_offset)));
		model_bytes.resize(start + raster_resource.model_stride as usize, 0);
	}
	if !model_bytes.is_empty() {
		render_queue.write_buffer(&raster_resource.model_buffer, 0, &model_bytes);
	}

	let face_bind_group_layout = raster_resource.renderer.as_ref().unwrap().face_bind_group_layout.clone();
	raster_resource.face_bind_group = Some(WgpuWrapper::new(render_device.wgpu_device().create_bind_group(&wgpu::BindGroupDescriptor {
		layout: &face_bind_group_layout,
		entries: &[
			wgpu::BindGroupEntry {
				binding: 0,
				resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
					buffer: face_buffer,
					offset: 0,
					size: None,
				}),
			},
			wgpu::BindGroupEntry {
				binding: 1,
				resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
					buffer: palette_buffer,
					offset: 0,
					size: None,
				}),
			},
		],
		label: Some("raster_face_bind_group"),
	})));
}

pub fn voxel_raster_render_pass(
	world: &World,
	view: ViewQuery<(&ViewTarget, &ViewDepthTexture)>,
	mut render_context: RenderContext,
) {
	let raster_resource = world.resource::<VoxelRasterRendererResource>();
	let extracted = world.resource::<ExtractedRasterScene>();
	let Some(renderer) = raster_resource.renderer.as_ref() else { return; };
	let Some(face_bind_group) = raster_resource.face_bind_group.as_ref() else { return; };
	if extracted.items.is_empty() { return; }

	let (view_target, view_depth) = view.into_inner();
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
	pass.set_bind_group(0, &*raster_resource.camera_bind_group, &[]);
	pass.set_bind_group(2, &**face_bind_group, &[]);
	for (index, item) in extracted.items.iter().enumerate() {
		let offset = (index as u64 * raster_resource.model_stride) as u32;
		pass.set_bind_group(1, &*raster_resource.model_bind_group, &[offset]);
		pass.draw(item.face_offset * 6..(item.face_offset + item.face_count) * 6, 0..1);
	}
}
