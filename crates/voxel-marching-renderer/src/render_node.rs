use bevy::prelude::*;
use bevy::render::render_resource::{BindGroupEntries, PipelineCache, StoreOp};
use bevy::render::renderer::{RenderContext, RenderDevice, RenderQueue, ViewQuery};
use bevy::render::view::{ExtractedView, ViewDepthTexture, ViewTarget, ViewUniformOffset, ViewUniforms};

use voxel_gpu::SlangShaderParam;
use crate::{
	MarchingShader,
	camera::CameraUniform,
	extract::ExtractedMarchingScene,
	model::ModelUniform,
	renderer_resource::{MarchingRendererResource, MarchingViewResources},
};

pub fn prepare_marching_view_bind_groups(
	mut commands: Commands,
	device: Res<RenderDevice>,
	queue: Res<RenderQueue>,
	cache: Res<PipelineCache>,
	view_uniforms: Res<ViewUniforms>,
	mut shader: SlangShaderParam<MarchingShader>,
	mut shared: ResMut<MarchingRendererResource>,
	mut views: Query<(Entity, &ExtractedView, &ViewTarget, &ViewUniformOffset, &ExtractedMarchingScene, Option<&mut MarchingViewResources>)>,
) {
	let shader = shader.get().and_then(|shader| shader.bevy_shader().cloned());
	for (entity, view, target, _, extracted, prepared) in &mut views {
		let Some(shader) = shader.as_ref() else { continue };
		if view_uniforms.uniforms.binding().is_none() { continue; }
		let pipeline = shared.pipeline(&cache, target.main_texture_format(), shader);
		if let Some(mut prepared) = prepared {
			prepare_view(&mut prepared, view, extracted, pipeline, &device, &queue, &cache, &shared);
		} else {
			let mut prepared = MarchingViewResources::new(&device, &queue, &cache, &shared);
			prepare_view(&mut prepared, view, extracted, pipeline, &device, &queue, &cache, &shared);
			commands.entity(entity).insert(prepared);
		}
	}
}

fn prepare_view(
	prepared: &mut MarchingViewResources,
	view: &ExtractedView,
	extracted: &ExtractedMarchingScene,
	pipeline: bevy::render::render_resource::CachedRenderPipelineId,
	device: &RenderDevice,
	queue: &RenderQueue,
	cache: &PipelineCache,
	shared: &MarchingRendererResource,
) {
	prepared.camera_buffer.set(CameraUniform::from_view(view));
	prepared.camera_buffer.write_buffer(device, queue);
	let models = extracted.items.iter().map(|item| ModelUniform::from_mat4(item.transform.to_matrix())).collect();
	prepared.write_models(models, device, queue, cache, shared);
	prepared.pipeline = Some(pipeline);
}

pub fn marching_render_pass(
	shared: Res<MarchingRendererResource>,
	cache: Res<PipelineCache>,
	view: ViewQuery<(&ViewTarget, &ViewDepthTexture, &ExtractedMarchingScene, &MarchingViewResources)>,
	mut context: RenderContext,
) {
	let (target, depth, extracted, prepared) = view.into_inner();
	let Some(pipeline_id) = prepared.pipeline else { return };
	let Some(pipeline) = cache.get_render_pipeline(pipeline_id) else { return };
	if extracted.items.is_empty() { return; }

	let vertex_layout = cache.get_bind_group_layout(&shared.vertex_layout);
	let vertex_bind_groups: Vec<_> = extracted.items.iter().map(|item| {
		context.render_device().create_bind_group(
			"marching_vertex_bind_group",
			&vertex_layout,
			&BindGroupEntries::single(wgpu::BufferBinding { buffer: &item.vertex_buffer, offset: 0, size: None }),
		)
	}).collect();
	let color_attachment = target.get_color_attachment();
	let mut pass = context.command_encoder().begin_render_pass(&wgpu::RenderPassDescriptor {
		label: Some("Marching Cubes Render Pass"),
		color_attachments: &[Some(color_attachment)],
		depth_stencil_attachment: Some(depth.get_attachment(StoreOp::Store)),
		occlusion_query_set: None,
		timestamp_writes: None,
		multiview_mask: None,
	});
	pass.set_pipeline(pipeline);
	pass.set_bind_group(0, &*prepared.camera_bind_group, &[]);
	pass.set_bind_group(1, &*prepared.model_bind_group, &[]);
	for (index, (item, vertex_bind_group)) in extracted.items.iter().zip(&vertex_bind_groups).enumerate() {
		pass.set_bind_group(2, &**vertex_bind_group, &[]);
		pass.draw(0..item.vertex_count, index as u32..index as u32 + 1);
	}
}
