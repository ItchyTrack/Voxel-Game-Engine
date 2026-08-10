use bevy::prelude::*;
use bevy::render::render_resource::{BindGroupEntries, PipelineCache, StoreOp};
use bevy::render::renderer::{RenderContext, RenderDevice, RenderQueue, ViewQuery};
use bevy::render::view::{ViewDepthTexture, ViewTarget, ViewUniformOffset, ViewUniforms};

use voxel_gpu::SlangShaderParam;
use crate::{
	MarchingShader,
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
	mut views: Query<(Entity, &ViewTarget, &ViewUniformOffset, &ExtractedMarchingScene, Option<&mut MarchingViewResources>)>,
) {
	let Some(view_binding) = view_uniforms.uniforms.binding() else {
		shared.view_bind_group = None;
		for (_, _, _, _, prepared) in &mut views {
			if let Some(mut prepared) = prepared { prepared.pipeline = None; }
		}
		return;
	};
	shared.view_bind_group = Some(device.create_bind_group(
		"marching_view_bind_group",
		&cache.get_bind_group_layout(&shared.view_layout),
		&BindGroupEntries::single(view_binding),
	));

	let shader = shader.get().and_then(|shader| shader.bevy_shader().cloned());
	for (entity, target, view_offset, extracted, prepared) in &mut views {
		let Some(shader) = shader.as_ref() else {
			if let Some(mut prepared) = prepared { prepared.pipeline = None; }
			continue;
		};
		let pipeline = shared.pipeline(&cache, target.main_texture_format(), shader);
		if let Some(mut prepared) = prepared {
			prepare_view(&mut prepared, view_offset.offset, extracted, pipeline, &device, &queue, &cache, &shared);
		} else {
			let mut prepared = MarchingViewResources::new(&device, &queue, &cache, &shared);
			prepare_view(&mut prepared, view_offset.offset, extracted, pipeline, &device, &queue, &cache, &shared);
			commands.entity(entity).insert(prepared);
		}
	}
}

fn prepare_view(
	prepared: &mut MarchingViewResources,
	view_uniform_offset: u32,
	extracted: &ExtractedMarchingScene,
	pipeline: bevy::render::render_resource::CachedRenderPipelineId,
	device: &RenderDevice,
	queue: &RenderQueue,
	cache: &PipelineCache,
	shared: &MarchingRendererResource,
) {
	prepared.view_uniform_offset = view_uniform_offset;
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
	let Some(view_bind_group) = shared.view_bind_group.as_ref() else { return };
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
	pass.set_bind_group(0, view_bind_group, &[prepared.view_uniform_offset]);
	pass.set_bind_group(1, &*prepared.model_bind_group, &[]);
	for (index, (item, vertex_bind_group)) in extracted.items.iter().zip(&vertex_bind_groups).enumerate() {
		pass.set_bind_group(2, &**vertex_bind_group, &[]);
		pass.draw(0..item.vertex_count, index as u32..index as u32 + 1);
	}
}
