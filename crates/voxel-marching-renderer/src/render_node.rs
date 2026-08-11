use bevy::core_pipeline::core_3d::{Opaque3d, Opaque3dBatchSetKey, Opaque3dBinKey};
use bevy::ecs::{
	query::ROQueryItem,
	system::{Commands, Query, Res, ResMut, SystemParamItem, lifetimeless::SRes},
};
use bevy::prelude::*;
use bevy::render::{
	mesh::allocator::MeshSlabs,
	render_phase::{
		BinnedRenderPhaseType, DrawFunctions, InputUniformIndex, RenderCommand,
		RenderCommandResult, SetItemPipeline, TrackedRenderPass, ViewBinnedRenderPhases,
	},
	render_resource::{BindGroupEntries, PipelineCache},
	renderer::{RenderDevice, RenderQueue},
	view::{ExtractedView, ViewTarget, ViewUniformOffset, ViewUniforms},
};

use voxel_gpu::SlangShaderParam;
use crate::{
	MarchingShader,
	extract::ExtractedMarchingScene,
	model::ModelUniform,
	renderer_resource::{MarchingRendererResource, MarchingViewResources},
};

pub type DrawMarchingCommands = (SetItemPipeline, DrawMarching);

pub struct DrawMarching;

impl RenderCommand<Opaque3d> for DrawMarching {
	type Param = SRes<MarchingRendererResource>;
	type ViewQuery = (&'static ExtractedMarchingScene, &'static MarchingViewResources);
	type ItemQuery = ();

	fn render<'w>(
		item: &Opaque3d,
		(extracted, prepared): ROQueryItem<'w, '_, Self::ViewQuery>,
		_item: Option<ROQueryItem<'w, '_, Self::ItemQuery>>,
		shared: SystemParamItem<'w, '_, Self::Param>,
		pass: &mut TrackedRenderPass<'w>,
	) -> RenderCommandResult {
		if prepared.pipeline != Some(item.batch_set_key.pipeline) || extracted.items.is_empty() {
			return RenderCommandResult::Skip;
		}
		let shared = shared.into_inner();
		let Some(view_bind_group) = shared.view_bind_group.as_ref() else {
			return RenderCommandResult::Skip;
		};

		pass.set_bind_group(0, view_bind_group, &[prepared.view_uniform_offset]);
		pass.set_bind_group(1, &prepared.model_bind_group, &[]);
		for (batch, vertex_bind_group) in extracted.batches.iter().zip(&prepared.vertex_bind_groups) {
			pass.set_bind_group(2, vertex_bind_group, &[]);
			for index in batch.item_range.clone() {
				let item = &extracted.items[index];
				pass.draw(
					item.first_vertex..item.first_vertex + item.vertex_count,
					index as u32..index as u32 + 1,
				);
			}
		}
		RenderCommandResult::Success
	}
}

pub fn queue_marching_phase(
	cache: Res<PipelineCache>,
	mut shader: SlangShaderParam<MarchingShader>,
	mut shared: ResMut<MarchingRendererResource>,
	draw_functions: Res<DrawFunctions<Opaque3d>>,
	mut opaque_phases: ResMut<ViewBinnedRenderPhases<Opaque3d>>,
	views: Query<(Entity, &ExtractedView, &ViewTarget, &ExtractedMarchingScene)>,
) {
	let Some(shader) = shader.get().and_then(|shader| shader.bevy_shader().cloned()) else { return };
	let draw_function = draw_functions.read().id::<DrawMarchingCommands>();
	for (entity, view, target, extracted) in &views {
		if extracted.items.is_empty() { continue; }
		let Some(phase) = opaque_phases.get_mut(&view.retained_view_entity) else { continue };
		let pipeline = shared.pipeline(&cache, target.main_texture_format(), &shader);
		phase.add(
			Opaque3dBatchSetKey {
				pipeline,
				draw_function,
				material_bind_group_index: None,
				slabs: MeshSlabs::default(),
				lightmap_slab: None,
			},
			Opaque3dBinKey { asset_id: shader.id().untyped() },
			(entity, view.retained_view_entity.main_entity),
			InputUniformIndex::default(),
			BinnedRenderPhaseType::NonMesh,
		);
	}
}

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
	let vertex_layout = cache.get_bind_group_layout(&shared.vertex_layout);
	prepared.vertex_bind_groups = extracted.batches.iter().map(|batch| {
		device.create_bind_group(
			"marching_vertex_bind_group",
			&vertex_layout,
			&BindGroupEntries::single(wgpu::BufferBinding {
				buffer: &batch.vertex_buffer,
				offset: 0,
				size: None,
			}),
		)
	}).collect();
	prepared.pipeline = Some(pipeline);
}
