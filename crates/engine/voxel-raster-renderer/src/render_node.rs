use bevy::ecs::{
	query::ROQueryItem,
	system::{Commands, Query, Res, ResMut, SystemParamItem, lifetimeless::SRes},
};
use bevy::math::Mat4;
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
use bevy::core_pipeline::core_3d::{Opaque3d, Opaque3dBatchSetKey, Opaque3dBinKey};

use crate::extract::ExtractedRasterScene;
use crate::model::ModelUniform;
use crate::voxel_raster_renderer_resource::{RasterViewResources, VoxelRasterRendererResource};
use crate::VoxelRasterShader;
use voxel_gpu::{SlangShaderParam, packed_buffer_group::PackedBufferGroupBuffer};

pub type DrawVoxelRasterCommands = (SetItemPipeline, DrawVoxelRaster);

pub struct DrawVoxelRaster;

impl RenderCommand<Opaque3d> for DrawVoxelRaster {
	type Param = SRes<VoxelRasterRendererResource>;
	type ViewQuery = (&'static ExtractedRasterScene, &'static RasterViewResources);
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
		for (batch, data_bind_group) in extracted.batches.iter().zip(&prepared.data_bind_groups) {
			pass.set_bind_group(2, data_bind_group, &[]);
			for index in batch.item_range.clone() {
				let item = &extracted.items[index];
				let instance = index as u32;
				pass.draw(
					item.face_offset * 6..(item.face_offset + item.face_count) * 6,
					instance..instance + 1,
				);
			}
		}
		RenderCommandResult::Success
	}
}

pub fn queue_raster_phase(
	pipeline_cache: Res<PipelineCache>,
	mut shader: SlangShaderParam<VoxelRasterShader>,
	mut shared: ResMut<VoxelRasterRendererResource>,
	draw_functions: Res<DrawFunctions<Opaque3d>>,
	mut opaque_phases: ResMut<ViewBinnedRenderPhases<Opaque3d>>,
	views: Query<(bevy::ecs::entity::Entity, &ExtractedView, &ViewTarget, &ExtractedRasterScene)>,
) {
	let Some(shader) = shader.get().and_then(|shader| shader.bevy_shader().cloned()) else { return };
	let draw_function = draw_functions.read().id::<DrawVoxelRasterCommands>();
	for (entity, view, target, extracted) in &views {
		if extracted.items.is_empty() { continue; }
		let Some(phase) = opaque_phases.get_mut(&view.retained_view_entity) else { continue };
		let pipeline = shared.pipeline(&pipeline_cache, target.main_texture_format(), &shader);
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
		&ViewTarget,
		&ViewUniformOffset,
		&ExtractedRasterScene,
		Option<&mut RasterViewResources>,
	)>,
) {
	let Some(view_binding) = view_uniforms.uniforms.binding() else {
		raster_resource.view_bind_group = None;
		for (_, _, _, _, prepared) in &mut views {
			if let Some(mut prepared) = prepared { prepared.pipeline = None; }
		}
		return;
	};
	raster_resource.view_bind_group = Some(render_device.create_bind_group(
		"raster_view_bind_group",
		&pipeline_cache.get_bind_group_layout(&raster_resource.view_bind_group_layout),
		&BindGroupEntries::single(view_binding),
	));

	let shader = shader.get().and_then(|shader| shader.bevy_shader().cloned());
	for (entity, view_target, view_offset, extracted, prepared) in &mut views {
		let Some(shader) = shader.as_ref() else {
			if let Some(mut prepared) = prepared { prepared.pipeline = None; }
			continue;
		};
		let pipeline = raster_resource.pipeline(&pipeline_cache, view_target.main_texture_format(), shader);
		if let Some(mut prepared) = prepared {
			prepare_view(&mut prepared, view_offset.offset, extracted, pipeline, &render_device, &render_queue, &pipeline_cache, &raster_resource);
		} else {
			let mut prepared = RasterViewResources::new(&render_device, &render_queue, &pipeline_cache, &raster_resource);
			prepare_view(&mut prepared, view_offset.offset, extracted, pipeline, &render_device, &render_queue, &pipeline_cache, &raster_resource);
			commands.entity(entity).insert(prepared);
		}
	}
}

fn prepare_view(
	prepared: &mut RasterViewResources,
	view_uniform_offset: u32,
	extracted: &ExtractedRasterScene,
	pipeline: bevy::render::render_resource::CachedRenderPipelineId,
	render_device: &RenderDevice,
	render_queue: &RenderQueue,
	pipeline_cache: &PipelineCache,
	shared: &VoxelRasterRendererResource,
) {
	prepared.view_uniform_offset = view_uniform_offset;
	let models = extracted.items.iter()
		.map(|item| {
			let model = Mat4::from_scale_rotation_translation(item.transform.scale, item.transform.rotation, item.transform.translation);
			ModelUniform::new(model, item.voxel_type, item.voxel_data_offset)
		})
		.collect();
	prepared.write_models(models, render_device, render_queue, pipeline_cache, shared);
	prepared.data_bind_groups = make_data_bind_groups(
		&extracted.batches.iter().map(|batch| batch.face_buffer.clone()).collect::<Vec<_>>(),
		&extracted.batches.iter().map(|batch| batch.voxel_data_buffer.clone()).collect::<Vec<_>>(),
		render_device,
		pipeline_cache,
		shared,
	);
	prepared.pipeline = Some(pipeline);
}

fn make_data_bind_groups(
	face_buffers: &[PackedBufferGroupBuffer],
	voxel_data_buffers: &[PackedBufferGroupBuffer],
	render_device: &RenderDevice,
	pipeline_cache: &PipelineCache,
	shared: &VoxelRasterRendererResource,
) -> Vec<bevy::render::render_resource::BindGroup> {
	let layout = pipeline_cache.get_bind_group_layout(&shared.data_bind_group_layout);
	face_buffers.iter().zip(voxel_data_buffers).map(|(face_buffer, voxel_data_buffer)| {
		render_device.create_bind_group(
			"raster_data_bind_group",
			&layout,
			&BindGroupEntries::sequential((
				wgpu::BufferBinding { buffer: face_buffer, offset: 0, size: None },
				wgpu::BufferBinding { buffer: voxel_data_buffer, offset: 0, size: None },
			)),
		)
	}).collect()
}
