use std::collections::HashMap;

use bevy::asset::Handle;
use bevy::prelude::*;
use bevy::shader::Shader;
use bevy::render::render_resource::{
	BindGroup, BindGroupEntries, BindGroupLayoutDescriptor, BindGroupLayoutEntries,
	CachedRenderPipelineId, PipelineCache, ShaderStages, StorageBuffer,
	binding_types::{storage_buffer_read_only, storage_buffer_read_only_sized, uniform_buffer},
};
use bevy::render::renderer::{RenderDevice, RenderQueue};
use bevy::render::view::ViewUniform;

use crate::model::ModelUniform;

#[derive(Resource)]
pub struct MarchingRendererResource {
	pub view_layout: BindGroupLayoutDescriptor,
	pub model_layout: BindGroupLayoutDescriptor,
	pub vertex_layout: BindGroupLayoutDescriptor,
	pub view_bind_group: Option<BindGroup>,
	pub pipelines: HashMap<wgpu::TextureFormat, CachedRenderPipelineId>,
}

#[derive(Component)]
pub struct MarchingViewResources {
	pub pipeline: Option<CachedRenderPipelineId>,
	pub view_uniform_offset: u32,
	pub model_buffer: StorageBuffer<Vec<ModelUniform>>,
	pub model_bind_group: BindGroup,
}

impl FromWorld for MarchingRendererResource {
	fn from_world(_world: &mut World) -> Self {
		Self {
			view_layout: BindGroupLayoutDescriptor::new("marching_view_layout", &BindGroupLayoutEntries::single(ShaderStages::VERTEX, uniform_buffer::<ViewUniform>(true))),
			model_layout: BindGroupLayoutDescriptor::new("marching_model_layout", &BindGroupLayoutEntries::single(ShaderStages::VERTEX, storage_buffer_read_only::<Vec<ModelUniform>>(false))),
			vertex_layout: BindGroupLayoutDescriptor::new("marching_vertex_layout", &BindGroupLayoutEntries::single(ShaderStages::VERTEX, storage_buffer_read_only_sized(false, None))),
			view_bind_group: None,
			pipelines: HashMap::new(),
		}
	}
}

impl MarchingViewResources {
	pub fn new(device: &RenderDevice, queue: &RenderQueue, cache: &PipelineCache, shared: &MarchingRendererResource) -> Self {
		let mut model_buffer = StorageBuffer::from(vec![ModelUniform::default()]);
		model_buffer.set_label(Some("marching_model_buffer"));
		model_buffer.write_buffer(device, queue);
		let model_bind_group = device.create_bind_group("marching_model_bind_group", &cache.get_bind_group_layout(&shared.model_layout), &BindGroupEntries::single(model_buffer.binding().expect("model storage buffer must be initialized")));
		Self { pipeline: None, view_uniform_offset: 0, model_buffer, model_bind_group }
	}

	pub fn write_models(&mut self, mut models: Vec<ModelUniform>, device: &RenderDevice, queue: &RenderQueue, cache: &PipelineCache, shared: &MarchingRendererResource) {
		if models.is_empty() { models.push(ModelUniform::default()); }
		self.model_buffer.set(models);
		self.model_buffer.write_buffer(device, queue);
		self.model_bind_group = device.create_bind_group("marching_model_bind_group", &cache.get_bind_group_layout(&shared.model_layout), &BindGroupEntries::single(self.model_buffer.binding().expect("model storage buffer must be initialized")));
	}
}

impl MarchingRendererResource {
	pub fn pipeline(&mut self, cache: &PipelineCache, format: wgpu::TextureFormat, shader: &Handle<Shader>) -> CachedRenderPipelineId {
		if let Some(&pipeline) = self.pipelines.get(&format) { return pipeline; }
		let pipeline = cache.queue_render_pipeline(bevy::render::render_resource::RenderPipelineDescriptor {
			label: Some("marching_cubes_pipeline".into()),
			layout: vec![self.view_layout.clone(), self.model_layout.clone(), self.vertex_layout.clone()],
			vertex: bevy::render::render_resource::VertexState { shader: shader.clone(), entry_point: Some("vs_main".into()), ..Default::default() },
			fragment: Some(bevy::render::render_resource::FragmentState {
				shader: shader.clone(),
				entry_point: Some("fs_main".into()),
				targets: vec![Some(wgpu::ColorTargetState { format, blend: Some(wgpu::BlendState::REPLACE), write_mask: wgpu::ColorWrites::ALL })],
				..Default::default()
			}),
			primitive: wgpu::PrimitiveState { topology: wgpu::PrimitiveTopology::TriangleList, cull_mode: Some(wgpu::Face::Back), ..Default::default() },
			depth_stencil: Some(wgpu::DepthStencilState {
				format: wgpu::TextureFormat::Depth32Float,
				depth_write_enabled: Some(true),
				depth_compare: Some(wgpu::CompareFunction::GreaterEqual),
				stencil: Default::default(),
				bias: Default::default(),
			}),
			multisample: Default::default(),
			..Default::default()
		});
		self.pipelines.insert(format, pipeline);
		pipeline
	}
}
