use std::collections::HashMap;

use bevy::asset::Handle;
use bevy::prelude::*;
use bevy::shader::Shader;
use bevy::render::render_resource::{
	BindGroup, BindGroupEntries, BindGroupLayoutDescriptor, BindGroupLayoutEntries,
	CachedRenderPipelineId, PipelineCache, ShaderStages, StorageBuffer, UniformBuffer,
	binding_types::{storage_buffer_read_only, storage_buffer_read_only_sized, uniform_buffer},
};
use bevy::render::renderer::{RenderDevice, RenderQueue};

use crate::{camera::CameraUniform, model::ModelUniform};

#[derive(Resource)]
pub struct MarchingRendererResource {
	pub camera_layout: BindGroupLayoutDescriptor,
	pub model_layout: BindGroupLayoutDescriptor,
	pub vertex_layout: BindGroupLayoutDescriptor,
	pub pipelines: HashMap<wgpu::TextureFormat, CachedRenderPipelineId>,
}

#[derive(Component)]
pub struct MarchingViewResources {
	pub pipeline: Option<CachedRenderPipelineId>,
	pub camera_buffer: UniformBuffer<CameraUniform>,
	pub model_buffer: StorageBuffer<Vec<ModelUniform>>,
	pub camera_bind_group: BindGroup,
	pub model_bind_group: BindGroup,
}

impl FromWorld for MarchingRendererResource {
	fn from_world(_world: &mut World) -> Self {
		Self {
			camera_layout: BindGroupLayoutDescriptor::new("marching_camera_layout", &BindGroupLayoutEntries::single(ShaderStages::VERTEX, uniform_buffer::<CameraUniform>(false))),
			model_layout: BindGroupLayoutDescriptor::new("marching_model_layout", &BindGroupLayoutEntries::single(ShaderStages::VERTEX, storage_buffer_read_only::<Vec<ModelUniform>>(false))),
			vertex_layout: BindGroupLayoutDescriptor::new("marching_vertex_layout", &BindGroupLayoutEntries::single(ShaderStages::VERTEX, storage_buffer_read_only_sized(false, None))),
			pipelines: HashMap::new(),
		}
	}
}

impl MarchingViewResources {
	pub fn new(device: &RenderDevice, queue: &RenderQueue, cache: &PipelineCache, shared: &MarchingRendererResource) -> Self {
		let mut camera_buffer = UniformBuffer::from(CameraUniform::default());
		camera_buffer.set_label(Some("marching_camera_buffer"));
		camera_buffer.write_buffer(device, queue);
		let camera_bind_group = device.create_bind_group("marching_camera_bind_group", &cache.get_bind_group_layout(&shared.camera_layout), &BindGroupEntries::single(camera_buffer.binding().unwrap()));
		let mut model_buffer = StorageBuffer::from(vec![ModelUniform::default()]);
		model_buffer.set_label(Some("marching_model_buffer"));
		model_buffer.write_buffer(device, queue);
		let model_bind_group = device.create_bind_group("marching_model_bind_group", &cache.get_bind_group_layout(&shared.model_layout), &BindGroupEntries::single(model_buffer.binding().unwrap()));
		Self { pipeline: None, camera_buffer, model_buffer, camera_bind_group, model_bind_group }
	}

	pub fn write_models(&mut self, mut models: Vec<ModelUniform>, device: &RenderDevice, queue: &RenderQueue, cache: &PipelineCache, shared: &MarchingRendererResource) {
		if models.is_empty() { models.push(ModelUniform::default()); }
		self.model_buffer.set(models);
		self.model_buffer.write_buffer(device, queue);
		self.model_bind_group = device.create_bind_group("marching_model_bind_group", &cache.get_bind_group_layout(&shared.model_layout), &BindGroupEntries::single(self.model_buffer.binding().unwrap()));
	}
}

impl MarchingRendererResource {
	pub fn pipeline(&mut self, cache: &PipelineCache, format: wgpu::TextureFormat, shader: &Handle<Shader>) -> CachedRenderPipelineId {
		if let Some(&pipeline) = self.pipelines.get(&format) { return pipeline; }
		let pipeline = cache.queue_render_pipeline(bevy::render::render_resource::RenderPipelineDescriptor {
			label: Some("marching_cubes_pipeline".into()),
			layout: vec![self.camera_layout.clone(), self.model_layout.clone(), self.vertex_layout.clone()],
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
