use std::collections::HashMap;

use bevy::ecs::component::Component;
use bevy::ecs::resource::Resource;
use bevy::ecs::world::FromWorld;
use bevy::render::render_resource::{
	BindGroup, BindGroupLayoutDescriptor, BindGroupLayoutEntries, CachedRenderPipelineId,
	PipelineCache, ShaderStages, StorageBuffer, binding_types::{
		storage_buffer_read_only, storage_buffer_read_only_sized, uniform_buffer,
	},
};
use bevy::render::renderer::{RenderDevice, RenderQueue};
use bevy::render::view::ViewUniform;

use crate::model::ModelUniform;

#[derive(Resource)]
pub struct VoxelRasterRendererResource {
	pub view_bind_group_layout: BindGroupLayoutDescriptor,
	pub model_bind_group_layout: BindGroupLayoutDescriptor,
	pub face_bind_group_layout: BindGroupLayoutDescriptor,
	pub view_bind_group: Option<BindGroup>,
	pub pipelines: HashMap<wgpu::TextureFormat, CachedRenderPipelineId>,
}

#[derive(Component)]
pub struct RasterViewResources {
	pub pipeline: Option<CachedRenderPipelineId>,
	pub view_uniform_offset: u32,
	pub model_buffer: StorageBuffer<Vec<ModelUniform>>,
	pub model_bind_group: BindGroup,
	pub face_bind_groups: Vec<BindGroup>,
}

impl FromWorld for VoxelRasterRendererResource {
	fn from_world(_world: &mut bevy::ecs::world::World) -> Self {
		let view_bind_group_layout = BindGroupLayoutDescriptor::new(
			"raster_view_bind_group_layout",
			&BindGroupLayoutEntries::single(
				ShaderStages::VERTEX,
				uniform_buffer::<ViewUniform>(true),
			),
		);
		let model_bind_group_layout = BindGroupLayoutDescriptor::new(
			"raster_model_bind_group_layout",
			&BindGroupLayoutEntries::single(
				ShaderStages::VERTEX,
				storage_buffer_read_only::<Vec<ModelUniform>>(false),
			),
		);
		let face_bind_group_layout = BindGroupLayoutDescriptor::new(
			"raster_face_bind_group_layout",
			&BindGroupLayoutEntries::sequential(
				ShaderStages::VERTEX,
				(
					storage_buffer_read_only_sized(false, None),
					storage_buffer_read_only_sized(false, None),
				),
			),
		);
		Self {
			view_bind_group_layout,
			model_bind_group_layout,
			face_bind_group_layout,
			view_bind_group: None,
			pipelines: HashMap::new(),
		}
	}
}

impl RasterViewResources {
	pub fn new(
		render_device: &RenderDevice,
		render_queue: &RenderQueue,
		pipeline_cache: &PipelineCache,
		shared: &VoxelRasterRendererResource,
	) -> Self {
		let mut model_buffer = StorageBuffer::from(vec![ModelUniform::default()]);
		model_buffer.set_label(Some("raster_view_model_storage"));
		model_buffer.write_buffer(render_device, render_queue);
		let model_bind_group = render_device.create_bind_group(
			"raster_model_bind_group",
			&pipeline_cache.get_bind_group_layout(&shared.model_bind_group_layout),
			&bevy::render::render_resource::BindGroupEntries::single(
				model_buffer.binding().expect("model storage buffer must be initialized"),
			),
		);

		Self {
			pipeline: None,
			view_uniform_offset: 0,
			model_buffer,
			model_bind_group,
			face_bind_groups: Vec::new(),
		}
	}

	pub fn write_models(
		&mut self,
		mut models: Vec<ModelUniform>,
		render_device: &RenderDevice,
		render_queue: &RenderQueue,
		pipeline_cache: &PipelineCache,
		shared: &VoxelRasterRendererResource,
	) {
		if models.is_empty() { models.push(ModelUniform::default()); }
		self.model_buffer.set(models);
		self.model_buffer.write_buffer(render_device, render_queue);
		self.model_bind_group = render_device.create_bind_group(
			"raster_model_bind_group",
			&pipeline_cache.get_bind_group_layout(&shared.model_bind_group_layout),
			&bevy::render::render_resource::BindGroupEntries::single(
				self.model_buffer.binding().expect("model storage buffer must be initialized"),
			),
		);
	}
}
