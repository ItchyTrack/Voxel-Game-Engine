use bevy::ecs::component::Component;
use bevy::ecs::resource::Resource;
use bevy::ecs::world::FromWorld;
use bevy::render::render_resource::{DynamicUniformBuffer, ShaderType, UniformBuffer};
use bevy::render::renderer::{RenderDevice, RenderQueue, WgpuWrapper};

use crate::camera::CameraUniform;
use crate::model::ModelUniform;
use crate::voxel_raster_renderer::VoxelRasterRenderer;
use voxel_gpu::LoadedSlangShader;

type GpuBindGroup = WgpuWrapper<wgpu::BindGroup>;
type GpuBindGroupLayout = WgpuWrapper<wgpu::BindGroupLayout>;

#[derive(Resource)]
pub struct VoxelRasterRendererResource {
	pub camera_bind_group_layout: GpuBindGroupLayout,
	pub model_bind_group_layout: GpuBindGroupLayout,
}

#[derive(Component)]
pub struct RasterViewResources {
	pub ready: bool,
	pub renderer: Option<VoxelRasterRenderer>,
	pub format: Option<wgpu::TextureFormat>,
	pub camera_buffer: UniformBuffer<CameraUniform>,
	pub model_buffer: DynamicUniformBuffer<ModelUniform>,
	pub model_offsets: Vec<u32>,
	pub camera_bind_group: GpuBindGroup,
	pub model_bind_group: GpuBindGroup,
}

impl FromWorld for VoxelRasterRendererResource {
	fn from_world(world: &mut bevy::ecs::world::World) -> Self {
		let render_device = world.resource::<RenderDevice>();
		let device = render_device.wgpu_device();
		let camera_bind_group_layout = WgpuWrapper::new(device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[wgpu::BindGroupLayoutEntry {
				binding: 0,
				visibility: wgpu::ShaderStages::VERTEX,
				ty: wgpu::BindingType::Buffer {
					ty: wgpu::BufferBindingType::Uniform,
					has_dynamic_offset: false,
					min_binding_size: Some(CameraUniform::min_size()),
				},
				count: None,
			}],
			label: Some("Raster Camera Bind Group Layout"),
		}));
		let model_bind_group_layout = ModelUniform::get_dynamic_offset_bind_group_layout(device, 0);
		Self { camera_bind_group_layout, model_bind_group_layout }
	}
}

impl RasterViewResources {
	pub fn new(
		device: &wgpu::Device,
		render_device: &RenderDevice,
		render_queue: &RenderQueue,
		shared: &VoxelRasterRendererResource,
	) -> Self {
		let mut camera_buffer = UniformBuffer::from(CameraUniform::default());
		camera_buffer.set_label(Some("raster_view_camera_buffer"));
		camera_buffer.write_buffer(render_device, render_queue);
		let camera_bind_group = WgpuWrapper::new(device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &shared.camera_bind_group_layout,
			entries: &[wgpu::BindGroupEntry {
				binding: 0,
				resource: camera_buffer.binding().expect("camera uniform buffer must be initialized"),
			}],
			label: Some("raster_view_camera_bind_group"),
		}));

		let mut model_buffer = DynamicUniformBuffer::default();
		model_buffer.set_label(Some("raster_view_model_uniform"));
		model_buffer.push(&ModelUniform::default());
		model_buffer.write_buffer(render_device, render_queue);
		let model_bind_group = ModelUniform::get_bind_group(
			device,
			&shared.model_bind_group_layout,
			&model_buffer,
			0,
		);

		Self {
			ready: false,
			renderer: None,
			format: None,
			camera_buffer,
			model_buffer,
			model_offsets: Vec::new(),
			camera_bind_group,
			model_bind_group,
		}
	}

	pub fn ensure(
		&mut self,
		device: &wgpu::Device,
		format: wgpu::TextureFormat,
		camera_bind_group_layout: &GpuBindGroupLayout,
		model_bind_group_layout: &GpuBindGroupLayout,
		shader: &LoadedSlangShader<'_>,
	) {
		let need_rebuild = self.renderer.is_none()
			|| self.format != Some(format)
			|| shader.changed();
		if !need_rebuild { return; }
		let shader_source = match crate::shader_sources::from_compiled(&shader.shader().shaders) {
			Ok(shader_source) => shader_source,
			Err(error) => {
				log::error!("Failed to read compiled voxel raster shader asset: {error}");
				return;
			}
		};

		match VoxelRasterRenderer::new(
			device,
			format,
			camera_bind_group_layout,
			model_bind_group_layout,
			&shader_source,
		) {
			Ok(renderer) => {
				self.renderer = Some(renderer);
				self.format = Some(format);
			}
			Err(error) => log::error!("Failed to build VoxelRasterRenderer: {error}"),
		}
	}
}
