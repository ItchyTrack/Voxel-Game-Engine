use bevy::ecs::component::Component;
use bevy::ecs::resource::Resource;
use bevy::ecs::world::FromWorld;
use bevy::render::render_resource::{ShaderType, UniformBuffer};
use bevy::render::renderer::{RenderDevice, RenderQueue, WgpuWrapper};

use crate::camera::CameraUniform;
use crate::direction_feedback::DirectionFeedback;
use crate::graphics_settings::RenderSettingsUniform;
use crate::shader_sources::VoxelShaderSources;
use crate::voxel_renderer::VoxelRenderer;
use voxel_gpu::LoadedSlangShader;

type GpuBindGroup = WgpuWrapper<wgpu::BindGroup>;
type GpuBindGroupLayout = WgpuWrapper<wgpu::BindGroupLayout>;

#[derive(Resource)]
pub struct VoxelRendererResource {
	pub render_settings_buffer: UniformBuffer<RenderSettingsUniform>,
	pub camera_bind_group_layout: GpuBindGroupLayout,
}

#[derive(Component)]
#[require(DirectionFeedback)]
pub struct VoxelViewResources {
	pub ready: bool,
	pub voxel_renderer: Option<VoxelRenderer>,
	pub size: (u32, u32),
	pub format: Option<wgpu::TextureFormat>,
	pub camera_buffer: UniformBuffer<CameraUniform>,
	pub camera_bind_group: GpuBindGroup,
}

impl FromWorld for VoxelRendererResource {
	fn from_world(world: &mut bevy::ecs::world::World) -> Self {
		let render_device = world.resource::<RenderDevice>();
		let render_queue = world.resource::<RenderQueue>();
		let device = render_device.wgpu_device();

		let mut render_settings_buffer = UniformBuffer::from(RenderSettingsUniform::default());
		render_settings_buffer.set_label(Some("voxel_render_settings_uniform"));
		render_settings_buffer.write_buffer(render_device, render_queue);

		let camera_bind_group_layout = WgpuWrapper::new(device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[
				wgpu::BindGroupLayoutEntry {
					binding: 0,
					visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::COMPUTE,
					ty: wgpu::BindingType::Buffer {
						ty: wgpu::BufferBindingType::Uniform,
						has_dynamic_offset: false,
						min_binding_size: Some(CameraUniform::min_size()),
					},
					count: None,
				},
				wgpu::BindGroupLayoutEntry {
					binding: 1,
					visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::COMPUTE,
					ty: wgpu::BindingType::Buffer {
						ty: wgpu::BufferBindingType::Uniform,
						has_dynamic_offset: false,
						min_binding_size: Some(RenderSettingsUniform::min_size()),
					},
					count: None,
				},
			],
			label: Some("voxel_camera_bind_group_layout"),
		}));

		Self { render_settings_buffer, camera_bind_group_layout }
	}
}

impl VoxelViewResources {
	pub fn new(
		device: &wgpu::Device,
		render_device: &RenderDevice,
		render_queue: &RenderQueue,
		shared: &VoxelRendererResource,
	) -> Self {
		let mut camera_buffer = UniformBuffer::from(CameraUniform::default());
		camera_buffer.set_label(Some("voxel_view_camera_uniform"));
		camera_buffer.write_buffer(render_device, render_queue);
		let camera_bind_group = WgpuWrapper::new(device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &shared.camera_bind_group_layout,
			entries: &[
				wgpu::BindGroupEntry {
					binding: 0,
					resource: camera_buffer.binding().expect("camera uniform buffer must be initialized"),
				},
				wgpu::BindGroupEntry {
					binding: 1,
					resource: shared.render_settings_buffer.binding().expect("render settings uniform must be initialized"),
				},
			],
			label: Some("voxel_view_camera_bind_group"),
		}));
		Self {
			ready: false,
			voxel_renderer: None,
			size: (0, 0),
			format: None,
			camera_buffer,
			camera_bind_group,
		}
	}

	pub fn ensure(
		&mut self,
		device: &wgpu::Device,
		width: u32,
		height: u32,
		format: wgpu::TextureFormat,
		camera_bind_group_layout: &GpuBindGroupLayout,
		shader: &LoadedSlangShader<'_>,
	) {
		if width == 0 || height == 0 { return; }
		let need_rebuild = self.voxel_renderer.is_none()
			|| self.size != (width, height)
			|| self.format != Some(format)
			|| shader.changed();
		if !need_rebuild { return; }
		let shader_sources = match VoxelShaderSources::from_compiled(&shader.shader().shaders) {
			Ok(shader_sources) => shader_sources,
			Err(error) => {
				log::error!("Failed to read compiled voxel ray shader asset: {error}");
				return;
			}
		};

		match VoxelRenderer::new(device, width, height, format, camera_bind_group_layout, &shader_sources) {
			Ok(voxel_renderer) => {
				self.voxel_renderer = Some(voxel_renderer);
				self.size = (width, height);
				self.format = Some(format);
			}
			Err(error) => log::error!("Failed to build VoxelRenderer: {error}"),
		}
	}
}
