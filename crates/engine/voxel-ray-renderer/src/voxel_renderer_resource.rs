use bevy::ecs::component::Component;
use bevy::ecs::resource::Resource;
use bevy::ecs::world::FromWorld;
use bevy::render::render_resource::{ShaderType, UniformBuffer};
use bevy::render::renderer::{RenderDevice, RenderQueue, WgpuWrapper};
use bevy::render::view::ViewUniform;

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
	pub view_bind_group_layout: GpuBindGroupLayout,
	pub view_bind_group: Option<GpuBindGroup>,
}

#[derive(Component)]
#[require(DirectionFeedback)]
pub struct VoxelViewResources {
	pub ready: bool,
	pub voxel_renderer: Option<VoxelRenderer>,
	pub size: (u32, u32),
	pub format: Option<wgpu::TextureFormat>,
	pub view_uniform_offset: u32,
}

impl FromWorld for VoxelRendererResource {
	fn from_world(world: &mut bevy::ecs::world::World) -> Self {
		let render_device = world.resource::<RenderDevice>();
		let render_queue = world.resource::<RenderQueue>();
		let device = render_device.wgpu_device();

		let mut render_settings_buffer = UniformBuffer::from(RenderSettingsUniform::default());
		render_settings_buffer.set_label(Some("voxel_render_settings_uniform"));
		render_settings_buffer.write_buffer(render_device, render_queue);

		let view_bind_group_layout = WgpuWrapper::new(device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[
				wgpu::BindGroupLayoutEntry {
					binding: 0,
					visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::COMPUTE,
					ty: wgpu::BindingType::Buffer {
						ty: wgpu::BufferBindingType::Uniform,
						has_dynamic_offset: true,
						min_binding_size: Some(ViewUniform::min_size()),
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
			label: Some("voxel_view_bind_group_layout"),
		}));

		Self { render_settings_buffer, view_bind_group_layout, view_bind_group: None }
	}
}

impl VoxelViewResources {
	pub fn new() -> Self {
		Self {
			ready: false,
			voxel_renderer: None,
			size: (0, 0),
			format: None,
			view_uniform_offset: 0,
		}
	}

	pub fn ensure(
		&mut self,
		device: &wgpu::Device,
		width: u32,
		height: u32,
		format: wgpu::TextureFormat,
		view_bind_group_layout: &GpuBindGroupLayout,
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

		match VoxelRenderer::new(device, width, height, format, view_bind_group_layout, &shader_sources) {
			Ok(voxel_renderer) => {
				self.voxel_renderer = Some(voxel_renderer);
				self.size = (width, height);
				self.format = Some(format);
			}
			Err(error) => log::error!("Failed to build VoxelRenderer: {error}"),
		}
	}
}
