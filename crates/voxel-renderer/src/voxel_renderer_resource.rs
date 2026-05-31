use bevy::ecs::resource::Resource;
use bevy::ecs::world::FromWorld;
use bevy::render::renderer::RenderDevice;

use crate::voxel_renderer::VoxelRenderer;

#[derive(Resource)]
pub struct VoxelRendererResource {
	pub voxel_renderer: Option<VoxelRenderer>,
	pub size: (u32, u32),
	pub format: Option<wgpu::TextureFormat>,
	pub camera_buffer: wgpu::Buffer,
	pub render_settings_buffer: wgpu::Buffer,
	pub camera_bind_group: wgpu::BindGroup,
	pub camera_bind_group_layout: wgpu::BindGroupLayout,
}

impl FromWorld for VoxelRendererResource {
	fn from_world(world: &mut bevy::ecs::world::World) -> Self {
		let render_device = world.resource::<RenderDevice>();
		let device = render_device.wgpu_device();

		let camera_buffer = device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("voxel_camera_uniform"),
			size: std::mem::size_of::<crate::camera::CameraUniform>() as u64,
			usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		});
		let render_settings_buffer = device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("voxel_render_settings_uniform"),
			size: std::mem::size_of::<crate::graphics_settings::RenderSettingsUniform>() as u64,
			usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		});

		let camera_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[
				wgpu::BindGroupLayoutEntry {
					binding: 0,
					visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::COMPUTE,
					ty: wgpu::BindingType::Buffer {
						ty: wgpu::BufferBindingType::Uniform,
						has_dynamic_offset: false,
						min_binding_size: None,
					},
					count: None,
				},
				wgpu::BindGroupLayoutEntry {
					binding: 1,
					visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::COMPUTE,
					ty: wgpu::BindingType::Buffer {
						ty: wgpu::BufferBindingType::Uniform,
						has_dynamic_offset: false,
						min_binding_size: None,
					},
					count: None,
				},
			],
			label: Some("voxel_camera_bind_group_layout"),
		});
		let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &camera_bind_group_layout,
			entries: &[
				wgpu::BindGroupEntry { binding: 0, resource: camera_buffer.as_entire_binding() },
				wgpu::BindGroupEntry { binding: 1, resource: render_settings_buffer.as_entire_binding() },
			],
			label: Some("voxel_camera_bind_group"),
		});

		Self {
			voxel_renderer: None,
			size: (0, 0),
			format: None,
			camera_buffer,
			render_settings_buffer,
			camera_bind_group,
			camera_bind_group_layout,
		}
	}
}

impl VoxelRendererResource {
	pub fn ensure(&mut self, device: &wgpu::Device, width: u32, height: u32, format: wgpu::TextureFormat) {
		if width == 0 || height == 0 { return; }

		let need_rebuild = self.voxel_renderer.is_none() || self.format != Some(format);
		if need_rebuild {
			match VoxelRenderer::new(device, width, height, format, &self.camera_bind_group_layout) {
				Ok(vr) => {
					self.voxel_renderer = Some(vr);
					self.size = (width, height);
					self.format = Some(format);
				},
				Err(e) => {
					log::error!("Failed to build VoxelRenderer: {e}");
				}
			}
			return;
		}

		if self.size != (width, height) {
			if let Some(vr) = self.voxel_renderer.as_mut() {
				vr.resize(device, width, height);
				self.size = (width, height);
			}
		}
	}
}
