use bevy::ecs::resource::Resource;
use bevy::ecs::world::FromWorld;
use bevy::render::renderer::{RenderDevice, WgpuWrapper};

type GpuBuffer = WgpuWrapper<wgpu::Buffer>;
type GpuBindGroup = WgpuWrapper<wgpu::BindGroup>;
type GpuBindGroupLayout = WgpuWrapper<wgpu::BindGroupLayout>;

#[cfg(all(feature = "shader_hot_reload", not(target_arch = "wasm32")))]
use crate::shader_hot_reload::VoxelShaderHotReload;
use crate::shader_sources::VoxelShaderSources;
use crate::voxel_renderer::VoxelRenderer;

#[derive(Resource)]
pub struct VoxelRendererResource {
	pub voxel_renderer: Option<VoxelRenderer>,
	pub size: (u32, u32),
	pub format: Option<wgpu::TextureFormat>,
	pub camera_buffer: GpuBuffer,
	pub render_settings_buffer: GpuBuffer,
	pub camera_bind_group: GpuBindGroup,
	pub camera_bind_group_layout: GpuBindGroupLayout,
	shader_sources: VoxelShaderSources,
	#[cfg(all(feature = "shader_hot_reload", not(target_arch = "wasm32")))]
	hot_reload: Option<VoxelShaderHotReload>,
}

impl FromWorld for VoxelRendererResource {
	fn from_world(world: &mut bevy::ecs::world::World) -> Self {
		let render_device = world.resource::<RenderDevice>();
		let device = render_device.wgpu_device();

		let camera_buffer = WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("voxel_camera_uniform"),
			size: std::mem::size_of::<crate::camera::CameraUniform>() as u64,
			usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		}));
		let render_settings_buffer = WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("voxel_render_settings_uniform"),
			size: std::mem::size_of::<crate::graphics_settings::RenderSettingsUniform>() as u64,
			usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		}));

		let camera_bind_group_layout = WgpuWrapper::new(device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
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
		}));
		let camera_bind_group = WgpuWrapper::new(device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &camera_bind_group_layout,
			entries: &[
				wgpu::BindGroupEntry { binding: 0, resource: camera_buffer.as_entire_binding() },
				wgpu::BindGroupEntry { binding: 1, resource: render_settings_buffer.as_entire_binding() },
			],
			label: Some("voxel_camera_bind_group"),
		}));

		#[cfg(all(feature = "shader_hot_reload", not(target_arch = "wasm32")))]
		let shader_sources = VoxelShaderSources::load_from_disk()
			.unwrap_or_else(|_| VoxelShaderSources::embedded());
		#[cfg(not(all(feature = "shader_hot_reload", not(target_arch = "wasm32"))))]
		let shader_sources = VoxelShaderSources::embedded();

		Self {
			voxel_renderer: None,
			size: (0, 0),
			format: None,
			camera_buffer,
			render_settings_buffer,
			camera_bind_group,
			camera_bind_group_layout,
			shader_sources,
			#[cfg(all(feature = "shader_hot_reload", not(target_arch = "wasm32")))]
			hot_reload: match VoxelShaderHotReload::new() {
				Ok(hot_reload) => Some(hot_reload),
				Err(error) => {
					log::error!("Failed to initialize voxel shader hot reload: {error}");
					None
				}
			},
		}
	}
}

impl VoxelRendererResource {
	pub fn ensure(&mut self, device: &wgpu::Device, width: u32, height: u32, format: wgpu::TextureFormat) {
		if width == 0 || height == 0 { return; }

		let shaders_changed = self.refresh_shader_sources_if_needed();
		let need_rebuild = self.voxel_renderer.is_none()
			|| self.size != (width, height)
			|| self.format != Some(format)
			|| shaders_changed;
		if need_rebuild {
			self.rebuild_renderer(device, width, height, format);
		}
	}

	fn rebuild_renderer(&mut self, device: &wgpu::Device, width: u32, height: u32, format: wgpu::TextureFormat) {
		match VoxelRenderer::new(device, width, height, format, &self.camera_bind_group_layout, &self.shader_sources) {
			Ok(voxel_renderer) => {
				self.voxel_renderer = Some(voxel_renderer);
				self.size = (width, height);
				self.format = Some(format);
			}
			Err(error) => {
				log::error!("Failed to build VoxelRenderer: {error}");
			}
		}
	}

	#[cfg(all(feature = "shader_hot_reload", not(target_arch = "wasm32")))]
	fn refresh_shader_sources_if_needed(&mut self) -> bool {
		let Some(hot_reload) = self.hot_reload.as_ref() else { return false; };
		if !hot_reload.take_dirty() { return false; }

		match VoxelShaderSources::load_from_disk() {
			Ok(shader_sources) => {
				self.shader_sources = shader_sources;
				log::info!("Reloaded voxel shaders from disk");
				true
			}
			Err(error) => {
				log::error!("Failed to hot reload voxel shaders: {error}");
				false
			}
		}
	}

	#[cfg(not(all(feature = "shader_hot_reload", not(target_arch = "wasm32"))))]
	fn refresh_shader_sources_if_needed(&mut self) -> bool {
		false
	}
}
