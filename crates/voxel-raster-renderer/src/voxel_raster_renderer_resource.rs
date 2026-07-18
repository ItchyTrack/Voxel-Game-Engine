use bevy::ecs::resource::Resource;
use bevy::ecs::world::FromWorld;
use bevy::render::renderer::{RenderDevice, WgpuWrapper};

use crate::camera::CameraUniform;
use crate::model::ModelUniform;
use crate::voxel_raster_renderer::VoxelRasterRenderer;
#[cfg(all(feature = "shader_hot_reload", not(target_arch = "wasm32")))]
use crate::shader_hot_reload::RasterShaderHotReload;

type GpuBuffer = WgpuWrapper<wgpu::Buffer>;
type GpuBindGroup = WgpuWrapper<wgpu::BindGroup>;
type GpuBindGroupLayout = WgpuWrapper<wgpu::BindGroupLayout>;

#[derive(Resource)]
pub struct VoxelRasterRendererResource {
	pub renderer: Option<VoxelRasterRenderer>,
	pub size: (u32, u32),
	pub format: Option<wgpu::TextureFormat>,
	pub camera_buffer: GpuBuffer,
	pub model_buffer: GpuBuffer,
	pub camera_bind_group: GpuBindGroup,
	pub model_bind_group: GpuBindGroup,
	pub face_bind_group: Option<GpuBindGroup>,
	pub camera_bind_group_layout: GpuBindGroupLayout,
	pub model_bind_group_layout: GpuBindGroupLayout,
	pub model_stride: u64,
	model_buffer_size: u64,
	shader_source: String,
	#[cfg(all(feature = "shader_hot_reload", not(target_arch = "wasm32")))]
	hot_reload: Option<RasterShaderHotReload>,
}

impl FromWorld for VoxelRasterRendererResource {
	fn from_world(world: &mut bevy::ecs::world::World) -> Self {
		let render_device = world.resource::<RenderDevice>();
		let device = render_device.wgpu_device();

		let (camera_buffer, camera_bind_group, camera_bind_group_layout) = CameraUniform::get_buffer(device, 0);
		let model_bind_group_layout = ModelUniform::get_dynamic_offset_bind_group_layout(device, 0);
		let model_stride = (std::mem::size_of::<ModelUniform>() as u64)
			.next_multiple_of(device.limits().min_uniform_buffer_offset_alignment as u64);
		let model_buffer_size = model_stride.max(std::mem::size_of::<ModelUniform>() as u64);
		let model_buffer = WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("raster_model_uniform"),
			size: model_buffer_size,
			usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		}));
		let model_bind_group = ModelUniform::get_bind_group(device, &model_bind_group_layout, &model_buffer, 0);

		#[cfg(all(feature = "shader_hot_reload", not(target_arch = "wasm32")))]
		let shader_source = crate::shader_sources::load_from_disk()
			.or_else(|_| crate::shader_sources::embedded())
			.expect("embedded voxel raster shader must compile");
		#[cfg(not(all(feature = "shader_hot_reload", not(target_arch = "wasm32"))))]
		let shader_source = crate::shader_sources::embedded()
			.expect("embedded voxel raster shader must compile");

		Self {
			renderer: None,
			size: (0, 0),
			format: None,
			camera_buffer,
			model_buffer,
			camera_bind_group,
			model_bind_group,
			face_bind_group: None,
			camera_bind_group_layout,
			model_bind_group_layout,
			model_stride,
			model_buffer_size,
			shader_source,
			#[cfg(all(feature = "shader_hot_reload", not(target_arch = "wasm32")))]
			hot_reload: RasterShaderHotReload::new().map_err(|error| {
				log::error!("Failed to initialize raster shader hot reload: {error}");
				error
			}).ok(),
		}
	}
}

impl VoxelRasterRendererResource {
	pub fn ensure(&mut self, device: &wgpu::Device, width: u32, height: u32, format: wgpu::TextureFormat) {
		if width == 0 || height == 0 { return; }
		let shaders_changed = self.refresh_shader_source_if_needed();
		let need_rebuild = self.renderer.is_none() || self.size != (width, height) || self.format != Some(format) || shaders_changed;
		if !need_rebuild { return; }

		match VoxelRasterRenderer::new(
			device,
			width,
			height,
			format,
			&self.camera_bind_group_layout,
			&self.model_bind_group_layout,
			&self.shader_source,
		) {
			Ok(renderer) => {
				self.renderer = Some(renderer);
				self.size = (width, height);
				self.format = Some(format);
			}
			Err(error) => {
				log::error!("Failed to build VoxelRasterRenderer: {error}");
			}
		}
	}

	#[cfg(all(feature = "shader_hot_reload", not(target_arch = "wasm32")))]
	fn refresh_shader_source_if_needed(&mut self) -> bool {
		let Some(hot_reload) = self.hot_reload.as_ref() else { return false; };
		if !hot_reload.take_dirty() { return false; }
		match crate::shader_sources::load_from_disk() {
			Ok(shader_source) => {
				self.shader_source = shader_source;
				log::info!("Reloaded voxel raster shader from disk");
				true
			}
			Err(error) => {
				log::error!("Failed to hot reload voxel raster shader: {error}");
				false
			}
		}
	}

	#[cfg(not(all(feature = "shader_hot_reload", not(target_arch = "wasm32"))))]
	fn refresh_shader_source_if_needed(&mut self) -> bool { false }

	pub fn ensure_model_buffer_capacity(&mut self, device: &wgpu::Device, draw_count: usize) {
		let needed_size = (draw_count.max(1) as u64) * self.model_stride;
		if needed_size <= self.model_buffer_size { return; }

		self.model_buffer_size = needed_size.next_power_of_two();
		self.model_buffer = WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("raster_model_uniform"),
			size: self.model_buffer_size,
			usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		}));
		self.model_bind_group = ModelUniform::get_bind_group(device, &self.model_bind_group_layout, &self.model_buffer, 0);
	}
}
