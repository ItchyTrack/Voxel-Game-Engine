use bevy::ecs::resource::Resource;
use bevy::ecs::world::FromWorld;
use bevy::render::renderer::{RenderDevice, WgpuWrapper};

use crate::camera::CameraUniform;
use crate::model::ModelUniform;
use crate::voxel_raster_renderer::VoxelRasterRenderer;

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
		}
	}
}

impl VoxelRasterRendererResource {
	pub fn ensure(&mut self, device: &wgpu::Device, width: u32, height: u32, format: wgpu::TextureFormat) {
		if width == 0 || height == 0 { return; }
		let need_rebuild = self.renderer.is_none() || self.size != (width, height) || self.format != Some(format);
		if !need_rebuild { return; }

		match VoxelRasterRenderer::new(
			device,
			width,
			height,
			format,
			&self.camera_bind_group_layout,
			&self.model_bind_group_layout,
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
