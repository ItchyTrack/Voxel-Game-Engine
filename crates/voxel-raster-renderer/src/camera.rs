use bevy::{camera::Projection, render::renderer::WgpuWrapper, transform::components::Transform};

type GpuBuffer = WgpuWrapper<wgpu::Buffer>;
type GpuBindGroup = WgpuWrapper<wgpu::BindGroup>;
type GpuBindGroupLayout = WgpuWrapper<wgpu::BindGroupLayout>;

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct CameraUniform {
	view_proj: [[f32; 4]; 4],
}

impl CameraUniform {
	pub fn from_camera(transform: &Transform, projection: &Projection) -> Self {
		let view_proj = projection.get_clip_from_view() * transform.to_matrix().inverse();
		Self { view_proj: view_proj.to_cols_array_2d() }
	}

	pub fn as_bytes(&self) -> &[u8] {
		bytemuck::bytes_of(self)
	}

	pub fn get_bind_group_layout(device: &wgpu::Device, binding: u32) -> GpuBindGroupLayout {
		WgpuWrapper::new(device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[wgpu::BindGroupLayoutEntry {
				binding,
				visibility: wgpu::ShaderStages::VERTEX,
				ty: wgpu::BindingType::Buffer {
					ty: wgpu::BufferBindingType::Uniform,
					has_dynamic_offset: false,
					min_binding_size: None,
				},
				count: None,
			}],
			label: Some("Raster Camera Bind Group Layout"),
		}))
	}

	pub fn get_buffer(device: &wgpu::Device, binding: u32) -> (GpuBuffer, GpuBindGroup, GpuBindGroupLayout) {
		let buffer = WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("raster_camera_buffer"),
			size: std::mem::size_of::<CameraUniform>() as u64,
			usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		}));
		let bind_group_layout = Self::get_bind_group_layout(device, binding);
		let bind_group = WgpuWrapper::new(device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &bind_group_layout,
			entries: &[wgpu::BindGroupEntry {
				binding,
				resource: buffer.as_entire_binding(),
			}],
			label: Some("raster_camera_bind_group"),
		}));
		(buffer, bind_group, bind_group_layout)
	}
}
