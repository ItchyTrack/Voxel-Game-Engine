use bevy::{camera::Projection, render::renderer::WgpuWrapper, transform::components::Transform};

type GpuBuffer = WgpuWrapper<wgpu::Buffer>;
type GpuBindGroup = WgpuWrapper<wgpu::BindGroup>;
type GpuBindGroupLayout = WgpuWrapper<wgpu::BindGroupLayout>;

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct CameraUniform {
	transform: [[f32; 4]; 4],
	camera_view_size: [f32; 2],
	padding: [u8; 8],
}

impl CameraUniform {
	pub fn from_camera(transform: &Transform, projection: &Projection) -> Result<Self, ()> {
		if let Projection::Perspective(perspective_projection) = projection {
			let tan_fov = (perspective_projection.fov * 0.5).tan();
			Ok(Self {
				transform: transform.to_matrix().to_cols_array_2d(),
				camera_view_size: [
					tan_fov * perspective_projection.aspect_ratio,
					tan_fov,
				],
				padding: [0; 8],
			})
		} else {
			Err(())
		}
	}

	pub fn as_bytes(&self) -> &[u8] {
		bytemuck::bytes_of(self)
	}

	pub fn get_bind_group_layout(device: &wgpu::Device, binding: u32) -> GpuBindGroupLayout {
		WgpuWrapper::new(device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
				entries: &[wgpu::BindGroupLayoutEntry {
					binding: binding,
					visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::COMPUTE,
					ty: wgpu::BindingType::Buffer {
						ty: wgpu::BufferBindingType::Uniform,
						has_dynamic_offset: false,
						min_binding_size: None,
					},
					count: None,
				}],
				label: Some("Camera Bind Group Layout"),
			}))
	}
	pub fn get_buffer(device: &wgpu::Device, binding: u32) -> (GpuBuffer, GpuBindGroup, GpuBindGroupLayout) {
		let buffer = WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
				label: Some("camera_buffer"),
				size: std::mem::size_of::<CameraUniform>() as u64,
				usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
				mapped_at_creation: false,
			}));
		let bind_group_layout = Self::get_bind_group_layout(&device, binding);
		let bind_group = WgpuWrapper::new(device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &bind_group_layout,
			entries: &[wgpu::BindGroupEntry {
				binding,
				resource: buffer.as_entire_binding(),
			}],
			label: Some("camera_bind_group"),
		}));
		(buffer, bind_group, bind_group_layout)
	}
}
