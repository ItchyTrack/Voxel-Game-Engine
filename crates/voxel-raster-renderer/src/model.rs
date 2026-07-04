use bevy::{math::Mat4, render::renderer::WgpuWrapper};

type GpuBindGroup = WgpuWrapper<wgpu::BindGroup>;
type GpuBindGroupLayout = WgpuWrapper<wgpu::BindGroupLayout>;

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct ModelUniform {
	model: [[f32; 4]; 4],
	palette_offset: u32,
	_padding: [u32; 3],
}

impl ModelUniform {
	pub fn from_mat4(mat: &Mat4, palette_offset: u32) -> Self {
		Self {
			model: mat.to_cols_array_2d(),
			palette_offset,
			_padding: [0; 3],
		}
	}

	pub fn get_dynamic_offset_bind_group_layout(device: &wgpu::Device, binding: u32) -> GpuBindGroupLayout {
		WgpuWrapper::new(device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[wgpu::BindGroupLayoutEntry {
				binding,
				visibility: wgpu::ShaderStages::VERTEX,
				ty: wgpu::BindingType::Buffer {
					ty: wgpu::BufferBindingType::Uniform,
					has_dynamic_offset: true,
					min_binding_size: None,
				},
				count: None,
			}],
			label: Some("raster_model_bind_group_layout"),
		}))
	}

	pub fn get_bind_group(device: &wgpu::Device, layout: &GpuBindGroupLayout, buffer: &wgpu::Buffer, binding: u32) -> GpuBindGroup {
		WgpuWrapper::new(device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout,
			entries: &[wgpu::BindGroupEntry {
				binding,
				resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
					buffer,
					offset: 0,
					size: Some(std::num::NonZeroU64::new(std::mem::size_of::<ModelUniform>() as u64).unwrap()),
				}),
			}],
			label: Some("raster_model_bind_group"),
		}))
	}
}
