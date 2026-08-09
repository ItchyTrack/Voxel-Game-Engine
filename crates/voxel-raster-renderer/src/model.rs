use bevy::math::Mat4;
use bevy::render::render_resource::{DynamicUniformBuffer, ShaderType};
use bevy::render::renderer::WgpuWrapper;

type GpuBindGroup = WgpuWrapper<wgpu::BindGroup>;
type GpuBindGroupLayout = WgpuWrapper<wgpu::BindGroupLayout>;

#[derive(Debug, Copy, Clone, Default, ShaderType)]
pub struct ModelUniform {
	model: Mat4,
	palette_offset: u32,
}

impl ModelUniform {
	pub fn from_mat4(mat: &Mat4, palette_offset: u32) -> Self {
		Self { model: *mat, palette_offset }
	}

	pub fn get_dynamic_offset_bind_group_layout(device: &wgpu::Device, binding: u32) -> GpuBindGroupLayout {
		WgpuWrapper::new(device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[wgpu::BindGroupLayoutEntry {
				binding,
				visibility: wgpu::ShaderStages::VERTEX,
				ty: wgpu::BindingType::Buffer {
					ty: wgpu::BufferBindingType::Uniform,
					has_dynamic_offset: true,
					min_binding_size: Some(Self::min_size()),
				},
				count: None,
			}],
			label: Some("raster_model_bind_group_layout"),
		}))
	}

	pub fn get_bind_group(
		device: &wgpu::Device,
		layout: &GpuBindGroupLayout,
		uniforms: &DynamicUniformBuffer<Self>,
		binding: u32,
	) -> GpuBindGroup {
		WgpuWrapper::new(device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout,
			entries: &[wgpu::BindGroupEntry {
				binding,
				resource: uniforms.binding().expect("model uniform buffer must be initialized"),
			}],
			label: Some("raster_model_bind_group"),
		}))
	}
}
