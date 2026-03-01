use glam::Mat4;

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct MatrixUniform {
    matrix: [[f32; 4]; 4],
}

impl MatrixUniform {
	pub fn from_mat4(mat: Mat4) -> MatrixUniform {
		MatrixUniform{ matrix: mat.to_cols_array_2d() }
	}
	pub fn get_bind_group_layout(device: &wgpu::Device, binding: u32) -> wgpu::BindGroupLayout {
		device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
				entries: &[wgpu::BindGroupLayoutEntry {
					binding: binding,
					visibility: wgpu::ShaderStages::VERTEX,
					ty: wgpu::BindingType::Buffer {
						ty: wgpu::BufferBindingType::Uniform,
						has_dynamic_offset: false,
						min_binding_size: None,
					},
					count: None,
				}],
				label: Some("matrix_bind_group_layout"),
			})
	}
	pub fn get_buffer(device: &wgpu::Device, binding: u32) -> (wgpu::Buffer, wgpu::BindGroup, wgpu::BindGroupLayout) {
		let buffer = device.create_buffer(&wgpu::BufferDescriptor {
				label: Some("Matrix Buffer"),
				size: std::mem::size_of::<MatrixUniform>() as u64,
				usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
				mapped_at_creation: false,
			});
		let bind_group_layout = Self::get_bind_group_layout(&device, binding);
		let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
				layout: &bind_group_layout,
				entries: &[wgpu::BindGroupEntry {
					binding: binding,
					resource: buffer.as_entire_binding(),
				}],
				label: Some("matrix_bind_group"),
			});
		(buffer, bind_group, bind_group_layout)
	}
}

