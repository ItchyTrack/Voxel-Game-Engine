use wgpu::Device;

use crate::gpu_objects::packed_dynamic_buffer::PackedDynamicBuffer;

pub struct WorldGpuData {
	pub packed_64_tree_dynamic_buffer: PackedDynamicBuffer,
	pub packed_voxel_data_dynamic_buffer: PackedDynamicBuffer,
}

impl WorldGpuData {
	pub fn new(device: &Device) -> anyhow::Result<Self> {
		let packed_64_tree_dynamic_buffer = PackedDynamicBuffer::new(&device, 12, wgpu::BufferUsages::STORAGE);
		if let Err(err) = packed_64_tree_dynamic_buffer {
			println!("{}", err);
			return Err(anyhow::Error::msg(err));
		}
		let packed_64_tree_dynamic_buffer = packed_64_tree_dynamic_buffer.unwrap();

		let packed_voxel_data_dynamic_buffer = PackedDynamicBuffer::new(&device, 4, wgpu::BufferUsages::STORAGE);
		if let Err(err) = packed_voxel_data_dynamic_buffer {
			println!("{}", err);
			return Err(anyhow::Error::msg(err));
		}
		let packed_voxel_data_dynamic_buffer = packed_voxel_data_dynamic_buffer.unwrap();

		Ok(Self {
			packed_64_tree_dynamic_buffer,
			packed_voxel_data_dynamic_buffer,
		})
	}
}
