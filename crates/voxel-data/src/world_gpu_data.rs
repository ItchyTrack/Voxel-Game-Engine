use parking_lot::RwLock;
use wgpu::{Device, Queue};

use crate::{gpu_objects::packed_dynamic_buffer::PackedDynamicBuffer};

pub struct WorldGpuData {
	pub packed_64_tree_dynamic_buffer: RwLock<PackedDynamicBuffer>,
	pub packed_voxel_data_dynamic_buffer: RwLock<PackedDynamicBuffer>,
}

impl WorldGpuData {
	pub fn new(device: Device, queue: Queue) -> anyhow::Result<Self> {
		let packed_64_tree_dynamic_buffer = PackedDynamicBuffer::new(device.clone(), queue.clone(), 12, wgpu::BufferUsages::STORAGE);
		if let Err(err) = packed_64_tree_dynamic_buffer {
			println!("{}", err);
			return Err(anyhow::Error::msg(err));
		}
		let packed_64_tree_dynamic_buffer = packed_64_tree_dynamic_buffer.unwrap();

		let packed_voxel_data_dynamic_buffer = PackedDynamicBuffer::new(device, queue, 4, wgpu::BufferUsages::STORAGE);
		if let Err(err) = packed_voxel_data_dynamic_buffer {
			println!("{}", err);
			return Err(anyhow::Error::msg(err));
		}
		let packed_voxel_data_dynamic_buffer = packed_voxel_data_dynamic_buffer.unwrap();

		Ok(Self {
			packed_64_tree_dynamic_buffer: RwLock::new(packed_64_tree_dynamic_buffer),
			packed_voxel_data_dynamic_buffer: RwLock::new(packed_voxel_data_dynamic_buffer),
		})
	}
}
