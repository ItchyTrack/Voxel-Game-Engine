use std::sync::{Arc, Mutex, MutexGuard, OnceLock};

use bevy::prelude::Resource;
use bevy::render::renderer::{RenderDevice, RenderQueue};
use voxel_gpu::packed_buffer_group::{PackedBufferGroupAllocation, PackedBufferGroup};

#[derive(Resource, Clone, Default)]
pub struct MarchingWorldGpuData {
	inner: Arc<OnceLock<Mutex<PackedBufferGroup>>>,
}

impl MarchingWorldGpuData {
	pub fn initialize(&self, device: &RenderDevice, queue: &RenderQueue) {
		let storage = PackedBufferGroup::new(
			device,
			queue,
			std::mem::align_of::<crate::marching_cubes::MarchingTriangle>() as u32,
			wgpu::BufferUsages::STORAGE,
			"marching_triangle_buffer",
		).expect("failed to create packed marching-cubes storage");
		let _ = self.inner.set(Mutex::new(storage));
	}

	pub fn create_vertex_buffer(&self, data: Vec<u8>) -> PackedBufferGroupAllocation {
		self.lock().add_buffer(data).expect("failed to allocate marching-cubes tile data")
	}

	pub fn lock(&self) -> MutexGuard<'_, PackedBufferGroup> {
		self.inner.get().expect("marching-cubes GPU data was not initialized").lock().unwrap()
	}
}

pub fn collect_marching_gpu_garbage(gpu: bevy::prelude::Res<MarchingWorldGpuData>) {
	gpu.lock().collect_garbage();
}
