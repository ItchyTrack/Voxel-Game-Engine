use std::sync::{Arc, Mutex, MutexGuard, OnceLock};

use bevy::{ecs::resource::Resource, render::renderer::{RenderDevice, RenderQueue}};
use voxel_gpu::packed_buffer_group::PackedBufferGroup;

pub const FACE_BUFFER_ALIGNMENT: u32 = 4;
pub const VOXEL_DATA_ALIGNMENT: u32 = 16;

#[derive(Debug)]
pub struct RasterGpuBuffers {
	pub faces: PackedBufferGroup,
	pub voxel_data: PackedBufferGroup,
}

impl RasterGpuBuffers {
	pub fn collect_garbage(&mut self) {
		self.faces.collect_garbage();
		self.voxel_data.collect_garbage();
	}
}

#[derive(Resource, Clone, Default)]
pub struct RasterWorldGpuData {
	inner: Arc<OnceLock<Mutex<RasterGpuBuffers>>>,
}

impl RasterWorldGpuData {
	pub fn initialize(&self, device: &RenderDevice, queue: &RenderQueue) {
		let usage = wgpu::BufferUsages::STORAGE;
		let _ = self.inner.set(Mutex::new(RasterGpuBuffers {
			faces: PackedBufferGroup::new(device, queue, FACE_BUFFER_ALIGNMENT, usage, "raster_face_buffer")
				.expect("failed to create packed raster face storage"),
			voxel_data: PackedBufferGroup::new(device, queue, VOXEL_DATA_ALIGNMENT, usage, "raster_voxel_data_buffer")
				.expect("failed to create packed raster voxel data storage"),
		}));
	}
	pub fn lock(&self) -> MutexGuard<'_, RasterGpuBuffers> { self.inner.get().expect("raster GPU data was not initialized").lock().unwrap() }
}

pub fn collect_raster_gpu_garbage(gpu: bevy::prelude::Res<RasterWorldGpuData>) {
	gpu.lock().collect_garbage();
}
