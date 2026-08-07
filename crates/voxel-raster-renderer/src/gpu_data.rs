use std::sync::{Arc, Mutex, MutexGuard, OnceLock};

use bevy::{ecs::resource::Resource, render::renderer::{RenderDevice, RenderQueue}};
use voxel_gpu::packed_dynamic_buffer::PackedDynamicBuffer;

pub const FACE_BUFFER_ALIGNMENT: u32 = 4;
pub const PALETTE_BUFFER_ALIGNMENT: u32 = 4;

#[derive(Debug)]
pub struct RasterGpuBuffers {
	pub faces: PackedDynamicBuffer,
	pub palettes: PackedDynamicBuffer,
	next_generation: u64,
}

impl RasterGpuBuffers {
	pub fn next_generation(&mut self) -> u64 {
		let generation = self.next_generation;
		self.next_generation = self.next_generation.wrapping_add(1);
		generation
	}

	pub fn collect_garbage(&mut self) {
		self.faces.collect_garbage();
		self.palettes.collect_garbage();
	}
}

#[derive(Resource, Clone, Default)]
pub struct RasterWorldGpuData {
	inner: Arc<OnceLock<Mutex<RasterGpuBuffers>>>,
}

impl RasterWorldGpuData {
	pub fn initialize(&self, device: &RenderDevice, queue: &RenderQueue) {
		let usage = wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC;
		let _ = self.inner.set(Mutex::new(RasterGpuBuffers {
			faces: PackedDynamicBuffer::new(device, queue, FACE_BUFFER_ALIGNMENT, usage).expect("failed to create raster face buffer"),
			palettes: PackedDynamicBuffer::new(device, queue, PALETTE_BUFFER_ALIGNMENT, usage).expect("failed to create raster palette buffer"),
			next_generation: 1,
		}));
	}
	pub fn lock(&self) -> MutexGuard<'_, RasterGpuBuffers> { self.inner.get().expect("raster GPU data was not initialized").lock().unwrap() }
}

pub fn collect_raster_gpu_garbage(gpu: bevy::prelude::Res<RasterWorldGpuData>) {
	gpu.lock().collect_garbage();
}
