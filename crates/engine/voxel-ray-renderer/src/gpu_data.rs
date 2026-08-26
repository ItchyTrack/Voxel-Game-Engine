use std::sync::{Arc, Mutex, MutexGuard, OnceLock};

use bevy::{ecs::resource::Resource, render::renderer::{RenderDevice, RenderQueue}};
use voxel_gpu::packed_dynamic_buffer::PackedDynamicBuffer;

pub const TREE_BUFFER_ALIGNMENT: u32 = 12;
pub const VOXEL_BUFFER_ALIGNMENT: u32 = 4;

#[derive(Debug)]
pub struct RayGpuBuffers {
	pub trees: PackedDynamicBuffer,
	pub voxels: PackedDynamicBuffer,
	next_generation: u64,
}

impl RayGpuBuffers {
	pub fn next_generation(&mut self) -> u64 {
		let generation = self.next_generation;
		self.next_generation = self.next_generation.wrapping_add(1);
		generation
	}

	pub fn collect_garbage(&mut self) {
		self.trees.collect_garbage();
		self.voxels.collect_garbage();
	}
}

#[derive(Resource, Clone, Default)]
pub struct RayWorldGpuData {
	inner: Arc<OnceLock<Mutex<RayGpuBuffers>>>,
}

impl RayWorldGpuData {
	pub fn initialize(&self, device: &RenderDevice, queue: &RenderQueue) {
		let usage = wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC;
		let _ = self.inner.set(Mutex::new(RayGpuBuffers {
			trees: PackedDynamicBuffer::new(device, queue, TREE_BUFFER_ALIGNMENT, usage).expect("failed to create ray tree buffer"),
			voxels: PackedDynamicBuffer::new(device, queue, VOXEL_BUFFER_ALIGNMENT, usage).expect("failed to create ray voxel buffer"),
			next_generation: 1,
		}));
	}
	pub fn lock(&self) -> MutexGuard<'_, RayGpuBuffers> { self.inner.get().expect("ray GPU data was not initialized").lock().unwrap() }
}

pub fn collect_ray_gpu_garbage(gpu: bevy::prelude::Res<RayWorldGpuData>) {
	gpu.lock().collect_garbage();
}
