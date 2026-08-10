use std::sync::{Arc, OnceLock};

use bevy::render::renderer::{RenderDevice, WgpuWrapper};
use bevy::prelude::Resource;
use wgpu::util::DeviceExt;

pub type MarchingGpuBuffer = WgpuWrapper<wgpu::Buffer>;

#[derive(Resource, Clone, Default)]
pub struct MarchingWorldGpuData {
	device: Arc<OnceLock<WgpuWrapper<wgpu::Device>>>,
}

impl MarchingWorldGpuData {
	pub fn initialize(&self, device: &RenderDevice) {
		let _ = self.device.set(WgpuWrapper::new(device.wgpu_device().clone()));
	}

	pub fn create_vertex_buffer(&self, data: &[u8]) -> MarchingGpuBuffer {
		let device = self.device.get().expect("marching-cubes GPU data was not initialized");
		WgpuWrapper::new(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label: Some("marching_cubes_tile_vertices"),
			contents: data,
			usage: wgpu::BufferUsages::STORAGE,
		}))
	}
}
