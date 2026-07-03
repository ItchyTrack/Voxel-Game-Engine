use std::collections::HashMap;

use bevy::ecs::entity::Entity;
use bevy::ecs::resource::Resource;
use bevy::ecs::world::FromWorld;
use bevy::render::renderer::{RenderDevice, RenderQueue, WgpuWrapper};

use voxel_gpu::world_gpu_data::{RASTER_FACE_BUFFER_ALIGNMENT, WorldGpuData};

type GpuBuffer = WgpuWrapper<wgpu::Buffer>;
type GpuDevice = WgpuWrapper<wgpu::Device>;
type GpuQueue = WgpuWrapper<wgpu::Queue>;

const SLOTS: usize = 2;
const INITIAL_CAPACITY: u64 = 1 << 20;

fn align_up(value: u32, alignment: u32) -> u32 {
	value.next_multiple_of(alignment)
}

#[derive(Resource)]
pub struct RasterResidencyBuffers {
	device: GpuDevice,
	queue: GpuQueue,
	usage: wgpu::BufferUsages,
	alignment: u32,
	face_slots: Vec<GpuBuffer>,
	capacity: u64,
	binding_limit: u64,
	current: usize,
	offsets: HashMap<Entity, u32>,
}

fn create_slots(device: &GpuDevice, capacity: u64, usage: wgpu::BufferUsages) -> Vec<GpuBuffer> {
	(0..SLOTS)
		.map(|_| {
			WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
				label: Some("raster_residency_slot"),
				size: capacity,
				usage,
				mapped_at_creation: false,
			}))
		})
		.collect()
}

impl FromWorld for RasterResidencyBuffers {
	fn from_world(world: &mut bevy::ecs::world::World) -> Self {
		let render_device = world.resource::<RenderDevice>();
		let render_queue = world.resource::<RenderQueue>();
		let raw_device = render_device.wgpu_device();
		let device = WgpuWrapper::new(raw_device.clone());
		let queue = WgpuWrapper::new(bevy::render::renderer::WgpuWrapper::clone(&**render_queue).into_inner());
		let alignment = align_up(RASTER_FACE_BUFFER_ALIGNMENT, wgpu::COPY_BUFFER_ALIGNMENT as u32);
		let binding_limit = raw_device.limits().max_storage_buffer_binding_size as u64;
		let usage = wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST;
		Self {
			face_slots: create_slots(&device, INITIAL_CAPACITY, usage),
			capacity: INITIAL_CAPACITY,
			binding_limit,
			alignment,
			current: 0,
			offsets: HashMap::new(),
			device,
			queue,
			usage,
		}
	}
}

impl RasterResidencyBuffers {
	pub fn binding_limit(&self) -> u64 { self.binding_limit }
	pub fn alignment(&self) -> u32 { self.alignment }

	pub fn upload(&mut self, world_gpu: &WorldGpuData, resident: &[(Entity, u32)]) {
		self.current = (self.current + 1) % SLOTS;

		let mut bytes = 0u64;
		for (_, buffer_id) in resident {
			if let Some(held) = world_gpu.packed_raster_face_dynamic_buffer.held_buffer(*buffer_id) {
				bytes += align_up(held.size(), self.alignment) as u64;
			}
		}
		self.ensure_capacity(bytes);

		let mut offsets = HashMap::with_capacity(resident.len());
		let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("raster_residency_pack") });
		let src = world_gpu.packed_raster_face_dynamic_buffer.buffer();
		let dst = &self.face_slots[self.current];

		let mut face_off = 0u32;
		for (entity, buffer_id) in resident {
			let Some(held) = world_gpu.packed_raster_face_dynamic_buffer.held_buffer(*buffer_id) else { continue; };
			let copy_size = align_up(held.size(), wgpu::COPY_BUFFER_ALIGNMENT as u32) as u64;
			encoder.copy_buffer_to_buffer(src, held.offset() as u64, dst, face_off as u64, copy_size);
			offsets.insert(*entity, face_off);
			face_off += align_up(held.size(), self.alignment);
		}

		self.queue.submit(std::iter::once(encoder.finish()));
		self.offsets = offsets;
	}

	fn ensure_capacity(&mut self, bytes: u64) {
		if bytes > self.capacity {
			self.capacity = bytes.next_power_of_two().max(INITIAL_CAPACITY).min(self.binding_limit);
			self.face_slots = create_slots(&self.device, self.capacity, self.usage);
		}
	}

	pub fn offsets(&self) -> &HashMap<Entity, u32> { &self.offsets }
	pub fn face_buffer(&self) -> &GpuBuffer { &self.face_slots[self.current] }
}
