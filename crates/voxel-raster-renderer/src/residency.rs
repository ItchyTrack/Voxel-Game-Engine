use std::collections::HashMap;

use bevy::ecs::entity::Entity;
use bevy::ecs::resource::Resource;
use bevy::ecs::world::FromWorld;
use bevy::render::renderer::{RenderDevice, RenderQueue, WgpuWrapper};

use voxel_gpu::world_gpu_data::{
	RASTER_FACE_BUFFER_ALIGNMENT, RASTER_PALETTE_BUFFER_ALIGNMENT, WorldGpuData,
};

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
	face_alignment: u32,
	palette_alignment: u32,
	face_slots: Vec<GpuBuffer>,
	palette_slots: Vec<GpuBuffer>,
	face_capacity: u64,
	palette_capacity: u64,
	binding_limit: u64,
	current: usize,
	face_offsets: HashMap<Entity, u32>,
	palette_offsets: HashMap<Entity, u32>,
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
		let face_alignment = align_up(RASTER_FACE_BUFFER_ALIGNMENT, wgpu::COPY_BUFFER_ALIGNMENT as u32);
		let palette_alignment = align_up(RASTER_PALETTE_BUFFER_ALIGNMENT, wgpu::COPY_BUFFER_ALIGNMENT as u32);
		let binding_limit = raw_device.limits().max_storage_buffer_binding_size as u64;
		let usage = wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST;
		Self {
			face_slots: create_slots(&device, INITIAL_CAPACITY, usage),
			palette_slots: create_slots(&device, INITIAL_CAPACITY, usage),
			face_capacity: INITIAL_CAPACITY,
			palette_capacity: INITIAL_CAPACITY,
			binding_limit,
			face_alignment,
			palette_alignment,
			current: 0,
			face_offsets: HashMap::new(),
			palette_offsets: HashMap::new(),
			device,
			queue,
			usage,
		}
	}
}

impl RasterResidencyBuffers {
	pub fn binding_limit(&self) -> u64 { self.binding_limit }
	pub fn face_alignment(&self) -> u32 { self.face_alignment }
	pub fn palette_alignment(&self) -> u32 { self.palette_alignment }

	pub fn upload(&mut self, world_gpu: &WorldGpuData, resident: &[(Entity, u32, u32)]) {
		self.current = (self.current + 1) % SLOTS;

		let mut face_bytes = 0u64;
		let mut palette_bytes = 0u64;
		for (_, buffer_id, palette_id) in resident {
			if let Some(held) = world_gpu.packed_raster_face_dynamic_buffer.held_buffer(*buffer_id) {
				face_bytes += align_up(held.size(), self.face_alignment) as u64;
			}
			if let Some(held) = world_gpu.packed_raster_palette_dynamic_buffer.held_buffer(*palette_id) {
				palette_bytes += align_up(held.size(), self.palette_alignment) as u64;
			}
		}
		self.ensure_face_capacity(face_bytes);
		self.ensure_palette_capacity(palette_bytes);

		let mut face_offsets = HashMap::with_capacity(resident.len());
		let mut palette_offsets = HashMap::with_capacity(resident.len());
		let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("raster_residency_pack") });
		let face_src = world_gpu.packed_raster_face_dynamic_buffer.buffer();
		let palette_src = world_gpu.packed_raster_palette_dynamic_buffer.buffer();
		let face_dst = &self.face_slots[self.current];
		let palette_dst = &self.palette_slots[self.current];

		let mut face_off = 0u32;
		let mut palette_off = 0u32;
		for (entity, buffer_id, palette_id) in resident {
			let Some(face_held) = world_gpu.packed_raster_face_dynamic_buffer.held_buffer(*buffer_id) else { continue; };
			let Some(palette_held) = world_gpu.packed_raster_palette_dynamic_buffer.held_buffer(*palette_id) else { continue; };

			let face_copy_size = align_up(face_held.size(), wgpu::COPY_BUFFER_ALIGNMENT as u32) as u64;
			encoder.copy_buffer_to_buffer(face_src, face_held.offset() as u64, face_dst, face_off as u64, face_copy_size);
			face_offsets.insert(*entity, face_off);
			face_off += align_up(face_held.size(), self.face_alignment);

			let palette_copy_size = align_up(palette_held.size(), wgpu::COPY_BUFFER_ALIGNMENT as u32) as u64;
			encoder.copy_buffer_to_buffer(palette_src, palette_held.offset() as u64, palette_dst, palette_off as u64, palette_copy_size);
			palette_offsets.insert(*entity, palette_off);
			palette_off += align_up(palette_held.size(), self.palette_alignment);
		}

		self.queue.submit(std::iter::once(encoder.finish()));
		self.face_offsets = face_offsets;
		self.palette_offsets = palette_offsets;
	}

	fn ensure_face_capacity(&mut self, bytes: u64) {
		if bytes > self.face_capacity {
			self.face_capacity = bytes.next_power_of_two().max(INITIAL_CAPACITY).min(self.binding_limit);
			self.face_slots = create_slots(&self.device, self.face_capacity, self.usage);
		}
	}

	fn ensure_palette_capacity(&mut self, bytes: u64) {
		if bytes > self.palette_capacity {
			self.palette_capacity = bytes.next_power_of_two().max(INITIAL_CAPACITY).min(self.binding_limit);
			self.palette_slots = create_slots(&self.device, self.palette_capacity, self.usage);
		}
	}

	pub fn face_offsets(&self) -> &HashMap<Entity, u32> { &self.face_offsets }
	pub fn palette_offsets(&self) -> &HashMap<Entity, u32> { &self.palette_offsets }
	pub fn face_buffer(&self) -> &GpuBuffer { &self.face_slots[self.current] }
	pub fn palette_buffer(&self) -> &GpuBuffer { &self.palette_slots[self.current] }
}
