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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct ResidentRasterVoxels {
	pub entity: Entity,
	pub buffer_id: u32,
	pub palette_id: u32,
	pub generation: u64,
}

#[derive(Clone, Copy)]
struct PendingCopy {
	entity: Entity,
	generation: u64,
	src_offset: u32,
	size: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct SlotEntry {
	generation: u64,
	src_offset: u32,
	size: u32,
	dst_offset: u32,
}

#[derive(Clone, Copy)]
struct CopyRegion {
	src_offset: u64,
	dst_offset: u64,
	size: u64,
}

fn record_copy_runs(encoder: &mut wgpu::CommandEncoder, src: &GpuBuffer, dst: &GpuBuffer, regions: &[CopyRegion]) {
	let Some(mut run) = regions.first().copied() else { return; };
	for region in &regions[1..] {
		let contiguous_src = run.src_offset + run.size == region.src_offset;
		let contiguous_dst = run.dst_offset + run.size == region.dst_offset;
		if contiguous_src && contiguous_dst {
			run.size += region.size;
		} else {
			encoder.copy_buffer_to_buffer(src, run.src_offset, dst, run.dst_offset, run.size);
			run = *region;
		}
	}
	encoder.copy_buffer_to_buffer(src, run.src_offset, dst, run.dst_offset, run.size);
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
	last_resident: Vec<ResidentRasterVoxels>,
	face_slot_entries: Vec<HashMap<Entity, SlotEntry>>,
	palette_slot_entries: Vec<HashMap<Entity, SlotEntry>>,
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

fn create_slot_entry_maps() -> Vec<HashMap<Entity, SlotEntry>> {
	(0..SLOTS).map(|_| HashMap::new()).collect()
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
			last_resident: Vec::new(),
			face_slot_entries: create_slot_entry_maps(),
			palette_slot_entries: create_slot_entry_maps(),
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

	pub fn upload(&mut self, world_gpu: &WorldGpuData, resident: &[ResidentRasterVoxels]) {
		if self.last_resident == resident {
			return;
		}

		self.current = (self.current + 1) % SLOTS;

		let mut face_bytes = 0u64;
		let mut palette_bytes = 0u64;
		let mut face_entries = Vec::with_capacity(resident.len());
		let mut palette_entries = Vec::with_capacity(resident.len());
		for item in resident {
			let Some(face_held) = world_gpu.packed_raster_face_dynamic_buffer.held_buffer(item.buffer_id) else { continue; };
			let Some(palette_held) = world_gpu.packed_raster_palette_dynamic_buffer.held_buffer(item.palette_id) else { continue; };

			face_bytes += align_up(face_held.size(), self.face_alignment) as u64;
			palette_bytes += align_up(palette_held.size(), self.palette_alignment) as u64;
			face_entries.push(PendingCopy {
				entity: item.entity,
				generation: item.generation,
				src_offset: face_held.offset(),
				size: face_held.size(),
			});
			palette_entries.push(PendingCopy {
				entity: item.entity,
				generation: item.generation,
				src_offset: palette_held.offset(),
				size: palette_held.size(),
			});
		}
		self.ensure_face_capacity(face_bytes);
		self.ensure_palette_capacity(palette_bytes);

		face_entries.sort_unstable_by_key(|entry| entry.src_offset);
		palette_entries.sort_unstable_by_key(|entry| entry.src_offset);

		let mut face_offsets = HashMap::with_capacity(face_entries.len());
		let mut palette_offsets = HashMap::with_capacity(palette_entries.len());
		let mut face_regions = Vec::with_capacity(face_entries.len());
		let mut palette_regions = Vec::with_capacity(palette_entries.len());
		let mut next_face_slot_entries = HashMap::with_capacity(face_entries.len());
		let mut next_palette_slot_entries = HashMap::with_capacity(palette_entries.len());

		{
			let previous_face_slot_entries = &self.face_slot_entries[self.current];
			let mut face_off = 0u32;
			for entry in face_entries {
				let copy_size = align_up(entry.size, wgpu::COPY_BUFFER_ALIGNMENT as u32) as u64;
				let slot_entry = SlotEntry {
					generation: entry.generation,
					src_offset: entry.src_offset,
					size: entry.size,
					dst_offset: face_off,
				};
				if previous_face_slot_entries.get(&entry.entity).copied() != Some(slot_entry) {
					face_regions.push(CopyRegion {
						src_offset: entry.src_offset as u64,
						dst_offset: face_off as u64,
						size: copy_size,
					});
				}
				face_offsets.insert(entry.entity, face_off);
				next_face_slot_entries.insert(entry.entity, slot_entry);
				face_off += align_up(entry.size, self.face_alignment);
			}
		}

		{
			let previous_palette_slot_entries = &self.palette_slot_entries[self.current];
			let mut palette_off = 0u32;
			for entry in palette_entries {
				let copy_size = align_up(entry.size, wgpu::COPY_BUFFER_ALIGNMENT as u32) as u64;
				let slot_entry = SlotEntry {
					generation: entry.generation,
					src_offset: entry.src_offset,
					size: entry.size,
					dst_offset: palette_off,
				};
				if previous_palette_slot_entries.get(&entry.entity).copied() != Some(slot_entry) {
					palette_regions.push(CopyRegion {
						src_offset: entry.src_offset as u64,
						dst_offset: palette_off as u64,
						size: copy_size,
					});
				}
				palette_offsets.insert(entry.entity, palette_off);
				next_palette_slot_entries.insert(entry.entity, slot_entry);
				palette_off += align_up(entry.size, self.palette_alignment);
			}
		}

		if !face_regions.is_empty() || !palette_regions.is_empty() {
			let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("raster_residency_pack") });
			let face_src = world_gpu.packed_raster_face_dynamic_buffer.buffer();
			let palette_src = world_gpu.packed_raster_palette_dynamic_buffer.buffer();
			let face_dst = &self.face_slots[self.current];
			let palette_dst = &self.palette_slots[self.current];
			record_copy_runs(&mut encoder, face_src, face_dst, &face_regions);
			record_copy_runs(&mut encoder, palette_src, palette_dst, &palette_regions);
			self.queue.submit(std::iter::once(encoder.finish()));
		}

		self.face_slot_entries[self.current] = next_face_slot_entries;
		self.palette_slot_entries[self.current] = next_palette_slot_entries;
		if face_offsets.len() == resident.len() && palette_offsets.len() == resident.len() {
			self.last_resident.clear();
			self.last_resident.extend_from_slice(resident);
		} else {
			self.last_resident.clear();
		}
		self.face_offsets = face_offsets;
		self.palette_offsets = palette_offsets;
	}

	fn ensure_face_capacity(&mut self, bytes: u64) {
		if bytes > self.face_capacity {
			self.face_capacity = bytes.next_power_of_two().max(INITIAL_CAPACITY).min(self.binding_limit);
			self.face_slots = create_slots(&self.device, self.face_capacity, self.usage);
			self.face_slot_entries = create_slot_entry_maps();
		}
	}

	fn ensure_palette_capacity(&mut self, bytes: u64) {
		if bytes > self.palette_capacity {
			self.palette_capacity = bytes.next_power_of_two().max(INITIAL_CAPACITY).min(self.binding_limit);
			self.palette_slots = create_slots(&self.device, self.palette_capacity, self.usage);
			self.palette_slot_entries = create_slot_entry_maps();
		}
	}

	pub fn face_offsets(&self) -> &HashMap<Entity, u32> { &self.face_offsets }
	pub fn palette_offsets(&self) -> &HashMap<Entity, u32> { &self.palette_offsets }
	pub fn face_buffer(&self) -> &GpuBuffer { &self.face_slots[self.current] }
	pub fn palette_buffer(&self) -> &GpuBuffer { &self.palette_slots[self.current] }
}
