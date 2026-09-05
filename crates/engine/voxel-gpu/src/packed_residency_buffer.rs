use std::collections::HashMap;

use bevy::ecs::entity::Entity;
use bevy::render::renderer::{RenderDevice, RenderQueue, WgpuWrapper};

use crate::packed_dynamic_buffer::PackedDynamicBuffer;
use crate::residency_packing::{CopyRegion, PendingUpload, SlotEntry, plan_residency};

type GpuBuffer = WgpuWrapper<wgpu::Buffer>;
type GpuDevice = WgpuWrapper<wgpu::Device>;
type GpuQueue = WgpuWrapper<wgpu::Queue>;

const INITIAL_CAPACITY: u64 = 1 << 20;
const COMPACTION_MOVE_CALL_BUDGET: usize = 64;

fn record_copy_runs(
	encoder: &mut wgpu::CommandEncoder,
	src: &GpuBuffer,
	dst: &GpuBuffer,
	regions: &[CopyRegion],
) {
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

/// Compact render-only working set for one [`PackedDynamicBuffer`] stream.
///
/// `SLOTS` controls how many independently retained destination buffers rotate
/// between uploads. Callers compose one instance per logical data stream.
pub struct PackedResidencyBuffer<const SLOTS: usize> {
	device: GpuDevice,
	queue: GpuQueue,
	usage: wgpu::BufferUsages,
	label: &'static str,
	alignment: u32,
	slots: Vec<GpuBuffer>,
	capacity: u64,
	binding_limit: u64,
	current: usize,
	offsets: HashMap<Entity, u32>,
	slot_entries: Vec<Vec<SlotEntry>>,
	slot_entry_indices: Vec<HashMap<Entity, usize>>,
}

impl<const SLOTS: usize> PackedResidencyBuffer<SLOTS> {
	pub fn new(
		render_device: &RenderDevice,
		render_queue: &RenderQueue,
		source_alignment: u32,
		label: &'static str,
	) -> Self {
		assert!(SLOTS > 0, "packed residency buffer requires at least one slot");
		let raw_device = render_device.wgpu_device();
		let device = WgpuWrapper::new(raw_device.clone());
		let queue = WgpuWrapper::new(WgpuWrapper::clone(&**render_queue).into_inner());
		let usage = wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST;
		let alignment = source_alignment.next_multiple_of(wgpu::COPY_BUFFER_ALIGNMENT as u32);
		Self {
			slots: Self::create_slots(&device, INITIAL_CAPACITY, usage, label),
			capacity: INITIAL_CAPACITY,
			binding_limit: raw_device.limits().max_storage_buffer_binding_size,
			current: 0,
			offsets: HashMap::new(),
			slot_entries: Self::create_slot_entry_lists(),
			slot_entry_indices: Self::create_slot_entry_index_maps(),
			device,
			queue,
			usage,
			label,
			alignment,
		}
	}

	pub fn binding_limit(&self) -> u64 { self.binding_limit }
	pub fn alignment(&self) -> u32 { self.alignment }
	pub fn offsets(&self) -> &HashMap<Entity, u32> { &self.offsets }
	pub fn buffer(&self) -> &GpuBuffer { &self.slots[self.current] }

	fn prepare_upload(&mut self, mut entries: Vec<PendingUpload>) -> Vec<CopyRegion> {
		self.current = (self.current + 1) % SLOTS;
		entries.sort_unstable_by_key(|entry| entry.src_offset);

		let mut plan = plan_residency(
			&self.slot_entries[self.current],
			&self.slot_entry_indices[self.current],
			&entries,
			self.alignment,
			COMPACTION_MOVE_CALL_BUDGET,
		);
		if plan.required_capacity > self.binding_limit {
			plan = plan_residency(&[], &HashMap::new(), &entries, self.alignment, 0);
		}
		if plan.required_capacity > self.capacity {
			self.ensure_capacity(plan.required_capacity);
			plan = plan_residency(
				&self.slot_entries[self.current],
				&self.slot_entry_indices[self.current],
				&entries,
				self.alignment,
				COMPACTION_MOVE_CALL_BUDGET,
			);
		}

		self.slot_entries[self.current] = plan.slot_entries;
		self.slot_entry_indices[self.current] = self.slot_entries[self.current]
			.iter()
			.enumerate()
			.map(|(index, entry)| (entry.entity, index))
			.collect();
		self.offsets = plan.offsets;
		plan.copy_regions
	}

	fn create_slots(
		device: &GpuDevice,
		capacity: u64,
		usage: wgpu::BufferUsages,
		label: &'static str,
	) -> Vec<GpuBuffer> {
		(0..SLOTS)
			.map(|_| {
				WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
					label: Some(label),
					size: capacity,
					usage,
					mapped_at_creation: false,
				}))
			})
			.collect()
	}

	fn create_slot_entry_lists() -> Vec<Vec<SlotEntry>> {
		(0..SLOTS).map(|_| Vec::new()).collect()
	}

	fn create_slot_entry_index_maps() -> Vec<HashMap<Entity, usize>> {
		(0..SLOTS).map(|_| HashMap::new()).collect()
	}

	fn ensure_capacity(&mut self, bytes: u64) {
		if bytes <= self.capacity { return; }
		self.capacity = bytes.next_power_of_two().max(INITIAL_CAPACITY).min(self.binding_limit);
		self.slots = Self::create_slots(&self.device, self.capacity, self.usage, self.label);
		self.slot_entries = Self::create_slot_entry_lists();
		self.slot_entry_indices = Self::create_slot_entry_index_maps();
	}
}

/// Updates two residency streams and submits their copies in one command buffer.
pub fn upload_packed_residency_pair<const SLOTS: usize>(
	first: &mut PackedResidencyBuffer<SLOTS>,
	first_source: &PackedDynamicBuffer,
	first_entries: Vec<PendingUpload>,
	second: &mut PackedResidencyBuffer<SLOTS>,
	second_source: &PackedDynamicBuffer,
	second_entries: Vec<PendingUpload>,
	encoder_label: &'static str,
) {
	let first_regions = first.prepare_upload(first_entries);
	let second_regions = second.prepare_upload(second_entries);
	if first_regions.is_empty() && second_regions.is_empty() { return; }

	let mut encoder = first.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
		label: Some(encoder_label),
	});
	record_copy_runs(
		&mut encoder,
		first_source.buffer(),
		&first.slots[first.current],
		&first_regions,
	);
	record_copy_runs(
		&mut encoder,
		second_source.buffer(),
		&second.slots[second.current],
		&second_regions,
	);
	first.queue.submit(std::iter::once(encoder.finish()));
}
