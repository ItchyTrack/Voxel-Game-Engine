use std::collections::HashMap;

use bevy::ecs::entity::Entity;
use bevy::ecs::resource::Resource;
use bevy::ecs::world::FromWorld;
use bevy::render::renderer::{RenderDevice, RenderQueue, WgpuWrapper};

use crate::residency_packing::{plan_residency, CopyRegion, PendingUpload, SlotEntry};
use crate::world_gpu_data::{WorldGpuData, TREE_BUFFER_ALIGNMENT, VOXEL_BUFFER_ALIGNMENT};

type GpuBuffer = WgpuWrapper<wgpu::Buffer>;
type GpuDevice = WgpuWrapper<wgpu::Device>;
type GpuQueue = WgpuWrapper<wgpu::Queue>;

// Pipelined rendering is one frame behind, so two slots keep the main world's
// build off the slot the render thread is reading.
const SLOTS: usize = 2;
const INITIAL_CAPACITY: u64 = 1 << 20;
const COMPACTION_MOVE_CALL_BUDGET: usize = 64;

fn align_up(value: u32, alignment: u32) -> u32 {
	value.next_multiple_of(alignment)
}

/// Render-only compact copy of the sub-grids needed this frame. The render world
/// reads only these slots, never the big [`WorldGpuData`] buffers the main world
/// mutates, which is what avoids the write-during-read race.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ResidentVoxels {
	pub entity: Entity,
	pub tree_id: u32,
	pub voxels_id: u32,
	pub generation: u64,
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
pub struct ResidencyBuffers {
	device: GpuDevice,
	queue: GpuQueue,
	usage: wgpu::BufferUsages,
	tree_alignment: u32,
	voxel_alignment: u32,
	tree_slots: Vec<GpuBuffer>,
	voxel_slots: Vec<GpuBuffer>,
	tree_capacity: u64,
	voxel_capacity: u64,
	binding_limit: u64,
	current: usize,
	offsets: HashMap<Entity, (u32, u32)>,
	tree_slot_entries: Vec<Vec<SlotEntry>>,
	tree_slot_entry_indices: Vec<HashMap<Entity, usize>>,
	voxel_slot_entries: Vec<Vec<SlotEntry>>,
	voxel_slot_entry_indices: Vec<HashMap<Entity, usize>>,
}

fn create_slots(device: &GpuDevice, capacity: u64, usage: wgpu::BufferUsages) -> Vec<GpuBuffer> {
	(0..SLOTS)
		.map(|_| {
			WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
				label: Some("residency_slot"),
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

impl FromWorld for ResidencyBuffers {
	fn from_world(world: &mut bevy::ecs::world::World) -> Self {
		let render_device = world.resource::<RenderDevice>();
		let render_queue = world.resource::<RenderQueue>();
		let raw_device = render_device.wgpu_device();
		let device = WgpuWrapper::new(raw_device.clone());
		let queue = WgpuWrapper::new(bevy::render::renderer::WgpuWrapper::clone(&**render_queue).into_inner());
		let tree_alignment = align_up(TREE_BUFFER_ALIGNMENT, wgpu::COPY_BUFFER_ALIGNMENT as u32);
		let voxel_alignment = align_up(VOXEL_BUFFER_ALIGNMENT, wgpu::COPY_BUFFER_ALIGNMENT as u32);
		let binding_limit = raw_device.limits().max_storage_buffer_binding_size;

		let usage = wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST;
		Self {
			tree_slots: create_slots(&device, INITIAL_CAPACITY, usage),
			voxel_slots: create_slots(&device, INITIAL_CAPACITY, usage),
			tree_capacity: INITIAL_CAPACITY,
			voxel_capacity: INITIAL_CAPACITY,
			binding_limit,
			tree_alignment,
			voxel_alignment,
			current: 0,
			offsets: HashMap::new(),
			tree_slot_entries: create_slot_entry_lists(),
			tree_slot_entry_indices: create_slot_entry_index_maps(),
			voxel_slot_entries: create_slot_entry_lists(),
			voxel_slot_entry_indices: create_slot_entry_index_maps(),
			device,
			queue,
			usage,
		}
	}
}

impl ResidencyBuffers {
	/// Max bytes a slot may hold and still be bindable as a storage buffer.
	pub fn binding_limit(&self) -> u64 {
		self.binding_limit
	}

	pub fn tree_alignment(&self) -> u32 {
		self.tree_alignment
	}

	pub fn voxel_alignment(&self) -> u32 {
		self.voxel_alignment
	}

	/// Copy the resident sub-grids into the next slot and publish it. Caller
	/// trims `resident` to [`Self::binding_limit`] and provides it in a stable
	/// order.
	pub fn upload(&mut self, world_gpu: &WorldGpuData, resident: &[ResidentVoxels]) {
		self.current = (self.current + 1) % SLOTS;

		let mut tree_entries = Vec::with_capacity(resident.len());
		let mut voxel_entries = Vec::with_capacity(resident.len());
		for item in resident {
			let (Some(tree_held), Some(voxel_held)) = (
				world_gpu.packed_64_tree_dynamic_buffer.held_buffer(item.tree_id),
				world_gpu.packed_voxel_data_dynamic_buffer.held_buffer(item.voxels_id),
			) else {
				continue;
			};

			tree_entries.push(PendingUpload {
				entity: item.entity,
				generation: item.generation,
				src_offset: tree_held.offset(),
				size: tree_held.size(),
			});
			voxel_entries.push(PendingUpload {
				entity: item.entity,
				generation: item.generation,
				src_offset: voxel_held.offset(),
				size: voxel_held.size(),
			});
		}

		tree_entries.sort_unstable_by_key(|entry| entry.src_offset);
		voxel_entries.sort_unstable_by_key(|entry| entry.src_offset);

		let mut tree_plan = plan_residency(
			&self.tree_slot_entries[self.current],
			&self.tree_slot_entry_indices[self.current],
			&tree_entries,
			self.tree_alignment,
			COMPACTION_MOVE_CALL_BUDGET,
		);
		if tree_plan.required_capacity > self.binding_limit {
			tree_plan = plan_residency(&[], &HashMap::new(), &tree_entries, self.tree_alignment, 0);
		}
		if tree_plan.required_capacity > self.tree_capacity {
			self.ensure_tree_capacity(tree_plan.required_capacity);
			tree_plan = plan_residency(
				&self.tree_slot_entries[self.current],
				&self.tree_slot_entry_indices[self.current],
				&tree_entries,
				self.tree_alignment,
				COMPACTION_MOVE_CALL_BUDGET,
			);
		}

		let mut voxel_plan = plan_residency(
			&self.voxel_slot_entries[self.current],
			&self.voxel_slot_entry_indices[self.current],
			&voxel_entries,
			self.voxel_alignment,
			COMPACTION_MOVE_CALL_BUDGET,
		);
		if voxel_plan.required_capacity > self.binding_limit {
			voxel_plan = plan_residency(&[], &HashMap::new(), &voxel_entries, self.voxel_alignment, 0);
		}
		if voxel_plan.required_capacity > self.voxel_capacity {
			self.ensure_voxel_capacity(voxel_plan.required_capacity);
			voxel_plan = plan_residency(
				&self.voxel_slot_entries[self.current],
				&self.voxel_slot_entry_indices[self.current],
				&voxel_entries,
				self.voxel_alignment,
				COMPACTION_MOVE_CALL_BUDGET,
			);
		}

		if !tree_plan.copy_regions.is_empty() || !voxel_plan.copy_regions.is_empty() {
			let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("residency_pack") });
			let src_tree = world_gpu.packed_64_tree_dynamic_buffer.buffer();
			let src_voxel = world_gpu.packed_voxel_data_dynamic_buffer.buffer();
			let dst_tree = &self.tree_slots[self.current];
			let dst_voxel = &self.voxel_slots[self.current];
			record_copy_runs(&mut encoder, src_tree, dst_tree, &tree_plan.copy_regions);
			record_copy_runs(&mut encoder, src_voxel, dst_voxel, &voxel_plan.copy_regions);
			self.queue.submit(std::iter::once(encoder.finish()));
		}

		self.tree_slot_entries[self.current] = tree_plan.slot_entries;
		self.tree_slot_entry_indices[self.current] = self.tree_slot_entries[self.current]
			.iter()
			.enumerate()
			.map(|(index, entry)| (entry.entity, index))
			.collect();
		self.voxel_slot_entries[self.current] = voxel_plan.slot_entries;
		self.voxel_slot_entry_indices[self.current] = self.voxel_slot_entries[self.current]
			.iter()
			.enumerate()
			.map(|(index, entry)| (entry.entity, index))
			.collect();
		self.offsets = resident
			.iter()
			.filter_map(|item| {
				Some((
					item.entity,
					(*tree_plan.offsets.get(&item.entity)?, *voxel_plan.offsets.get(&item.entity)?),
				))
			})
			.collect();
	}

	fn ensure_tree_capacity(&mut self, bytes: u64) {
		if bytes > self.tree_capacity {
			self.tree_capacity = bytes.next_power_of_two().max(INITIAL_CAPACITY).min(self.binding_limit);
			self.tree_slots = create_slots(&self.device, self.tree_capacity, self.usage);
			self.tree_slot_entries = create_slot_entry_lists();
			self.tree_slot_entry_indices = create_slot_entry_index_maps();
		}
	}

	fn ensure_voxel_capacity(&mut self, bytes: u64) {
		if bytes > self.voxel_capacity {
			self.voxel_capacity = bytes.next_power_of_two().max(INITIAL_CAPACITY).min(self.binding_limit);
			self.voxel_slots = create_slots(&self.device, self.voxel_capacity, self.usage);
			self.voxel_slot_entries = create_slot_entry_lists();
			self.voxel_slot_entry_indices = create_slot_entry_index_maps();
		}
	}

	pub fn offsets(&self) -> &HashMap<Entity, (u32, u32)> {
		&self.offsets
	}

	pub fn tree_buffer(&self) -> &GpuBuffer {
		&self.tree_slots[self.current]
	}

	pub fn voxel_buffer(&self) -> &GpuBuffer {
		&self.voxel_slots[self.current]
	}
}
