use std::collections::HashMap;

use bevy::ecs::entity::Entity;
use bevy::ecs::resource::Resource;
use bevy::ecs::world::FromWorld;
use bevy::render::renderer::{RenderDevice, RenderQueue, WgpuWrapper};

type GpuBuffer = WgpuWrapper<wgpu::Buffer>;
type GpuDevice = WgpuWrapper<wgpu::Device>;
type GpuQueue = WgpuWrapper<wgpu::Queue>;

use crate::world_gpu_data::{WorldGpuData, TREE_BUFFER_ALIGNMENT, VOXEL_BUFFER_ALIGNMENT};

// Pipelined rendering is one frame behind, so two slots keep the main world's
// build off the slot the render thread is reading.
const SLOTS: usize = 2;
const INITIAL_CAPACITY: u64 = 1 << 20;

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
	last_resident: Vec<ResidentVoxels>,
	tree_slot_entries: Vec<HashMap<Entity, SlotEntry>>,
	voxel_slot_entries: Vec<HashMap<Entity, SlotEntry>>,
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

fn create_slot_entry_maps() -> Vec<HashMap<Entity, SlotEntry>> {
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
		let binding_limit = raw_device.limits().max_storage_buffer_binding_size as u64;

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
			last_resident: Vec::new(),
			tree_slot_entries: create_slot_entry_maps(),
			voxel_slot_entries: create_slot_entry_maps(),
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
		if self.last_resident == resident {
			return;
		}

		self.current = (self.current + 1) % SLOTS;

		let mut tree_bytes = 0u64;
		let mut voxel_bytes = 0u64;
		let mut tree_entries = Vec::with_capacity(resident.len());
		let mut voxel_entries = Vec::with_capacity(resident.len());
		for item in resident {
			let (Some(tree_held), Some(voxel_held)) = (
				world_gpu.packed_64_tree_dynamic_buffer.held_buffer(item.tree_id),
				world_gpu.packed_voxel_data_dynamic_buffer.held_buffer(item.voxels_id),
			) else {
				continue;
			};

			tree_bytes += align_up(tree_held.size(), self.tree_alignment) as u64;
			voxel_bytes += align_up(voxel_held.size(), self.voxel_alignment) as u64;
			tree_entries.push(PendingCopy {
				entity: item.entity,
				generation: item.generation,
				src_offset: tree_held.offset(),
				size: tree_held.size(),
			});
			voxel_entries.push(PendingCopy {
				entity: item.entity,
				generation: item.generation,
				src_offset: voxel_held.offset(),
				size: voxel_held.size(),
			});
		}
		self.ensure_capacity(tree_bytes, voxel_bytes);

		tree_entries.sort_unstable_by_key(|entry| entry.src_offset);
		voxel_entries.sort_unstable_by_key(|entry| entry.src_offset);

		let mut tree_offsets = HashMap::with_capacity(tree_entries.len());
		let mut voxel_offsets = HashMap::with_capacity(voxel_entries.len());
		let mut tree_regions = Vec::with_capacity(tree_entries.len());
		let mut voxel_regions = Vec::with_capacity(voxel_entries.len());
		let mut next_tree_slot_entries = HashMap::with_capacity(tree_entries.len());
		let mut next_voxel_slot_entries = HashMap::with_capacity(voxel_entries.len());

		{
			let previous_tree_slot_entries = &self.tree_slot_entries[self.current];
			let mut tree_off = 0u32;
			for entry in tree_entries {
				let copy_size = align_up(entry.size, wgpu::COPY_BUFFER_ALIGNMENT as u32) as u64;
				let slot_entry = SlotEntry {
					generation: entry.generation,
					src_offset: entry.src_offset,
					size: entry.size,
					dst_offset: tree_off,
				};
				if previous_tree_slot_entries.get(&entry.entity).copied() != Some(slot_entry) {
					tree_regions.push(CopyRegion {
						src_offset: entry.src_offset as u64,
						dst_offset: tree_off as u64,
						size: copy_size,
					});
				}
				tree_offsets.insert(entry.entity, tree_off);
				next_tree_slot_entries.insert(entry.entity, slot_entry);
				tree_off += align_up(entry.size, self.tree_alignment);
			}
		}

		{
			let previous_voxel_slot_entries = &self.voxel_slot_entries[self.current];
			let mut voxel_off = 0u32;
			for entry in voxel_entries {
				let copy_size = align_up(entry.size, wgpu::COPY_BUFFER_ALIGNMENT as u32) as u64;
				let slot_entry = SlotEntry {
					generation: entry.generation,
					src_offset: entry.src_offset,
					size: entry.size,
					dst_offset: voxel_off,
				};
				if previous_voxel_slot_entries.get(&entry.entity).copied() != Some(slot_entry) {
					voxel_regions.push(CopyRegion {
						src_offset: entry.src_offset as u64,
						dst_offset: voxel_off as u64,
						size: copy_size,
					});
				}
				voxel_offsets.insert(entry.entity, voxel_off);
				next_voxel_slot_entries.insert(entry.entity, slot_entry);
				voxel_off += align_up(entry.size, self.voxel_alignment);
			}
		}

		let mut offsets = HashMap::with_capacity(resident.len());
		for item in resident {
			let (Some(&tree_off), Some(&voxel_off)) = (tree_offsets.get(&item.entity), voxel_offsets.get(&item.entity)) else {
				continue;
			};
			offsets.insert(item.entity, (tree_off, voxel_off));
		}

		if !tree_regions.is_empty() || !voxel_regions.is_empty() {
			let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("residency_pack") });
			let src_tree = world_gpu.packed_64_tree_dynamic_buffer.buffer();
			let src_voxel = world_gpu.packed_voxel_data_dynamic_buffer.buffer();
			let dst_tree = &self.tree_slots[self.current];
			let dst_voxel = &self.voxel_slots[self.current];
			record_copy_runs(&mut encoder, src_tree, dst_tree, &tree_regions);
			record_copy_runs(&mut encoder, src_voxel, dst_voxel, &voxel_regions);
			self.queue.submit(std::iter::once(encoder.finish()));
		}

		self.tree_slot_entries[self.current] = next_tree_slot_entries;
		self.voxel_slot_entries[self.current] = next_voxel_slot_entries;
		if offsets.len() == resident.len() {
			self.last_resident.clear();
			self.last_resident.extend_from_slice(resident);
		} else {
			self.last_resident.clear();
		}
		self.offsets = offsets;
	}

	fn ensure_capacity(&mut self, tree_bytes: u64, voxel_bytes: u64) {
		if tree_bytes > self.tree_capacity {
			self.tree_capacity = tree_bytes.next_power_of_two().max(INITIAL_CAPACITY).min(self.binding_limit);
			self.tree_slots = create_slots(&self.device, self.tree_capacity, self.usage);
			self.tree_slot_entries = create_slot_entry_maps();
		}
		if voxel_bytes > self.voxel_capacity {
			self.voxel_capacity = voxel_bytes.next_power_of_two().max(INITIAL_CAPACITY).min(self.binding_limit);
			self.voxel_slots = create_slots(&self.device, self.voxel_capacity, self.usage);
			self.voxel_slot_entries = create_slot_entry_maps();
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
