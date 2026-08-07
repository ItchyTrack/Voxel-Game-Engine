use std::collections::HashMap;

use bevy::ecs::entity::Entity;
use bevy::ecs::resource::Resource;
use bevy::ecs::world::FromWorld;
use bevy::render::renderer::{RenderDevice, RenderQueue, WgpuWrapper};

use crate::incoming_ray_directions::IncomingRayDirections;
use voxel_gpu::{AllocationId, residency_packing::{plan_residency, CopyRegion, PendingUpload, SlotEntry}};
use crate::gpu_data::{RayGpuBuffers, TREE_BUFFER_ALIGNMENT, VOXEL_BUFFER_ALIGNMENT};

type GpuBuffer = WgpuWrapper<wgpu::Buffer>;
type GpuDevice = WgpuWrapper<wgpu::Device>;
type GpuQueue = WgpuWrapper<wgpu::Queue>;

const INITIAL_CAPACITY: u64 = 1 << 20;
const COMPACTION_MOVE_CALL_BUDGET: usize = 64;

fn align_up(value: u32, alignment: u32) -> u32 {
	value.next_multiple_of(alignment)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResidencyDirections {
	/// The complete, non-directionally-culled voxel data is resident.
	Unculled,
	/// Only the specified incoming directions are resident.
	Culled(IncomingRayDirections),
}

/// Render-only compact copy of the ray tiles needed this frame. The render world
/// binds this instead of the long-lived [`RayWorldGpuData`] buffers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ResidentVoxels {
	pub entity: Entity,
	pub tree: AllocationId,
	pub voxels: AllocationId,
	pub generation: u64,
	pub loaded_directions: ResidencyDirections,
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
	tree_buffer: GpuBuffer,
	voxel_buffer: GpuBuffer,
	tree_capacity: u64,
	voxel_capacity: u64,
	binding_limit: u64,
	offsets: HashMap<Entity, (u32, u32)>,
	loaded_directions: HashMap<Entity, ResidencyDirections>,
	tree_entries: Vec<SlotEntry>,
	tree_entry_indices: HashMap<Entity, usize>,
	voxel_entries: Vec<SlotEntry>,
	voxel_entry_indices: HashMap<Entity, usize>,
}

fn create_buffer(device: &GpuDevice, capacity: u64, usage: wgpu::BufferUsages) -> GpuBuffer {
	WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
		label: Some("residency_buffer"),
		size: capacity,
		usage,
		mapped_at_creation: false,
	}))
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
			tree_buffer: create_buffer(&device, INITIAL_CAPACITY, usage),
			voxel_buffer: create_buffer(&device, INITIAL_CAPACITY, usage),
			tree_capacity: INITIAL_CAPACITY,
			voxel_capacity: INITIAL_CAPACITY,
			binding_limit,
			tree_alignment,
			voxel_alignment,
			offsets: HashMap::new(),
			loaded_directions: HashMap::new(),
			tree_entries: Vec::new(),
			tree_entry_indices: HashMap::new(),
			voxel_entries: Vec::new(),
			voxel_entry_indices: HashMap::new(),
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

	/// Copy the resident sub-grids into the residency slot and publish it. Caller
	/// trims `resident` to [`Self::binding_limit`] and provides it in a stable
	/// order.
	pub fn upload(&mut self, world_gpu: &RayGpuBuffers, resident: &[ResidentVoxels]) {
		let mut tree_entries = Vec::with_capacity(resident.len());
		let mut voxel_entries = Vec::with_capacity(resident.len());
		for item in resident {
			let (Some(tree_held), Some(voxel_held)) = (
				world_gpu.trees.held_buffer(item.tree),
				world_gpu.voxels.held_buffer(item.voxels),
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
			&self.tree_entries,
			&self.tree_entry_indices,
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
				&self.tree_entries,
				&self.tree_entry_indices,
				&tree_entries,
				self.tree_alignment,
				COMPACTION_MOVE_CALL_BUDGET,
			);
		}

		let mut voxel_plan = plan_residency(
			&self.voxel_entries,
			&self.voxel_entry_indices,
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
				&self.voxel_entries,
				&self.voxel_entry_indices,
				&voxel_entries,
				self.voxel_alignment,
				COMPACTION_MOVE_CALL_BUDGET,
			);
		}

		if !tree_plan.copy_regions.is_empty() || !voxel_plan.copy_regions.is_empty() {
			let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("residency_pack") });
			let src_tree = world_gpu.trees.buffer();
			let src_voxel = world_gpu.voxels.buffer();
			let dst_tree = &self.tree_buffer;
			let dst_voxel = &self.voxel_buffer;
			record_copy_runs(&mut encoder, src_tree, dst_tree, &tree_plan.copy_regions);
			record_copy_runs(&mut encoder, src_voxel, dst_voxel, &voxel_plan.copy_regions);
			self.queue.submit(std::iter::once(encoder.finish()));
		}

		self.tree_entries = tree_plan.slot_entries;
		self.tree_entry_indices = self.tree_entries
			.iter()
			.enumerate()
			.map(|(index, entry)| (entry.entity, index))
			.collect();
		self.voxel_entries = voxel_plan.slot_entries;
		self.voxel_entry_indices = self.voxel_entries
			.iter()
			.enumerate()
			.map(|(index, entry)| (entry.entity, index))
			.collect();
		self.offsets.clear();
		self.loaded_directions.clear();
		self.offsets.reserve(resident.len());
		self.loaded_directions.reserve(resident.len());
		for item in resident {
			let (Some(&tree_offset), Some(&voxel_offset)) = (
				tree_plan.offsets.get(&item.entity),
				voxel_plan.offsets.get(&item.entity),
			) else {
				continue;
			};

			self.offsets.insert(item.entity, (tree_offset, voxel_offset));
			self.loaded_directions.insert(item.entity, item.loaded_directions);
		}
	}

	fn ensure_tree_capacity(&mut self, bytes: u64) {
		if bytes > self.tree_capacity {
			self.tree_capacity = bytes.next_power_of_two().max(INITIAL_CAPACITY).min(self.binding_limit);
			self.tree_buffer = create_buffer(&self.device, self.tree_capacity, self.usage);
			self.tree_entries.clear();
			self.tree_entry_indices.clear();
		}
	}

	fn ensure_voxel_capacity(&mut self, bytes: u64) {
		if bytes > self.voxel_capacity {
			self.voxel_capacity = bytes.next_power_of_two().max(INITIAL_CAPACITY).min(self.binding_limit);
			self.voxel_buffer = create_buffer(&self.device, self.voxel_capacity, self.usage);
			self.voxel_entries.clear();
			self.voxel_entry_indices.clear();
		}
	}

	pub fn offsets(&self) -> &HashMap<Entity, (u32, u32)> {
		&self.offsets
	}

	pub fn loaded_directions(&self) -> &HashMap<Entity, ResidencyDirections> {
		&self.loaded_directions
	}

	pub fn tree_buffer(&self) -> &GpuBuffer {
		&self.tree_buffer
	}

	pub fn voxel_buffer(&self) -> &GpuBuffer {
		&self.voxel_buffer
	}
}
