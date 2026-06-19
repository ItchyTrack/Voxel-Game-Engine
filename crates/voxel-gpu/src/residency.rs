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

	/// Copy the resident sub-grids `(entity, tree_id, voxels_id)` into the next
	/// slot and publish it. Caller trims `resident` to [`Self::binding_limit`].
	pub fn upload(&mut self, world_gpu: &WorldGpuData, resident: &[(Entity, u32, u32)]) {
		self.current = (self.current + 1) % SLOTS;

		let mut tree_bytes = 0u64;
		let mut voxel_bytes = 0u64;
		for (_, tree_id, voxels_id) in resident {
			if let Some(h) = world_gpu.packed_64_tree_dynamic_buffer.held_buffer(*tree_id) {
				tree_bytes += align_up(h.size(), self.tree_alignment) as u64;
			}
			if let Some(h) = world_gpu.packed_voxel_data_dynamic_buffer.held_buffer(*voxels_id) {
				voxel_bytes += align_up(h.size(), self.voxel_alignment) as u64;
			}
		}
		self.ensure_capacity(tree_bytes, voxel_bytes);

		let mut offsets = HashMap::with_capacity(resident.len());
		let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("residency_pack") });

		let src_tree = world_gpu.packed_64_tree_dynamic_buffer.buffer();
		let src_voxel = world_gpu.packed_voxel_data_dynamic_buffer.buffer();
		let dst_tree = &self.tree_slots[self.current];
		let dst_voxel = &self.voxel_slots[self.current];

		let mut tree_off = 0u32;
		let mut voxel_off = 0u32;
		for (entity, tree_id, voxels_id) in resident {
			let (Some(tree_held), Some(voxel_held)) = (
				world_gpu.packed_64_tree_dynamic_buffer.held_buffer(*tree_id),
				world_gpu.packed_voxel_data_dynamic_buffer.held_buffer(*voxels_id),
			) else {
				continue;
			};

			// copy_buffer_to_buffer requires 4-byte-aligned sizes.
			let tree_copy = align_up(tree_held.size(), wgpu::COPY_BUFFER_ALIGNMENT as u32) as u64;
			let voxel_copy = align_up(voxel_held.size(), wgpu::COPY_BUFFER_ALIGNMENT as u32) as u64;
			encoder.copy_buffer_to_buffer(src_tree, tree_held.offset() as u64, dst_tree, tree_off as u64, tree_copy);
			encoder.copy_buffer_to_buffer(src_voxel, voxel_held.offset() as u64, dst_voxel, voxel_off as u64, voxel_copy);

			offsets.insert(*entity, (tree_off, voxel_off));
			tree_off += align_up(tree_held.size(), self.tree_alignment);
			voxel_off += align_up(voxel_held.size(), self.voxel_alignment);
		}

		self.queue.submit(std::iter::once(encoder.finish()));
		self.offsets = offsets;
	}

	fn ensure_capacity(&mut self, tree_bytes: u64, voxel_bytes: u64) {
		if tree_bytes > self.tree_capacity {
			self.tree_capacity = tree_bytes.next_power_of_two().max(INITIAL_CAPACITY).min(self.binding_limit);
			self.tree_slots = create_slots(&self.device, self.tree_capacity, self.usage);
		}
		if voxel_bytes > self.voxel_capacity {
			self.voxel_capacity = voxel_bytes.next_power_of_two().max(INITIAL_CAPACITY).min(self.binding_limit);
			self.voxel_slots = create_slots(&self.device, self.voxel_capacity, self.usage);
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
