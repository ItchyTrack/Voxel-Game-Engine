use std::collections::HashMap;

use bevy::ecs::entity::Entity;
use bevy::ecs::resource::Resource;
use bevy::ecs::world::FromWorld;
use bevy::render::renderer::{RenderDevice, RenderQueue, WgpuWrapper};

use voxel_gpu::packed_residency_buffer::{PackedResidencyBuffer, upload_packed_residency_pair};
use voxel_gpu::residency_packing::PendingUpload;
use voxel_gpu::AllocationId;

use crate::gpu_data::{RayGpuBuffers, TREE_BUFFER_ALIGNMENT, VOXEL_BUFFER_ALIGNMENT};
use crate::incoming_ray_directions::IncomingRayDirections;

type GpuBuffer = WgpuWrapper<wgpu::Buffer>;

const SLOTS: usize = 1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResidencyDirections {
	/// The complete, non-directionally-culled voxel data is resident.
	Unculled,
	/// Only the specified incoming directions are resident.
	Culled(IncomingRayDirections),
}

/// Render-only compact copy of the ray tiles needed this frame. The render world
/// binds this instead of the long-lived [`crate::gpu_data::RayWorldGpuData`] buffers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ResidentVoxels {
	pub entity: Entity,
	pub tree: AllocationId,
	pub voxels: AllocationId,
	pub generation: u64,
	pub loaded_directions: ResidencyDirections,
}

#[derive(Resource)]
pub struct ResidencyBuffers {
	trees: PackedResidencyBuffer<SLOTS>,
	voxels: PackedResidencyBuffer<SLOTS>,
	offsets: HashMap<Entity, (u32, u32)>,
	loaded_directions: HashMap<Entity, ResidencyDirections>,
}

impl FromWorld for ResidencyBuffers {
	fn from_world(world: &mut bevy::ecs::world::World) -> Self {
		let render_device = world.resource::<RenderDevice>();
		let render_queue = world.resource::<RenderQueue>();
		Self {
			trees: PackedResidencyBuffer::new(
				render_device,
				render_queue,
				TREE_BUFFER_ALIGNMENT,
				"residency_buffer",
			),
			voxels: PackedResidencyBuffer::new(
				render_device,
				render_queue,
				VOXEL_BUFFER_ALIGNMENT,
				"residency_buffer",
			),
			offsets: HashMap::new(),
			loaded_directions: HashMap::new(),
		}
	}
}

impl ResidencyBuffers {
	/// Max bytes a slot may hold and still be bindable as a storage buffer.
	pub fn binding_limit(&self) -> u64 { self.trees.binding_limit() }
	pub fn tree_alignment(&self) -> u32 { self.trees.alignment() }
	pub fn voxel_alignment(&self) -> u32 { self.voxels.alignment() }

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
		upload_packed_residency_pair(
			&mut self.trees,
			&world_gpu.trees,
			tree_entries,
			&mut self.voxels,
			&world_gpu.voxels,
			voxel_entries,
			"residency_pack",
		);

		self.offsets.clear();
		self.loaded_directions.clear();
		self.offsets.reserve(resident.len());
		self.loaded_directions.reserve(resident.len());
		for item in resident {
			let (Some(&tree_offset), Some(&voxel_offset)) = (
				self.trees.offsets().get(&item.entity),
				self.voxels.offsets().get(&item.entity),
			) else {
				continue;
			};
			self.offsets.insert(item.entity, (tree_offset, voxel_offset));
			self.loaded_directions.insert(item.entity, item.loaded_directions);
		}
	}

	pub fn offsets(&self) -> &HashMap<Entity, (u32, u32)> { &self.offsets }
	pub fn loaded_directions(&self) -> &HashMap<Entity, ResidencyDirections> { &self.loaded_directions }
	pub fn tree_buffer(&self) -> &GpuBuffer { self.trees.buffer() }
	pub fn voxel_buffer(&self) -> &GpuBuffer { self.voxels.buffer() }
}
