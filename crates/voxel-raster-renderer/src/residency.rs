use bevy::ecs::entity::Entity;
use bevy::ecs::resource::Resource;
use bevy::ecs::world::FromWorld;
use bevy::render::renderer::{RenderDevice, RenderQueue, WgpuWrapper};

use voxel_gpu::packed_residency_buffer::{PackedResidencyBuffer, upload_packed_residency_pair};
use voxel_gpu::residency_packing::PendingUpload;
use voxel_gpu::AllocationId;

use crate::gpu_data::{FACE_BUFFER_ALIGNMENT, PALETTE_BUFFER_ALIGNMENT, RasterGpuBuffers};

type GpuBuffer = WgpuWrapper<wgpu::Buffer>;

const SLOTS: usize = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct ResidentRasterVoxels {
	pub entity: Entity,
	pub faces: AllocationId,
	pub palette: AllocationId,
	pub generation: u64,
}

#[derive(Resource)]
pub struct RasterResidencyBuffers {
	faces: PackedResidencyBuffer<SLOTS>,
	palettes: PackedResidencyBuffer<SLOTS>,
}

impl FromWorld for RasterResidencyBuffers {
	fn from_world(world: &mut bevy::ecs::world::World) -> Self {
		let render_device = world.resource::<RenderDevice>();
		let render_queue = world.resource::<RenderQueue>();
		Self {
			faces: PackedResidencyBuffer::new(
				render_device,
				render_queue,
				FACE_BUFFER_ALIGNMENT,
				"raster_residency_slot",
			),
			palettes: PackedResidencyBuffer::new(
				render_device,
				render_queue,
				PALETTE_BUFFER_ALIGNMENT,
				"raster_residency_slot",
			),
		}
	}
}

impl RasterResidencyBuffers {
	pub fn binding_limit(&self) -> u64 { self.faces.binding_limit() }
	pub fn face_alignment(&self) -> u32 { self.faces.alignment() }
	pub fn palette_alignment(&self) -> u32 { self.palettes.alignment() }

	pub fn upload(&mut self, world_gpu: &RasterGpuBuffers, resident: &[ResidentRasterVoxels]) {
		let mut face_entries = Vec::with_capacity(resident.len());
		let mut palette_entries = Vec::with_capacity(resident.len());
		for item in resident {
			let Some(face_held) = world_gpu.faces.held_buffer(item.faces) else { continue; };
			let Some(palette_held) = world_gpu.palettes.held_buffer(item.palette) else { continue; };
			face_entries.push(PendingUpload {
				entity: item.entity,
				generation: item.generation,
				src_offset: face_held.offset(),
				size: face_held.size(),
			});
			palette_entries.push(PendingUpload {
				entity: item.entity,
				generation: item.generation,
				src_offset: palette_held.offset(),
				size: palette_held.size(),
			});
		}
		upload_packed_residency_pair(
			&mut self.faces,
			&world_gpu.faces,
			face_entries,
			&mut self.palettes,
			&world_gpu.palettes,
			palette_entries,
			"raster_residency_pack",
		);
	}

	pub fn face_offsets(&self) -> &std::collections::HashMap<Entity, u32> { self.faces.offsets() }
	pub fn palette_offsets(&self) -> &std::collections::HashMap<Entity, u32> { self.palettes.offsets() }
	pub fn face_buffer(&self) -> &GpuBuffer { self.faces.buffer() }
	pub fn palette_buffer(&self) -> &GpuBuffer { self.palettes.buffer() }
}
