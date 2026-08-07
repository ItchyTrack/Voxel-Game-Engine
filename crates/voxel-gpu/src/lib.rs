pub mod packed_dynamic_buffer;
pub mod residency_packing;
pub mod shader_compiler;
pub mod shader_sources;
pub mod voxel_color;

use bevy::{prelude::*, render::extract_resource::ExtractResourcePlugin};

pub use packed_dynamic_buffer::{AllocationId, PackedBufferAllocation};
pub use voxel_color::{VoxelGpuAppExt, VoxelGpuBlockEncoder, VoxelGpuData, VoxelGpuDataReaders, VoxelGpuNodeEntry, VoxelShaderRegistration};

/// Shared GPU helpers and voxel shader-data registration. Renderer-specific
/// allocations, tile data, and upload paths live in the renderer crates.
#[derive(Default)]
pub struct GpuVoxelDataPlugin;

impl Plugin for GpuVoxelDataPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<VoxelGpuDataReaders>()
			.add_plugins(ExtractResourcePlugin::<VoxelGpuDataReaders>::default());
	}
}
