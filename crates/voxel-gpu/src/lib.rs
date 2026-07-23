pub mod gpu_grid_tree;
pub mod gpu_raster_mesh;
pub mod packed_dynamic_buffer;
pub mod world_gpu_data;
pub mod voxel_gpu_state;
pub mod voxel_color;
pub mod lod_voxels;
pub mod residency;
pub mod residency_packing;
pub mod shader_compiler;
pub mod shader_sources;
pub mod incoming_ray_directions;
pub mod upload;

use bevy::{ecs::message::Message, prelude::*, render::{extract_resource::ExtractResourcePlugin, RenderApp}};

use crate::world_gpu_data::WorldGpuData;

pub use lod_voxels::LodVoxels;
pub use voxel_color::{VoxelGpuAppExt, VoxelGpuData, VoxelGpuDataReaders, VoxelShaderRegistration};
pub use voxel_gpu_state::{RasterGpuState, RayGpuState, SubGridPlacement, VoxelGpuBounds, VoxelGpuFormat, VoxelGpuState};

#[doc(hidden)]
pub use bevy as __bevy;

#[derive(Message, Clone, Copy, Debug, PartialEq, Eq)]
pub struct VoxelGpuUploadFinished {
	pub entity: Entity,
}

#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GpuUploadSet {
	Clear,
	Upload,
}

#[derive(Default)]
pub struct GpuVoxelDataPlugin;

impl Plugin for GpuVoxelDataPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<upload::InFlightRayUploads>()
			.init_resource::<upload::InFlightRasterUploads>()
			.init_resource::<VoxelGpuDataReaders>()
			.add_plugins(ExtractResourcePlugin::<VoxelGpuDataReaders>::default())
			.add_message::<VoxelGpuUploadFinished>()
			.configure_sets(Update, (GpuUploadSet::Clear, GpuUploadSet::Upload).chain())
			.add_systems(Update, (upload::clear_inactive_formats, upload::flag_changed_sub_grids).chain().in_set(GpuUploadSet::Clear))
			.add_systems(
				Update,
				(
					upload::manage_ray_gpu_uploads,
					upload::manage_ray_lod_uploads,
					upload::manage_raster_gpu_uploads,
					upload::manage_raster_lod_uploads,
				)
					.in_set(GpuUploadSet::Upload),
			);
	}

	fn finish(&self, app: &mut App) {
		app.init_resource::<WorldGpuData>();

		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return };
		render_app.init_resource::<residency::ResidencyBuffers>();
	}
}
