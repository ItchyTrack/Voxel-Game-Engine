pub mod gpu_grid_tree;
pub mod packed_buffer;
pub mod packed_dynamic_buffer;
pub mod world_gpu_data;
pub mod sub_grid_gpu_state;
pub mod lod_voxels;
pub mod residency;
pub mod upload;

use bevy::{prelude::*, render::RenderApp};

use crate::world_gpu_data::WorldGpuData;

pub use lod_voxels::LodVoxels;
pub use sub_grid_gpu_state::SubGridGpuState;

#[doc(hidden)]
pub use bevy as __bevy;

#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GpuUploadSet {
	Clear,
	Upload,
}

#[derive(Default)]
pub struct GpuVoxelDataPlugin;

impl Plugin for GpuVoxelDataPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<upload::InFlightUploads>()
			.configure_sets(Update, (GpuUploadSet::Clear, GpuUploadSet::Upload).chain())
			.add_systems(Update, upload::flag_changed_sub_grids.in_set(GpuUploadSet::Clear))
			.add_systems(Update, (upload::manage_gpu_uploads, upload::manage_lod_uploads).in_set(GpuUploadSet::Upload));
	}

	fn finish(&self, app: &mut App) {
		app.init_resource::<WorldGpuData>();

		let Some(render_app) = app.get_sub_app_mut(RenderApp) else { return };
		render_app.init_resource::<residency::ResidencyBuffers>();
	}
}
