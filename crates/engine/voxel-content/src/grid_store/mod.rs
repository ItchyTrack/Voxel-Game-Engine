mod edit_api;
mod grid_store;
mod voxel_store_source;

pub use edit_api::GridStoreEditApi;
pub use grid_store::GridStore;
pub use voxel_store_source::{
	VoxelStoreSource, complete_voxel_store_acquisitions,
};

use bevy::prelude::*;
use voxel_mass::{SourceMassPlugin, VoxelMassReaders};
use voxel_sources::VoxelSourcesAppExt;

#[derive(Default)]
pub struct VoxelStoreSourcePlugin;

impl Plugin for VoxelStoreSourcePlugin {
	fn build(&self, app: &mut App) {
		let mass_readers = app.world().resource::<VoxelMassReaders>().clone();
		app.register_voxel_source(VoxelStoreSource::new(mass_readers))
			.add_systems(PreUpdate, complete_voxel_store_acquisitions)
			.add_plugins(SourceMassPlugin::<VoxelStoreSource>::default());
	}
}
