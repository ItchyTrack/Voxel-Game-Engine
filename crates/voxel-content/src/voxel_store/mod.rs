mod edit_commands;
mod grid_store;
mod voxel_store_source;

pub use edit_commands::VoxelEditCommands;
pub use grid_store::GridStore;
pub use voxel_store_source::{VoxelStoreSource, complete_voxel_store_acquisitions};

#[derive(Default)]
pub struct VoxelStoreSourcePlugin;

impl Plugin for VoxelStoreSourcePlugin {
	fn build(&self, app: &mut App) {
		app.register_voxel_source(VoxelStoreSource::default())
			.add_systems(PreUpdate, complete_voxel_store_acquisitions);
	}
}