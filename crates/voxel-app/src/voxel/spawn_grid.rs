use bevy::prelude::*;
use basic_voxel::BasicVoxel;
use voxel_content::{StreamingVoxels, VoxelStoreSource};
use voxel_data::grid::Grid;
use voxel_lightyear::ReplicateVoxels;
use voxel_sources::edit::GridEditIdManager;
use voxel_streaming::bounds_of_chunks;

pub fn spawn_grid(
	commands: &mut Commands,
	store: &mut VoxelStoreSource,
	parent: Option<Entity>,
	transform: Transform,
	voxels: StreamingVoxels,
	extra: impl Bundle,
) {
	let child = commands
		.spawn((transform, Grid::new::<BasicVoxel>(), GridEditIdManager::default(), ReplicateVoxels, extra))
		.id();
	if let Some(parent) = parent { commands.entity(parent).add_child(child); }

	if let Some(bounds) = bounds_of_chunks(voxels.chunk_positions()) {
		commands.entity(child).insert(bounds);
	}
	store.insert_chunk_data(child, voxels.into_chunk_data());
}
