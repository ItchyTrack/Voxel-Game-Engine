use bevy::prelude::*;
use basic_voxel::BasicVoxel;
use voxel_content::{StreamingVoxels, VoxelStoreSource};
use voxel_data::grid::Grid;
use voxel_lightyear::ReplicateVoxels;
use voxel_sources::edit::GridEditIdManager;
use voxel_streaming::GridStreaming;

pub fn spawn_grid(
	commands: &mut Commands,
	store: &VoxelStoreSource,
	parent: Option<Entity>,
	transform: Transform,
	voxels: StreamingVoxels,
	extra: impl Bundle,
) {
	let child = commands
		.spawn((transform, Grid::new::<BasicVoxel>(), GridEditIdManager::default(), ReplicateVoxels, extra))
		.id();
	if let Some(parent) = parent { commands.entity(parent).add_child(child); }

	let mut streaming = GridStreaming::default();
	for chunk in voxels.chunk_positions() {
		streaming.mark_present(chunk);
	}
	store.insert_chunk_data(child, voxels.into_chunk_data());
	commands.entity(child).insert(streaming);
}
