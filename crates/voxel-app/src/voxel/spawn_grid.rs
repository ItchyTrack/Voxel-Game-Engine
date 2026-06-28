use bevy::prelude::*;
use voxel_content::{StreamingVoxels, VoxelStoreSource};
use voxel_data::grid::Grid;
use voxel_edit::GridEdits;
use voxel_lightyear::ReplicateVoxels;
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
		.spawn((transform, Grid::new(), GridEdits::default(), ReplicateVoxels, extra))
		.id();
	if parent.is_some() { commands.entity(parent.unwrap()).add_child(child); }

	let mut streaming = GridStreaming::default();
	for chunk in voxels.chunk_positions() {
		streaming.presence_mut().mark_present(chunk);
	}
	store.insert_chunk_data(child, voxels.into_chunk_data());
	commands.entity(child).insert(streaming);
}
