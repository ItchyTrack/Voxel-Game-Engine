use bevy::ecs::message::Messages;
use bevy::prelude::*;
use voxel_data::{grid::Grid, grid_tree::NonZeroVoxelRegion, voxels::{Voxel, VoxelTypeId, VoxelTypeInfo}};
use voxel_edit::GridEdit;
use tile_data::{ChunkRegion, chunk_origin};
use voxel_sources::ChunkLoaded;
use voxel_streaming::systems;
use voxel_streaming::{
	apply_grid_edits, AuthoritativeGridCommand, ChunkAvailabilityChanged, ChunkLoadResolved, ChunkState,
	GridEdits, GridStreaming,
};

fn test_type_info() -> VoxelTypeInfo {
	VoxelTypeInfo { id: VoxelTypeId(1), size_bytes: 1 }
}

#[test]
fn present_chunk_that_loads_none_emits_not_visible_resolution() {
	let mut app = App::new();
	app.add_message::<ChunkLoadResolved>()
		.add_message::<ChunkAvailabilityChanged>()
		.add_message::<ChunkLoaded>()
		.add_systems(Update, systems::receive_results);

	let grid = app.world_mut().spawn((Grid::new_with_type(test_type_info()), GridStreaming::default(), GridEdits::default())).id();
	let chunk = IVec3::new(3, 0, -2);
	{
		let mut streaming = app.world_mut().get_mut::<GridStreaming>(grid).unwrap();
		streaming.mark_present(chunk);
		streaming.presence_mut().set_state(chunk, ChunkState::InFlight);
	}

	app.world_mut().resource_mut::<Messages<ChunkLoaded>>().write(ChunkLoaded { grid, chunk, generation: 0, voxels: None });
	app.update();

	let messages = app.world().resource::<Messages<ChunkLoadResolved>>();
	let resolved: Vec<_> = messages.iter_current_update_messages().copied().collect();
	assert_eq!(resolved, vec![ChunkLoadResolved { grid, chunk, visible: false }]);
}

#[test]
fn authoritative_region_command_applies_loaded_chunks_and_waits_on_unloaded_chunks() {
	let mut app = App::new();
	app.add_message::<AuthoritativeGridCommand>()
		.add_message::<ChunkLoadResolved>()
		.add_message::<ChunkAvailabilityChanged>()
		.add_message::<ChunkLoaded>()
		.add_systems(Update, (
			systems::materialize_authoritative_commands,
			apply_grid_edits,
			systems::receive_results,
		).chain());

	let grid_entity = app.world_mut().spawn((Grid::new_with_type(test_type_info()), GridStreaming::default(), GridEdits::default())).id();
	let loaded_chunk = IVec3::ZERO;
	let unloaded_chunk = IVec3::X;
	{
		let mut streaming = app.world_mut().get_mut::<GridStreaming>(grid_entity).unwrap();
		streaming.mark_present(loaded_chunk);
		streaming.presence_mut().set_state(loaded_chunk, ChunkState::Loaded);
		streaming.mark_present(unloaded_chunk);
		streaming.presence_mut().set_state(unloaded_chunk, ChunkState::InFlight);
	}

	let first = chunk_origin(unloaded_chunk) - IVec3::X;
	let second = chunk_origin(unloaded_chunk);
	let region = NonZeroVoxelRegion::new(first, UVec3::new(2, 1, 1)).unwrap();
	app.world_mut().resource_mut::<Messages<AuthoritativeGridCommand>>().write(AuthoritativeGridCommand {
		grid: grid_entity,
		region: ChunkRegion::new(loaded_chunk, UVec3::new(2, 1, 1)),
		stream_sequence: 1,
		generation: 1,
		edit: GridEdit::AddArea { region, voxel: Voxel::new(test_type_info().id, [7]) },
	});
	app.update();

	let grid = app.world().get::<Grid>(grid_entity).unwrap();
	assert!(grid.voxel(&first).is_some());
	assert!(grid.voxel(&second).is_none());

	app.world_mut().resource_mut::<Messages<ChunkLoaded>>().write(ChunkLoaded {
		grid: grid_entity,
		chunk: unloaded_chunk,
		generation: 0,
		voxels: None,
	});
	app.update();

	let grid = app.world().get::<Grid>(grid_entity).unwrap();
	assert!(grid.voxel(&first).is_some());
	assert!(grid.voxel(&second).is_some());
}
