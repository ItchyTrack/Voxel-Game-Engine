use bevy::ecs::message::Messages;
use voxel_streaming::systems;
use bevy::prelude::*;
use voxel_data::{grid::Grid, voxels::{VoxelTypeId, VoxelTypeInfo}};
use voxel_edit::GridEdits;
use voxel_sources::{ChunkChangeKind, ChunkChanged, ChunkLoaded};
use voxel_streaming::{ChunkAvailabilityChanged, ChunkLoadResolved, ChunkState, GridStreaming};

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
fn stale_inflight_chunk_result_is_ignored() {
	let mut app = App::new();
	app.add_message::<ChunkLoadResolved>()
		.add_message::<ChunkAvailabilityChanged>()
		.add_message::<ChunkLoaded>()
		.add_message::<ChunkChanged>()
		.add_systems(Update, (systems::apply_source_events, systems::receive_results).chain());

	let grid = app.world_mut().spawn((Grid::new_with_type(test_type_info()), GridStreaming::default(), GridEdits::default())).id();
	let chunk = IVec3::new(1, 2, 3);
	{
		let mut streaming = app.world_mut().get_mut::<GridStreaming>(grid).unwrap();
		streaming.mark_present(chunk);
		streaming.presence_mut().set_state(chunk, ChunkState::InFlight);
	}

	app.world_mut().resource_mut::<Messages<ChunkChanged>>().write(ChunkChanged {
		grid,
		min: chunk,
		size: IVec3::ONE,
		kind: ChunkChangeKind::Changed { generation: 5 },
		from_save: false,
	});
	app.world_mut().resource_mut::<Messages<ChunkLoaded>>().write(ChunkLoaded { grid, chunk, generation: 4, voxels: None });
	app.update();

	let streaming = app.world().get::<GridStreaming>(grid).unwrap();
	assert_eq!(streaming.state(chunk), Some(ChunkState::ExternalDirty));

	let messages = app.world().resource::<Messages<ChunkLoadResolved>>();
	let resolved: Vec<_> = messages.iter_current_update_messages().copied().collect();
	assert!(resolved.is_empty());
}

#[test]
fn fresh_inflight_chunk_result_is_applied() {
	let mut app = App::new();
	app.add_message::<ChunkLoadResolved>()
		.add_message::<ChunkAvailabilityChanged>()
		.add_message::<ChunkLoaded>()
		.add_message::<ChunkChanged>()
		.add_systems(Update, (systems::apply_source_events, systems::receive_results).chain());

	let grid = app.world_mut().spawn((Grid::new_with_type(test_type_info()), GridStreaming::default(), GridEdits::default())).id();
	let chunk = IVec3::new(4, 5, 6);
	{
		let mut streaming = app.world_mut().get_mut::<GridStreaming>(grid).unwrap();
		streaming.mark_present(chunk);
		streaming.presence_mut().set_state(chunk, ChunkState::InFlight);
	}

	app.world_mut().resource_mut::<Messages<ChunkChanged>>().write(ChunkChanged {
		grid,
		min: chunk,
		size: IVec3::ONE,
		kind: ChunkChangeKind::Changed { generation: 5 },
		from_save: false,
	});
	app.world_mut().resource_mut::<Messages<ChunkLoaded>>().write(ChunkLoaded { grid, chunk, generation: 5, voxels: None });
	app.update();

	let streaming = app.world().get::<GridStreaming>(grid).unwrap();
	assert_eq!(streaming.state(chunk), None);

	let messages = app.world().resource::<Messages<ChunkLoadResolved>>();
	let resolved: Vec<_> = messages.iter_current_update_messages().copied().collect();
	assert_eq!(resolved, vec![ChunkLoadResolved { grid, chunk, visible: false }]);
}
