use std::sync::{Arc, OnceLock};

use bevy::ecs::message::Messages;
use bevy::prelude::*;
use tile_data::{NonZeroChunkRegion, chunk_origin};
use voxel_data::{grid::Grid, grid_tree::NonZeroVoxelRegion, voxels::{Voxel, VoxelTypeId, VoxelTypeInfo}};
use voxel_edit::GridEdit;
use voxel_sources::{
	ChunkSource, RequestId, SourceCoverage, SourceHandle, SourceManager, VoxelSourcesAppExt,
	VoxelSourcesPlugin,
};
use voxel_streaming::systems;
use voxel_streaming::{
	apply_grid_edits, AuthoritativeGridCommand, ChunkAvailabilityChanged, ChunkLoadResolved, ChunkState,
	GridEdits, GridStreaming,
};
use voxel_tasks::CancellationToken;

#[derive(Clone, Default)]
struct EmptySource(Arc<OnceLock<SourceHandle>>);

impl ChunkSource for EmptySource {
	fn init(&self, handle: SourceHandle) {
		let _ = self.0.set(handle);
	}

	fn request_voxels(
		&self,
		request_id: RequestId,
		_cancellation: &CancellationToken,
		_grid: Entity,
		_region: NonZeroChunkRegion,
		_lod: u8,
		_voxel_type: Option<VoxelTypeId>,
	) -> SourceCoverage {
		self.0.get().unwrap().voxels_loaded(request_id);
		SourceCoverage::All
	}

	fn request_presence(&self, request_id: RequestId, _cancellation: CancellationToken, _grid: Entity) {
		self.0.get().unwrap().presence_loaded(request_id);
	}

	fn take_ownership(&self, _grid: Entity, _region: NonZeroChunkRegion) {}
}

fn test_type_info() -> VoxelTypeInfo {
	VoxelTypeInfo { id: VoxelTypeId(1), size_bytes: 1 }
}

fn request_chunk(app: &mut App, grid: Entity, chunk: IVec3) {
	app.world_mut().resource_scope(|world, mut sources: Mut<SourceManager>| {
		world.get_mut::<GridStreaming>(grid).unwrap().fetch(&mut sources, grid, chunk);
	});
}

fn test_app() -> App {
	let mut app = App::new();
	app.add_plugins(VoxelSourcesPlugin);
	app.register_voxel_source(EmptySource::default());
	app
}

#[test]
fn present_chunk_that_loads_none_emits_not_visible_resolution() {
	let mut app = test_app();
	app.add_message::<ChunkLoadResolved>()
		.add_message::<ChunkAvailabilityChanged>()
		.add_systems(Update, systems::receive_results);

	let grid = app.world_mut().spawn((Grid::new_with_type(test_type_info()), GridStreaming::default(), GridEdits::default())).id();
	let chunk = IVec3::new(3, 0, -2);
	app.world_mut().get_mut::<GridStreaming>(grid).unwrap().mark_present(chunk);
	request_chunk(&mut app, grid, chunk);
	app.update();

	let messages = app.world().resource::<Messages<ChunkLoadResolved>>();
	let resolved: Vec<_> = messages.iter_current_update_messages().copied().collect();
	assert_eq!(resolved, vec![ChunkLoadResolved { grid, chunk, visible: false }]);
}

#[test]
fn authoritative_region_command_applies_loaded_chunks_and_waits_on_unloaded_chunks() {
	let mut app = test_app();
	app.add_message::<AuthoritativeGridCommand>()
		.add_message::<ChunkLoadResolved>()
		.add_message::<ChunkAvailabilityChanged>()
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
	}
	request_chunk(&mut app, grid_entity, unloaded_chunk);

	let first = chunk_origin(unloaded_chunk) - IVec3::X;
	let second = chunk_origin(unloaded_chunk);
	let region = NonZeroVoxelRegion::new(first, UVec3::new(2, 1, 1)).unwrap();
	app.world_mut().resource_mut::<Messages<AuthoritativeGridCommand>>().write(AuthoritativeGridCommand {
		grid: grid_entity,
		region: NonZeroChunkRegion::new(loaded_chunk, UVec3::new(2, 1, 1)).unwrap(),
		stream_sequence: 1,
		generation: 1,
		edit: GridEdit::AddArea { region, voxel: Voxel::new(test_type_info().id, [7]) },
	});
	app.update();

	let grid = app.world().get::<Grid>(grid_entity).unwrap();
	assert!(grid.voxel(&first).is_some());
	assert!(grid.voxel(&second).is_some());
}
