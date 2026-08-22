use std::sync::{Arc, OnceLock};

use bevy::ecs::message::Messages;
use bevy::prelude::*;
use tile_data::{NonZeroChunkRegion, chunk_origin};
use voxel_data::{grid::Grid, grid_tree::NonZeroVoxelRegion, voxels::{Voxel, VoxelTypeId, VoxelTypeInfo}};
use voxel_edit::ResolvedGridEdit;
use voxel_sources::{
	ChunkSource, RequestId, SourceCoverage, SourceHandle, SourceManager, VoxelSourcesAppExt,
	VoxelSourcesPlugin,
};
use voxel_streaming::systems;
use voxel_streaming::{
	apply_grid_edits, AuthoritativeGridCommand, ChunkAvailabilityChanged, ChunkState,
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
