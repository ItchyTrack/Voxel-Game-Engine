use bevy::ecs::message::Message;
use bevy::ecs::schedule::ScheduleLabel;
use bevy::prelude::*;
use tile_data::{ChunkRegion, NonZeroChunkRegion};

mod consumer;
mod forgotten_chunks;
mod grid_source;
mod tile_dependency_index;
mod presence;
mod streaming;
mod source_request_handle;
mod tile_updates;
mod generation;
pub mod systems;
mod edit;

pub use consumer::{chunks_ready, ChunkConsumer, VoxelStreamingAppExt};
pub use edit::{apply_grid_edits, ApplyGridEdits, GridEdits};
pub use forgotten_chunks::ForgottenChunks;
pub use presence::{ChunkPresence, ChunkState};
pub use tile_data::{
	DynamicTileData, LoadedTile, TileClassId, TileClassRegistry, TileData,
	TileGenerationParameters, TileGenerationSession, TileGenerator, TileGeneratorRegistry, TileKey,
	VoxelAreaRequest, VoxelAreaResult,
};
pub use tile_updates::{TileLoadStatus, TileLoadUpdate};
pub use voxel_sources::{ChunkLoadRequest, ChunkLoaded, ChunkPresenceLoaded, ChunksEdited, VoxelAreaKey, VoxelAreaMessageRequest, VoxelAreaLoaded, PresenceLoadRequest};
pub type ChunkLoadResult = voxel_sources::ChunkLoaded;
pub use source_request_handle::StreamingSourceRequestHandle;
pub use streaming::{InflightChunkPresence, GridStreaming, RequestChunkPresence};
pub use systems::request_presence_for_new_grids;

// Re-exports used by the `chunk_consumer!` macro.
#[doc(hidden)]
pub use bevy as __bevy;
#[doc(hidden)]
pub use voxel_data::grid::GridId as __GridId;

/// Loading pipeline. Safe to run several times a frame; invoke with [`run_streaming`].
#[derive(ScheduleLabel, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct StreamingSchedule;

/// Frame-counted upkeep. Run exactly once a frame via [`run_streaming_maintenance`].
#[derive(ScheduleLabel, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct StreamingMaintenance;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChunkAvailabilityChangeKind {
	BecamePresent,
	BecameEmpty,
}

#[derive(Message, Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChunkAvailabilityChanged {
	pub grid: voxel_data::grid::GridId,
	pub region: NonZeroChunkRegion,
	pub kind: ChunkAvailabilityChangeKind,
}

#[derive(Message, Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChunkEditInterestChanged {
	pub grid: voxel_data::grid::GridId,
	pub region: NonZeroChunkRegion,
	pub version: u64,
	pub interested: bool,
}

#[derive(Message, Clone)]
pub struct AuthoritativeGridCommand {
	pub grid: voxel_data::grid::GridId,
	pub region: NonZeroChunkRegion,
	pub stream_sequence: u64,
	pub generation: u64,
	pub edit: voxel_edit::GridEdit,
}

#[derive(Message, Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChunkLoadResolved {
	pub grid: voxel_data::grid::GridId,
	pub chunk: IVec3,
	pub visible: bool,
}

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum StreamingPhase {
	Ingest,
	Request,
	Serve,
	Receive,
}

pub fn run_streaming(world: &mut World) {
	world.run_schedule(StreamingSchedule);
}

pub fn run_streaming_maintenance(world: &mut World) {
	world.run_schedule(StreamingMaintenance);
}

#[derive(Default)]
pub struct VoxelStreamingPlugin;

impl Plugin for VoxelStreamingPlugin {
	fn build(&self, app: &mut App) {
		use voxel_sources::VoxelSourcesAppExt;
		if !app.is_plugin_added::<voxel_sources::VoxelSourcesPlugin>() {
			app.add_plugins(voxel_sources::VoxelSourcesPlugin);
		}
		let grid_source = grid_source::StreamingGridSource::default();
		app.insert_resource(grid_source.clone()).register_voxel_source(grid_source);
		app.add_message::<ChunkAvailabilityChanged>()
			.add_message::<ChunkEditInterestChanged>()
			.add_message::<AuthoritativeGridCommand>()
			.add_message::<ChunkLoadResolved>()
			.init_resource::<TileClassRegistry>()
			.init_resource::<TileGeneratorRegistry>()
			.init_resource::<StreamingSourceRequestHandle>()
			.init_resource::<systems::PendingTileUpdates>()
			.init_resource::<generation::TileGenerationChannel>()
			.add_systems(
				StreamingSchedule,
				(
					(
						systems::receive_chunk_presence_loaded,
						systems::apply_source_presence,
						systems::apply_edit_events,
					)
						.in_set(StreamingPhase::Ingest),
					grid_source::serve_grid_source_requests
						.in_set(StreamingPhase::Serve),
				),
			)
			.init_schedule(StreamingSchedule)
			.init_schedule(StreamingMaintenance)
			.configure_sets(
				StreamingSchedule,
				(
					StreamingPhase::Ingest,
					StreamingPhase::Request,
					StreamingPhase::Serve,
					StreamingPhase::Receive,
				)
					.chain(),
			)
			.add_systems(
				StreamingSchedule,
				(
					(
						systems::invalidate_changed_generation_contexts,
						systems::request_presence_for_new_grids,
						systems::handle_dirty_chunks,
						systems::request_edit_takes,
						systems::request_stalled_chunks,
						systems::request_tiles,
						systems::publish_edit_interest_changes,
					)
						.chain()
						.in_set(StreamingPhase::Request),
					(
						systems::receive_results,
						(
							systems::receive_tile_results,
							systems::publish_tile_updates,
						).chain(),
					)
						.in_set(StreamingPhase::Receive),
				),
			)
			.add_systems(StreamingMaintenance, (systems::cleanup_released_tiles, systems::apply_chunk_clears).chain())
			.add_systems(PreUpdate, (systems::materialize_authoritative_commands, run_streaming, run_streaming_maintenance).chain())
			.add_systems(Last, apply_grid_edits.in_set(ApplyGridEdits))
			.add_systems(
				PostUpdate,
				bevy::transform::systems::propagate_transforms_for::<Added<LoadedTile>>
					.after(bevy::transform::TransformSystems::Propagate),
			);
	}
}
