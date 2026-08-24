use bevy::ecs::message::Message;
use bevy::ecs::schedule::ScheduleLabel;
use bevy::prelude::*;
use tile_data::{ChunkRegion, NonZeroChunkRegion};

mod tile_dependency_index;
mod presence;
mod streaming;
pub mod tile_requester;
mod tile_building;
pub mod systems;

use tile_data::{LoadedTile, TileKey};
pub use tile_requester::{TileLoadStatus, TileLoadUpdate};
pub use streaming::{InflightChunkPresence, GridStreaming, RequestChunkPresence};
pub use systems::request_presence_for_new_grids;

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
		if !app.is_plugin_added::<voxel_sources::VoxelSourcesPlugin>() {
			app.add_plugins(voxel_sources::VoxelSourcesPlugin);
		}
		if !app.is_plugin_added::<tile_data::TileDataPlugin>() {
			app.add_plugins(tile_data::TileDataPlugin);
		}
		app.add_message::<ChunkAvailabilityChanged>()
			.add_message::<ChunkEditInterestChanged>()
			.init_resource::<systems::PendingTileUpdates>()
			.init_resource::<tile_building::TileBuildingChannel>()
			.init_resource::<tile_building::TileVoxelSourceBridge>()
			.add_systems(
				StreamingSchedule,
				(
					(
						systems::receive_chunk_presence_loaded,
						systems::apply_source_presence,
					)
						.in_set(StreamingPhase::Ingest),
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
						systems::publish_edit_interest_changes,
						tile_building::submit_tile_voxel_requests,
					)
						.chain()
						.in_set(StreamingPhase::Request),
					(
						tile_building::route_tile_source_results,
						(
							systems::receive_tile_results,
						).chain(),
					)
						.in_set(StreamingPhase::Receive),
				),
			)
			.add_systems(StreamingMaintenance, systems::cleanup_released_tiles)
			.add_systems(PreUpdate, (run_streaming, run_streaming_maintenance).chain())
			.add_systems(
				PostUpdate,
				bevy::transform::systems::propagate_transforms_for::<Added<LoadedTile>>
					.after(bevy::transform::TransformSystems::Propagate),
			);
	}
}
