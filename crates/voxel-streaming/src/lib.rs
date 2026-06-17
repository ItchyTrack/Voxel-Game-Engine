use bevy::ecs::message::Message;
use bevy::ecs::schedule::ScheduleLabel;
use bevy::prelude::*;

mod chunk;
mod consumer;
mod loader;
mod lod_index;
mod presence;
mod streaming;

pub use chunk::{chunk_of, chunk_origin, CHUNK_SIZE};
pub use consumer::{chunks_ready, ChunkConsumer, VoxelStreamingAppExt};
pub use loader::{ChunkLoadRequest, ChunkLoadResult, ChunkLoaderChannel, ChunkRequestChannel, ChunkSaveChannel, ChunkSaveRequest, LodKey, LodLoadRequest, LodLoadResult, LodLoaderChannel, LodRequestChannel};
pub use presence::{ChunkPresence, ChunkState};
pub use streaming::{apply_chunk_clears, cleanup_released_lods, handle_dirty_chunks, receive_lod_results, receive_results, refresh_lod_uploads, request_lod_tiles, request_stalled_chunks, GridStreaming};

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

#[derive(Message, Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChunkBecamePresent {
	pub grid: voxel_data::grid::GridId,
	pub chunk: IVec3,
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
		use voxel_edit::VoxelEditAppExt;
		app.init_resource::<ChunkRequestChannel>()
			.add_message::<ChunkBecamePresent>()
			.add_message::<ChunkLoadResolved>()
			.init_resource::<ChunkLoaderChannel>()
			.init_resource::<ChunkSaveChannel>()
			.init_resource::<LodRequestChannel>()
			.init_resource::<LodLoaderChannel>()
			.register_edit_gate::<GridStreaming>()
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
					(streaming::handle_dirty_chunks, streaming::request_stalled_chunks, streaming::request_lod_tiles)
						.in_set(StreamingPhase::Request),
					(streaming::receive_results, streaming::receive_lod_results)
						.in_set(StreamingPhase::Receive),
				),
			)
			.add_systems(Update, streaming::refresh_lod_uploads.after(voxel_gpu::GpuUploadSet::Upload))
			.add_systems(StreamingMaintenance, (streaming::cleanup_released_lods, streaming::apply_chunk_clears).chain())
			.add_systems(PreUpdate, (run_streaming, run_streaming_maintenance).chain());
	}
}
