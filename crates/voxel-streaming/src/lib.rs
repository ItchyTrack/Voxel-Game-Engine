use bevy::prelude::*;

mod chunk;
mod consumer;
mod loader;
mod presence;
mod streaming;

pub use chunk::{chunk_of, chunk_origin, CHUNK_SIZE};
pub use consumer::{chunks_ready, ChunkConsumer, VoxelStreamingAppExt};
pub use loader::{ChunkLoadRequest, ChunkLoadResult, ChunkLoaderChannel, ChunkRequestChannel, ChunkSaveChannel, ChunkSaveRequest};
pub use presence::{ChunkPresence, ChunkState};
pub use streaming::{apply_chunk_clears, flush_dirty_chunks, receive_results, request_stalled_chunks, GridStreaming};

// Re-exports used by the `chunk_consumer!` macro.
#[doc(hidden)]
pub use bevy as __bevy;
#[doc(hidden)]
pub use voxel_data::grid::GridId as __GridId;

#[derive(Default)]
pub struct VoxelStreamingPlugin;

impl Plugin for VoxelStreamingPlugin {
	fn build(&self, app: &mut App) {
		use voxel_edit::VoxelEditAppExt;
		app.init_resource::<ChunkRequestChannel>()
			.init_resource::<ChunkLoaderChannel>()
			.init_resource::<ChunkSaveChannel>()
			.register_edit_gate::<GridStreaming>()
			.add_systems(
				PreUpdate,
				(
					streaming::request_stalled_chunks,
					streaming::receive_results,
					streaming::apply_chunk_clears,
				)
					.before(voxel_edit::ApplyGridEdits),
			)
			.add_systems(
				PreUpdate,
				streaming::flush_dirty_chunks.after(voxel_edit::ApplyGridEdits),
			);
	}
}
