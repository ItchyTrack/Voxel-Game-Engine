use bevy::prelude::*;

mod chunk;
mod consumer;
mod loader;
mod presence;
mod streaming;

pub use chunk::{chunk_of, chunk_origin, CHUNK_SIZE};
pub use consumer::{chunks_ready, ChunkConsumer, VoxelStreamingAppExt};
pub use loader::{ChunkLoadRequest, ChunkLoadResult, ChunkLoaderChannel, ChunkRequestChannel};
pub use presence::{ChunkPresence, ChunkState};
pub use streaming::{apply_chunk_clears, receive_results, GridStreaming};

// Re-exports used by the `chunk_consumer!` macro.
#[doc(hidden)]
pub use bevy as __bevy;
#[doc(hidden)]
pub use voxel_data::grid::GridId as __GridId;

#[derive(Default)]
pub struct VoxelStreamingPlugin;

impl Plugin for VoxelStreamingPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<ChunkRequestChannel>()
			.init_resource::<ChunkLoaderChannel>()
			.add_systems(
				PreUpdate,
				(streaming::receive_results, streaming::apply_chunk_clears)
					.before(voxel_data::ApplyGridEdits),
			);
	}
}
