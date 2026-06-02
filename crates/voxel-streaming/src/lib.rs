use bevy::prelude::*;

mod chunk;
mod grid_tree;
mod loader;
mod presence;
mod streaming;

pub use chunk::{chunk_of, chunk_origin, CHUNK_SIZE};
pub use loader::{ChunkLoadRequest, ChunkLoadResult, ChunkLoaderChannel};
pub use presence::ChunkPresence;
pub use streaming::{
	chunks_ready, ChunkReadiness, ChunkRequests, ChunkState, GridStreaming, StreamingSet,
};

#[derive(Default)]
pub struct VoxelStreamingPlugin;

impl Plugin for VoxelStreamingPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<ChunkRequests>()
			.init_resource::<ChunkReadiness>()
			.init_resource::<ChunkLoaderChannel>()
			.add_message::<ChunkLoadRequest>()
			.configure_sets(
				PreUpdate,
				(
					StreamingSet::Receive,
					StreamingSet::Clear,
					StreamingSet::Collect,
					StreamingSet::Emit,
				)
					.chain(),
			)
			.add_systems(PreUpdate, streaming::receive_results.in_set(StreamingSet::Receive))
			.add_systems(PreUpdate, streaming::clear_requests.in_set(StreamingSet::Clear))
			.add_systems(PreUpdate, streaming::emit_requests.in_set(StreamingSet::Emit));
	}
}
