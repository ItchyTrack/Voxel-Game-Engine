mod handle;
mod registry;
mod source;
mod systems;
mod worker;

use bevy::prelude::*;

pub use handle::SourceHandle;
pub use registry::SourceRegistry;
pub use source::{ChunkSource, GridKey, SourceId};

pub trait VoxelSourcesAppExt {
	fn register_source<S: ChunkSource + 'static>(&mut self, source: S) -> &mut Self;
}

impl VoxelSourcesAppExt for App {
	fn register_source<S: ChunkSource + 'static>(&mut self, source: S) -> &mut Self {
		self.world_mut().resource_mut::<SourceRegistry>().push(std::sync::Arc::new(source));
		self
	}
}

#[derive(Default)]
pub struct VoxelSourcesPlugin;

impl Plugin for VoxelSourcesPlugin {
	fn build(&self, app: &mut App) {
		use voxel_streaming::{StreamingPhase, StreamingSchedule};
		app.init_resource::<SourceRegistry>()
			.add_systems(Startup, (systems::init_sources, worker::spawn_workers).chain())
			.add_systems(
				StreamingSchedule,
				(
					(systems::sync_grid_keys, systems::apply_source_events)
						.chain()
						.in_set(StreamingPhase::Ingest),
					(
						systems::serve_saves,
						systems::drain_source_results,
						systems::drain_source_lod_results,
					)
						.chain()
						.in_set(StreamingPhase::Serve),
				),
			);
	}
}
