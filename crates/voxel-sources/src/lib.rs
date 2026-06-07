mod handle;
mod registry;
mod source;
mod systems;

use bevy::prelude::*;

pub use handle::SourceHandle;
pub use registry::SourceRegistry;
pub use source::{ChunkSource, GridKey, SourceId};

pub trait VoxelSourcesAppExt {
	fn register_source<S: ChunkSource + 'static>(&mut self, source: S) -> &mut Self;
}

impl VoxelSourcesAppExt for App {
	fn register_source<S: ChunkSource + 'static>(&mut self, source: S) -> &mut Self {
		self.world_mut().resource_mut::<SourceRegistry>().push(Box::new(source));
		self
	}
}

#[derive(Default)]
pub struct VoxelSourcesPlugin;

impl Plugin for VoxelSourcesPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<SourceRegistry>()
			.add_systems(Startup, systems::init_sources)
			.add_systems(
				PreUpdate,
				(
					systems::sync_grid_keys,
					systems::apply_source_events,
					systems::serve_requests,
					systems::serve_saves,
					systems::drain_source_results,
				)
					.chain()
					.before(voxel_streaming::receive_results),
			);
	}
}
