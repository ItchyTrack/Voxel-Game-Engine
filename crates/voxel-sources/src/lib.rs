mod handle;
mod loader;
mod registry;
mod source;
mod systems;
mod worker;

use bevy::prelude::*;

pub use handle::{SourceEvent, SourceHandle, SourceLodResult, SourceResult};
pub use loader::{ChunkLoadRequest, ChunkLoadResult, ChunkLoaderChannel, ChunkRequestChannel, ChunkSaveChannel, ChunkSaveRequest, LodKey, LodLoadRequest, LodLoadResult, LodLoaderChannel, LodRequestChannel};
pub use registry::SourceRegistry;
pub use source::{ChunkSource, GridKey, SourceId, VoxelLodGenerator};

pub trait VoxelSourcesAppExt {
	fn register_source<S: ChunkSource + 'static>(&mut self, source: S) -> &mut Self;
	fn set_voxel_lod_generator<G: VoxelLodGenerator + 'static>(&mut self, generator: G) -> &mut Self;
}

impl VoxelSourcesAppExt for App {
	fn register_source<S: ChunkSource + 'static>(&mut self, source: S) -> &mut Self {
		self.world_mut().resource_mut::<SourceRegistry>().push(std::sync::Arc::new(source));
		self
	}

	fn set_voxel_lod_generator<G: VoxelLodGenerator + 'static>(&mut self, generator: G) -> &mut Self {
		self.world_mut().resource_mut::<SourceRegistry>().set_lod_generator(std::sync::Arc::new(generator));
		self
	}
}

#[derive(Default)]
pub struct VoxelSourcesPlugin;

impl Plugin for VoxelSourcesPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<SourceRegistry>()
			.init_resource::<ChunkRequestChannel>()
			.init_resource::<ChunkLoaderChannel>()
			.init_resource::<ChunkSaveChannel>()
			.init_resource::<LodRequestChannel>()
			.init_resource::<LodLoaderChannel>()
			.add_systems(Startup, (systems::init_sources, worker::spawn_workers).chain());
	}
}
