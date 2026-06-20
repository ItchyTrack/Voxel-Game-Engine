mod handle;
mod loader;
mod registry;
mod requests;
mod source;
mod systems;
mod worker;

use bevy::ecs::message::Message;
use bevy::prelude::*;
use crate::registry::SourceRegistry;

pub use handle::{SourceEvent, SourceHandle, SourceLodResult, SourceResult};
pub use loader::{ChunkLoadRequest, ChunkSaveChannel, ChunkSaveRequest, LodKey, LodLoadRequest, PresenceLoadRequest};
pub use requests::{VoxelSourceRequestApi, VoxelSourceRequests, VoxelSources};
pub use source::{ChunkSource, SourceId, VoxelLodGenerator};

#[derive(Message, Debug, Clone)]
pub struct ChunkLoaded {
	pub grid: voxel_data::grid::GridId,
	pub chunk: bevy::math::IVec3,
	pub voxels: Option<voxel_data::voxels::Voxels>,
}

#[derive(Message, Debug, Clone)]
pub struct LodLoaded {
	pub grid: voxel_data::grid::GridId,
	pub requester: Entity,
	pub key: LodKey,
	pub priority: f32,
	pub generation: u64,
	pub voxels: Option<voxel_data::voxels::Voxels>,
	pub entity: Option<Entity>,
}

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
			.init_resource::<ChunkSaveChannel>()
			.add_message::<SourceEvent>()
			.add_message::<ChunkLoaded>()
			.add_message::<LodLoaded>()
			.add_systems(Startup, (systems::init_sources, worker::spawn_workers).chain())
			.add_systems(PreUpdate, systems::publish_source_messages);
	}
}
