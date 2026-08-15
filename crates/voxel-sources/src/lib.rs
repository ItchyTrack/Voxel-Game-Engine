mod reader;
mod generation;
mod source_manager;

use bevy::ecs::message::Message;
use bevy::prelude::*;

pub use reader::{
	ChunkLoadRequest, PresenceLoadRequest,
	VoxelAreaCancellation, VoxelAreaKey, VoxelAreaMessageRequest, VoxelAreaLoadEvent,
	VoxelAreaLoadRequest, VoxelAreaLoadResult,
	VoxelSourcesRequestHandle, VoxelSourcesRequestHandleGetter,
};
pub use generation::ChunkGenerationIndex;
pub use source_manager::{
	ChunkPresence, ChunksEdited, ChunkSource, Completed, SourceCoverage, SourceHandle, SourceId,
	SourceManager, TakeJob, VoxelLodGenerator,
};
pub use voxel_tasks::CancellationToken;

#[derive(Message, Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChunkPresenceLoaded {
	pub grid: voxel_data::grid::GridId,
}

#[derive(Message, Debug, Clone)]
pub struct ChunkLoaded {
	pub grid: voxel_data::grid::GridId,
	pub chunk: bevy::math::IVec3,
	pub generation: u64,
	pub voxels: Option<voxel_data::voxels::Voxels>,
}

#[derive(Message, Debug, Clone)]
pub struct VoxelAreaLoaded {
	pub grid: voxel_data::grid::GridId,
	pub requester: Entity,
	pub key: VoxelAreaKey,
	pub voxel_type: voxel_data::voxels::VoxelTypeId,
	pub tag: u64,
	pub priority: f32,
	pub generation: u64,
	pub voxels: Option<voxel_data::voxels::Voxels>,
}

pub trait VoxelSourcesAppExt {
	fn register_voxel_source<S: ChunkSource + 'static>(&mut self, source: S) -> &mut Self;
	fn register_voxel_lod_generator<G: VoxelLodGenerator + 'static>(&mut self, generator: G) -> &mut Self;
}

impl VoxelSourcesAppExt for App {
	fn register_voxel_source<S: ChunkSource + 'static>(&mut self, source: S) -> &mut Self {
		self.world_mut().resource_mut::<SourceManager>().push(std::sync::Arc::new(source));
		self
	}

	fn register_voxel_lod_generator<G: VoxelLodGenerator + 'static>(&mut self, generator: G) -> &mut Self {
		self.world_mut().resource_mut::<SourceManager>().register_lod_generator(std::sync::Arc::new(generator));
		self
	}
}

#[derive(Default)]
pub struct VoxelSourcesPlugin;

impl Plugin for VoxelSourcesPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<SourceManager>()
			.init_resource::<reader::VoxelReader>()
			.init_resource::<VoxelSourcesRequestHandleGetter>()
			.add_message::<ChunkPresence>()
			.add_message::<ChunksEdited>()
			.add_message::<ChunkPresenceLoaded>()
			.add_message::<ChunkLoaded>()
			.add_message::<VoxelAreaLoaded>()
			.add_systems(Startup, (reader::systems::init_sources, source_manager::spawn_workers).chain())
			.add_systems(PreUpdate, reader::systems::publish_source_messages)
			.add_systems(PostUpdate, reader::systems::process_source_requests);
	}
}
