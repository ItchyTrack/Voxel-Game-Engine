mod request;
mod source_manager;
pub mod edit;
mod source;

use bevy::prelude::*;

use crate::{edit::GridEditMessage, source_manager::publish_source_results_messages};
pub use crate::{
	request::{RequestId, SourceResult, SourceResultData},
	source::{ChunkSource, SourceCoverage, SourceHandle, SourceId},
	source_manager::SourceManager,
};

pub trait VoxelSourcesAppExt {
	fn register_voxel_source<S: ChunkSource + 'static>(&mut self, source: S) -> &mut Self;
}

impl VoxelSourcesAppExt for App {
	fn register_voxel_source<S: ChunkSource + 'static>(&mut self, source: S) -> &mut Self {
		self.world_mut()
			.get_resource_or_init::<SourceManager>()
			.add_source(source);
		self
	}
}

#[derive(Default)]
pub struct VoxelSourcesPlugin;

impl Plugin for VoxelSourcesPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<SourceManager>()
			.add_message::<SourceResult>()
			.add_message::<GridEditMessage>()
			.add_systems(PreUpdate, publish_source_results_messages);
	}
}
