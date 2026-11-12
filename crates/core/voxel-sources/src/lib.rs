mod request;
mod source_manager;
pub mod edit;
mod source;
mod forgotten_chunks;
mod data;
mod edit_interest;
mod node;
pub use forgotten_chunks::ForgottenChunks;

use bevy::prelude::*;
use bevy_trait_query::RegisterExt;
use bile_data::{BileAddDataType, BileDataPlugin, DataProducingNode};

use crate::{edit::GridEditMessage, source_manager::publish_source_results_messages};
pub use crate::{
	data::{PresenceRegions, PresenceRequester, VoxelData, VoxelPresence, VoxelRegions, VoxelRequest, VoxelRequester},
	edit_interest::{ChunkEditInterest, ChunkEditInterestChanged},
	node::VoxelSourceNode,
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

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct VoxelSourcesSet;

/// Runs in `Update`. Nodes that request [`VoxelData`] or [`VoxelPresence`] should run before it.
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct VoxelSourceNodeSet;

#[derive(Default)]
pub struct VoxelSourcesPlugin;

impl Plugin for VoxelSourcesPlugin {
	fn build(&self, app: &mut App) {
		if !app.is_plugin_added::<BileDataPlugin>() {
			app.add_plugins(BileDataPlugin::default());
		}
		app.bile_add_data_type::<VoxelData>();
		app.bile_add_data_type::<VoxelPresence>();
		app.world_mut().spawn((Name::new("Voxel Source Node"), VoxelSourceNode::default()));
		app.init_resource::<SourceManager>()
			.add_message::<SourceResult>()
			.add_message::<GridEditMessage>()
			.add_message::<ChunkEditInterestChanged>()
			.register_component_as::<dyn DataProducingNode<VoxelData>, VoxelSourceNode>()
			.register_component_as::<dyn DataProducingNode<VoxelPresence>, VoxelSourceNode>()
			.add_systems(PreUpdate, publish_source_results_messages.in_set(VoxelSourcesSet))
			.add_systems(Update, (
				node::process_voxel_requests,
				node::dirty_edited_voxels,
				node::receive_voxel_results,
				node::publish_edit_interest_changes,
			).chain().in_set(VoxelSourceNodeSet));
	}
}
