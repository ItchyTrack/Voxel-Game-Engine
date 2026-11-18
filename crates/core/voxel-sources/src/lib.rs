mod data;
pub mod edit;
mod edit_interest;
mod forgotten_chunks;
mod node;
mod request;
mod source;
mod tracker;

pub use forgotten_chunks::ForgottenChunks;

use bevy::prelude::*;
use bile_data::{BileDataPlugin, gather::BileRegisterGather};

use crate::edit::GridEditMessage;
pub use crate::{
	data::{PresenceRequester, StreamCollector, VoxelData, VoxelPresence, VoxelRequest, VoxelRequester},
	edit_interest::{ChunkEditInterest, ChunkEditInterestChanged},
	node::{VoxelSourceNode, VoxelSources},
	request::{RequestId, SourceResult, SourceResultData},
	source::{ChunkSource, SourceHandle, SourceId},
	tracker::VoxelRequestTracker,
};

pub trait VoxelSourcesAppExt {
	/// Spawns the source as a [`VoxelSourceNode`]. Use [`VoxelSources`] to reach it.
	fn register_voxel_source<S: ChunkSource>(&mut self, source: S) -> &mut Self;
}

impl VoxelSourcesAppExt for App {
	fn register_voxel_source<S: ChunkSource>(&mut self, source: S) -> &mut Self {
		node::register_source(self, source);
		self
	}
}

/// Runs in `Update`. Nodes that request [`VoxelData`] or [`VoxelPresence`] should run before it.
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct VoxelSourceNodeSet;

#[derive(Default)]
pub struct VoxelSourcesPlugin;

impl Plugin for VoxelSourcesPlugin {
	fn build(&self, app: &mut App) {
		if !app.is_plugin_added::<BileDataPlugin>() {
			app.add_plugins(BileDataPlugin);
		}
		app.world_mut().get_resource_or_init::<node::SourceChannel>();
		app.world_mut().spawn((Name::new("Voxel Request Tracker"), VoxelRequestTracker::default()));
		app.add_message::<SourceResult>()
			.add_message::<GridEditMessage>()
			.add_message::<ChunkEditInterestChanged>()
			.register_producer_gather::<VoxelData, VoxelRequestTracker>()
			.register_producer_gather::<VoxelPresence, VoxelRequestTracker>()
			.register_producer_gather::<VoxelData, VoxelSourceNode>()
			.register_producer_gather::<VoxelPresence, VoxelSourceNode>()
			.add_systems(Update, (
				tracker::finish_tracker_streams,
				node::receive_source_results,
				tracker::dirty_edited_voxels,
				tracker::publish_edit_interest_changes,
			).chain().in_set(VoxelSourceNodeSet));
	}
}
