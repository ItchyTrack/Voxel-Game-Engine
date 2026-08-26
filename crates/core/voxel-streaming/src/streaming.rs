use bevy::prelude::*;
use rustc_hash::FxHashMap;
use voxel_sources::RequestId;

use tile_data::NonZeroChunkRegion;
use voxel_sources::edit::GridGeneration;
use crate::tile_building::TileBuildingCancellationToken;
use crate::presence::ChunkPresence;
use crate::tile_dependency_index::TileDependencyIndex;
use crate::{ChunkEditInterest, TileKey};

#[derive(Debug)]
pub(crate) struct TileState {
	pub(crate) requesters: FxHashMap<Entity, f32>,
	pub(crate) status: TileStatus,
	pub(crate) entity: Option<Entity>,
}

#[derive(Debug)]
pub(crate) enum TileStatus {
	InFlight { generation: GridGeneration, cancellation: TileBuildingCancellationToken },
	Loaded,
}

#[derive(Component, Default)]
pub struct GridStreaming {
	presence: ChunkPresence,
	pub(crate) tiles: FxHashMap<TileKey, TileState>,
	pub(crate) tile_dependencies: TileDependencyIndex,
	edit_interest: ChunkEditInterest,
	pub(crate) queued_edit_interest: Vec<(NonZeroChunkRegion, bool)>,
}

#[derive(Component, Debug, Default)]
pub struct RequestChunkPresence;

#[derive(Component, Debug)]
pub struct InflightChunkPresence(pub(crate) RequestId);

impl GridStreaming {
	/* ----------- Presence ----------- */

	pub fn presence(&self) -> &ChunkPresence { &self.presence }
	#[doc(hidden)]
	pub fn presence_mut(&mut self) -> &mut ChunkPresence { &mut self.presence }

	pub fn mark_present_area(&mut self, region: NonZeroChunkRegion) { self.presence.mark_present_area(region); }
	pub(crate) fn mark_empty(&mut self, region: NonZeroChunkRegion) { self.presence.clear_present_area(region); }

	/* ----------- Interest ----------- */

	pub fn retain_edit_interest_region(&mut self, region: NonZeroChunkRegion) {
		self.edit_interest.retain(region);
		self.queued_edit_interest.push((region, true));
	}

	pub fn release_edit_interest_region(&mut self, region: NonZeroChunkRegion) {
		if self.edit_interest.release(region) {
			self.queued_edit_interest.push((region, false));
		}
	}
}
