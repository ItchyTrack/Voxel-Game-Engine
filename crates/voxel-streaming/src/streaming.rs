use bevy::math::IVec3;
use bevy::prelude::*;
use rustc_hash::FxHashMap;
use voxel_sources::RequestId;

use tile_data::NonZeroChunkRegion;
use voxel_sources::edit::GridGeneration;
use crate::tile_building::TileBuildingCancellationToken;
use crate::presence::ChunkPresence;
use crate::tile_dependency_index::TileDependencyIndex;
use crate::{ChunkRegion, TileKey};

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
	// Dirty, // for now when a tile is dirty we just rerequest it.
}

#[derive(Component, Default)]
pub struct GridStreaming {
	presence: ChunkPresence,
	pub(crate) tiles: FxHashMap<TileKey, TileState>,
	pub(crate) tile_dependencies: TileDependencyIndex,
	edit_interest_counts: FxHashMap<IVec3, u32>,
	edit_interest_version: u64,
	pub(crate) queued_edit_interest: FxHashMap<IVec3, (u64, bool)>,
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

	pub fn retain_edit_interest(&mut self, chunk: IVec3) {
		let count = self.edit_interest_counts.entry(chunk).or_default();
		*count = count.checked_add(1).expect("chunk edit-interest count overflow");
		if *count == 1 {
			self.edit_interest_version = self.edit_interest_version.checked_add(1).expect("edit-interest version space exhausted");
			self.queued_edit_interest.insert(chunk, (self.edit_interest_version, true));
		}
	}

	pub fn release_edit_interest(&mut self, chunk: IVec3) {
		let Some(count) = self.edit_interest_counts.get_mut(&chunk) else { return };
		*count = count.saturating_sub(1);
		if *count == 0 {
			self.edit_interest_counts.remove(&chunk);
			self.edit_interest_version = self.edit_interest_version.checked_add(1).expect("edit-interest version space exhausted");
			self.queued_edit_interest.insert(chunk, (self.edit_interest_version, false));
		}
	}

	pub fn retain_edit_interest_region(&mut self, region: ChunkRegion) {
		for z in region.min().z..region.end().z {
			for y in region.min().y..region.end().y {
				for x in region.min().x..region.end().x { self.retain_edit_interest(IVec3::new(x, y, z)); }
			}
		}
	}

	pub fn release_edit_interest_region(&mut self, region: ChunkRegion) {
		for z in region.min().z..region.end().z {
			for y in region.min().y..region.end().y {
				for x in region.min().x..region.end().x { self.release_edit_interest(IVec3::new(x, y, z)); }
			}
		}
	}
}
