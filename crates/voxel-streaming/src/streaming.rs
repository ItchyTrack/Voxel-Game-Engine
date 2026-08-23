use bevy::math::IVec3;
use bevy::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};
use voxel_sources::RequestId;

use voxel_data::{
	grid_tree::NonZeroVoxelRegion,
	voxels::Voxels,
};

use tile_data::{CHUNK_SIZE, NonZeroChunkRegion};
use crate::generation::TileGenerationCancellation;
use crate::presence::ChunkPresence;
use crate::tile_dependency_index::TileDependencyIndex;
use crate::{ChunkRegion, TileKey, TileLoadStatus, TileLoadUpdate};

const CLEAR_DELAY_FRAMES: u8 = 20;

#[derive(Debug)]
pub(crate) struct TileState {
	pub(crate) requesters: FxHashMap<Entity, f32>,
	pub(crate) status: TileStatus,
	pub(crate) active: Option<Entity>,
}

#[derive(Debug)]
pub(crate) enum TileStatus {
	Requested,
	InFlight { tag: u64, cancellation: TileGenerationCancellation },
	Loaded,
	Dirty,
	Empty,
}

#[derive(Component, Default)]
pub struct GridStreaming {
	pub(crate) presence: ChunkPresence,
	// pub(crate) inflight_chunk_cancellations: FxHashMap<IVec3, RequestId>,
	pub(crate) tiles: FxHashMap<TileKey, TileState>,
	pub(crate) pending_tile_requests: FxHashSet<TileKey>,
	pub(crate) tile_dependencies: TileDependencyIndex,
	pub(crate) inflight_tiles_by_tag: FxHashMap<u64, TileKey>,
	pub(crate) next_tile_tag: u64,
	pub(crate) queued_tile_updates: Vec<TileLoadUpdate>,
	pub(crate) edit_interest_counts: FxHashMap<IVec3, u32>,
	pub(crate) edit_interest_version: u64,
	pub(crate) queued_edit_interest: FxHashMap<IVec3, (u64, bool)>,
}

#[derive(Component, Debug, Default)]
pub struct RequestChunkPresence;

#[derive(Component, Debug)]
pub struct InflightChunkPresence(pub(crate) RequestId);

impl GridStreaming {
	pub fn presence(&self) -> &ChunkPresence { &self.presence }
	#[doc(hidden)]
	pub fn presence_mut(&mut self) -> &mut ChunkPresence { &mut self.presence }

	pub fn mark_present(&mut self, chunk: IVec3) { self.presence.mark_present(chunk); }
	pub fn mark_present_area(&mut self, region: NonZeroChunkRegion) { self.presence.mark_present_area(region); }

	pub fn fetch_tile(&mut self, requester: Entity, key: TileKey, priority: f32) -> bool {
		if !valid_tile_key(key) { return false; }
		let (new_requester, first_requester, status, should_request) = {
			let state = self.tiles.entry(key).or_insert_with(|| TileState {
				requesters: FxHashMap::default(),
				status: TileStatus::Requested,
				active: None,
			});
			let new_requester = state.requesters.insert(requester, priority).is_none();
			let status = match (&state.status, state.active) {
				(TileStatus::Loaded, Some(entity)) => Some(TileLoadStatus::Ready(entity)),
				(TileStatus::Empty, _) => Some(TileLoadStatus::Empty),
				_ => None,
			};
			(new_requester, new_requester && state.requesters.len() == 1, status, matches!(&state.status, TileStatus::Requested | TileStatus::Dirty))
		};
		if first_requester { self.retain_edit_interest_region(key.region.into()); }
		if new_requester && let Some(status) = status {
			self.queued_tile_updates.push(TileLoadUpdate { grid: Entity::PLACEHOLDER, requester, key, status });
		}
		if should_request { self.pending_tile_requests.insert(key); }
		true
	}

	pub fn release_tile(&mut self, requester: Entity, key: TileKey) {
		let status = {
			let Some(state) = self.tiles.get_mut(&key) else { return; };
			state.requesters.remove(&requester);
			if !state.requesters.is_empty() { return; }
			std::mem::replace(&mut state.status, TileStatus::Requested)
		};
		self.release_edit_interest_region(key.region.into());
		self.pending_tile_requests.remove(&key);
		match status {
			TileStatus::InFlight { tag, cancellation } => {
				self.inflight_tiles_by_tag.remove(&tag);
				cancellation.cancel();
			}
			other => self.tiles.get_mut(&key).unwrap().status = other,
		}
	}

	pub(crate) fn mark_empty(&mut self, region: NonZeroChunkRegion) {
		self.presence.clear_present_area(region);
	}

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

	pub(crate) fn dirty_stale_tiles(&mut self, region: NonZeroChunkRegion, generation: u64) {
		let keys: FxHashSet<_> = self.tile_dependencies.stale_tiles(region, generation).collect();
		for key in keys { self.dirty_tile(key); }
	}

	pub(crate) fn invalidate_generation_context(&mut self) {
		let keys: Vec<_> = self.tiles.keys().copied().collect();
		for key in keys { self.dirty_tile(key); }
	}

	fn dirty_tile(&mut self, key: TileKey) {
		let Some(state) = self.tiles.get_mut(&key) else { return };
		if state.requesters.is_empty() { return; }
		let status = std::mem::replace(&mut state.status, TileStatus::Dirty);
		if let TileStatus::InFlight { tag, cancellation } = status {
			self.inflight_tiles_by_tag.remove(&tag);
			cancellation.cancel();
		}
		state.status = TileStatus::Dirty;
		self.pending_tile_requests.insert(key);
	}
}

fn chunk_tree_region(region: NonZeroChunkRegion) -> NonZeroVoxelRegion {
	NonZeroVoxelRegion::new(region.min(), region.size()).unwrap()
}

fn valid_tile_key(key: TileKey) -> bool {
	let factor = 1u32 << key.lod;
	let coarse_extent = (key.size() * CHUNK_SIZE as u32) / factor;
	!coarse_extent.cmplt(UVec3::ONE).any() && !coarse_extent.cmpgt(UVec3::splat(CHUNK_SIZE as u32)).any()
}

#[cfg(test)]
mod tests {
	use tile_data::NonZeroChunkRegion;

use super::*;
	use crate::TileClassId;

	fn tile(size: IVec3, lod: u8) -> Option<TileKey> {
		Some(TileKey::new(NonZeroChunkRegion::new(IVec3::ZERO, size.as_uvec3())?, lod, TileClassId(0)))
	}

	#[test]
	fn tile_validity_keeps_size_and_lod_independent_with_one_chunk_output_limit() {
		assert!(valid_tile_key(tile(IVec3::ONE, 0).unwrap()));
		assert!(!valid_tile_key(tile(IVec3::splat(2), 0).unwrap()));
		assert!(valid_tile_key(tile(IVec3::splat(2), 1).unwrap()));
	}

	#[test]
	fn tile_class_is_part_of_shared_store_identity() {
		let requester = Entity::from_bits(1);
		let mut streaming = GridStreaming::default();
		let first = tile(IVec3::ONE, 0).unwrap();
		let second = TileKey { class: TileClassId(1), ..first };
		assert!(streaming.fetch_tile(requester, first, 0.0));
		assert!(streaming.fetch_tile(requester, second, 0.0));
		assert_eq!(streaming.tiles.len(), 2);
	}
}
