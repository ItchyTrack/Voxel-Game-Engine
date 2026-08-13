use std::collections::{HashMap, HashSet};

use bevy::math::IVec3;
use bevy::prelude::*;
use voxel_sources::{CancellationToken, VoxelSourcesRequestHandle};

use voxel_data::grid::GridId;
use voxel_edit::GridEdit;

use tile_data::CHUNK_SIZE;
use crate::consumer::ChunkConsumer;
use crate::generation::TileGenerationCancellation;
use crate::tile_dependency_index::TileDependencyIndex;
use crate::{ChunkLoadRequest, ChunkRegion, TileKey, TileLoadStatus, TileLoadUpdate};
use crate::presence::{ChunkPresence, ChunkState};

const CLEAR_DELAY_FRAMES: u8 = 20;

#[derive(Debug)]
pub(crate) struct TileState {
	pub(crate) requesters: HashMap<Entity, f32>,
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
	pub(crate) inflight_chunk_cancellations: HashMap<IVec3, CancellationToken>,
	pub(crate) pending_clears: Vec<(IVec3, u8)>,
	pub(crate) stalled_edits: HashMap<IVec3, Vec<GridEdit>>,
	pub(crate) stalled_pinned: HashSet<IVec3>,
	pub(crate) borrowed_chunks: HashSet<IVec3>,
	pub(crate) pending_borrow_edits: Vec<GridEdit>,
	pub(crate) inflight_borrow_edits: HashMap<u64, GridEdit>,
	pub(crate) completed_edits: Vec<GridEdit>,
	pub(crate) newly_dirty: Vec<IVec3>,
	pub(crate) newly_present_dirty: Vec<IVec3>,
	pub(crate) tiles: HashMap<TileKey, TileState>,
	pub(crate) pending_tile_requests: HashSet<TileKey>,
	pub(crate) tile_dependencies: TileDependencyIndex,
	pub(crate) inflight_tiles_by_tag: HashMap<u64, TileKey>,
	pub(crate) latest_source_edit_index: u64,
	pub(crate) next_tile_tag: u64,
	pub(crate) queued_tile_updates: Vec<TileLoadUpdate>,
	pub(crate) queued_edit_interest: Vec<(ChunkRegion, bool)>,
}

#[derive(Component, Debug, Default)]
pub struct RequestChunkPresence;

#[derive(Component, Debug, Default)]
pub struct InflightChunkPresence;

impl GridStreaming {
	pub fn presence(&self) -> &ChunkPresence { &self.presence }
	#[doc(hidden)]
	pub fn presence_mut(&mut self) -> &mut ChunkPresence { &mut self.presence }

	pub fn mark_present(&mut self, chunk: IVec3) { self.presence.mark_present(chunk); }
	pub fn mark_present_area(&mut self, min: IVec3, size: IVec3) { self.presence.mark_present_area(min, size); }
	pub fn state(&self, chunk: IVec3) -> Option<ChunkState> { self.presence.state(chunk) }
	pub fn is_loaded(&self, chunk: IVec3) -> bool { matches!(self.presence.state(chunk), Some(ChunkState::Loaded)) }

	fn start_request(&mut self, grid: GridId, requests: &VoxelSourcesRequestHandle, chunk: IVec3) -> bool {
		match self.presence.state(chunk) {
			None => return false,
			Some(ChunkState::Available) => {
				self.presence.set_state(chunk, ChunkState::InFlight);
				let cancellation = requests.request_chunk(ChunkLoadRequest { grid, chunk });
				self.inflight_chunk_cancellations.insert(chunk, cancellation);
			}
			_ => {}
		}
		let first_request = self.presence.request_count(chunk) == 0;
		self.presence.add_request(chunk);
		if first_request { self.queued_edit_interest.push((ChunkRegion::new(chunk, UVec3::ONE), true)); }
		true
	}

	pub fn fetch(&mut self, grid: GridId, requests: &VoxelSourcesRequestHandle, chunk: IVec3) { self.start_request(grid, requests, chunk); }

	pub fn fetch_needed<C: ChunkConsumer>(&mut self, grid: GridId, consumer: &mut C, requests: &VoxelSourcesRequestHandle, chunk: IVec3) {
		if !self.start_request(grid, requests, chunk) { return; }
		let resident = matches!(self.presence.state(chunk), Some(ChunkState::Loaded | ChunkState::InternalDirty));
		if consumer.needed_mut().entry(grid).or_default().insert(chunk) && !resident { *consumer.outstanding_mut() += 1; }
	}

	pub fn release(&mut self, chunk: IVec3) {
		if self.presence.remove_request(chunk) > 0 { return; }
		self.queued_edit_interest.push((ChunkRegion::new(chunk, UVec3::ONE), false));
		match self.presence.state(chunk) {
			Some(ChunkState::InFlight) => {
				self.presence.set_state(chunk, ChunkState::Available);
				if let Some(cancellation) = self.inflight_chunk_cancellations.remove(&chunk) { cancellation.cancel(); }
			}
			Some(ChunkState::Loaded | ChunkState::InternalDirty) => self.pending_clears.push((chunk, CLEAR_DELAY_FRAMES)),
			_ => {}
		}
	}

	pub(crate) fn release_completed(&mut self, chunk: IVec3) {
		if self.presence.remove_request(chunk) > 0 { return; }
		if matches!(self.presence.state(chunk), Some(ChunkState::Loaded | ChunkState::InternalDirty)) {
			self.pending_clears.push((chunk, CLEAR_DELAY_FRAMES));
		}
	}

	pub fn release_needed<C: ChunkConsumer>(&mut self, grid: GridId, consumer: &mut C, chunk: IVec3) {
		let resident = matches!(self.presence.state(chunk), Some(ChunkState::Loaded | ChunkState::InternalDirty));
		let removed = consumer.needed_mut().get_mut(&grid).is_some_and(|set| set.remove(&chunk));
		if consumer.needed().get(&grid).is_some_and(|set| set.is_empty()) { consumer.needed_mut().remove(&grid); }
		if removed && !resident { *consumer.outstanding_mut() = consumer.outstanding().saturating_sub(1); }
		self.release(chunk);
	}

	pub fn fetch_tile(&mut self, requester: Entity, key: TileKey, priority: f32) -> bool {
		if !valid_tile_key(key) { return false; }
		let state = self.tiles.entry(key).or_insert_with(|| TileState {
			requesters: HashMap::new(),
			status: TileStatus::Requested,
			active: None,
		});
		let new_requester = state.requesters.insert(requester, priority).is_none();
		if new_requester {
			if state.requesters.len() == 1 { self.queued_edit_interest.push((key.region.into(), true)); }
			let status = match (&state.status, state.active) {
				(TileStatus::Loaded, Some(entity)) => Some(TileLoadStatus::Ready(entity)),
				(TileStatus::Empty, _) => Some(TileLoadStatus::Empty),
				_ => None,
			};
			if let Some(status) = status { self.queued_tile_updates.push(TileLoadUpdate { grid: Entity::PLACEHOLDER, requester, key, status }); }
		}
		if matches!(&state.status, TileStatus::Requested | TileStatus::Dirty) {
			self.pending_tile_requests.insert(key);
		}
		true
	}

	pub fn release_tile(&mut self, requester: Entity, key: TileKey) {
		let Some(state) = self.tiles.get_mut(&key) else { return; };
		state.requesters.remove(&requester);
		if !state.requesters.is_empty() { return; }
		self.queued_edit_interest.push((key.region.into(), false));
		self.pending_tile_requests.remove(&key);
		let status = std::mem::replace(&mut state.status, TileStatus::Requested);
		match status {
			TileStatus::InFlight { tag, cancellation } => {
				self.inflight_tiles_by_tag.remove(&tag);
				cancellation.cancel();
			}
			other => state.status = other,
		}
	}

	pub(crate) fn finish_chunk_request(&mut self, chunk: IVec3) { self.inflight_chunk_cancellations.remove(&chunk); }
	pub(crate) fn mark_loaded(&mut self, chunk: IVec3) {
		self.presence.set_state(chunk, ChunkState::Loaded);
	}

	pub(crate) fn mark_empty(&mut self, min: IVec3, size: IVec3) {
		self.presence.clear_present_area(min, size);
	}

	pub(crate) fn note_source_edit_index(&mut self, edit_index: u64) {
		self.latest_source_edit_index = self.latest_source_edit_index.max(edit_index);
	}

	pub(crate) fn dirty_stale_tiles_covering(&mut self, chunk: IVec3, edit_index: u64) {
		let keys: HashSet<_> = self.tile_dependencies.stale_tiles_for_chunk(chunk, edit_index).collect();
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

fn valid_tile_key(key: TileKey) -> bool {
	let factor = 1u32 << key.lod;
	let coarse_extent = (key.size() * CHUNK_SIZE as u32) / factor;
	!coarse_extent.cmplt(UVec3::ONE).any() && !coarse_extent.cmpgt(UVec3::splat(CHUNK_SIZE as u32)).any()
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::TileClassId;

	fn tile(size: IVec3, lod: u8) -> TileKey {
		TileKey::new(IVec3::ZERO, size.as_uvec3(), lod, TileClassId(0)).unwrap()
	}

	#[test]
	fn tile_validity_keeps_size_and_lod_independent_with_one_chunk_output_limit() {
		assert!(valid_tile_key(tile(IVec3::ONE, 0)));
		assert!(!valid_tile_key(tile(IVec3::splat(2), 0)));
		assert!(valid_tile_key(tile(IVec3::splat(2), 1)));
	}

	#[test]
	fn tile_class_is_part_of_shared_store_identity() {
		let requester = Entity::from_bits(1);
		let mut streaming = GridStreaming::default();
		let first = tile(IVec3::ONE, 0);
		let second = TileKey { class: TileClassId(1), ..first };
		assert!(streaming.fetch_tile(requester, first, 0.0));
		assert!(streaming.fetch_tile(requester, second, 0.0));
		assert_eq!(streaming.tiles.len(), 2);
	}
}
