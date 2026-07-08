use std::collections::{HashMap, HashSet};

use bevy::math::IVec3;
use bevy::prelude::*;
use voxel_sources::VoxelSourceRequestApi;

use voxel_data::grid::GridId;
use voxel_edit::GridEdit;

use crate::chunk::CHUNK_SIZE;
use crate::consumer::ChunkConsumer;
use crate::lod_index::LodIndex;
use crate::{ChunkLoadRequest, LodKey};
use crate::presence::{ChunkPresence, ChunkState};

const CLEAR_DELAY_FRAMES: u8 = 20;

#[derive(Clone, Debug)]
pub(crate) struct LodTileState {
	pub(crate) requesters: HashMap<Entity, f32>,
	pub(crate) status: LodStatus,
	pub(crate) upload: LodUploadState,
	pub(crate) stale_entities: Vec<Entity>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum LodStatus {
	Requested,
	InFlight,
	Loaded,
	ExternalDirty,
	ExternalDirtyInFlight { generation: u64 },
	Empty,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum LodUploadState {
	None,
	Uploading { entity: Entity, generation: u64, active: Option<Entity> },
	Active { entity: Entity, generation: u64 },
}

#[derive(Component, Default)]
pub struct GridStreaming {
	pub(crate) presence: ChunkPresence,
	pub(crate) dirty_generations: HashMap<IVec3, u64>,
	pub(crate)pending_clears: Vec<(IVec3, u8)>,
	pub(crate) stalled_edits: HashMap<IVec3, Vec<GridEdit>>,
	pub(crate) stalled_pinned: HashSet<IVec3>,
	pub(crate) newly_dirty: Vec<IVec3>,
	pub(crate) newly_present_dirty: Vec<IVec3>,
	pub(crate)lods: HashMap<LodKey, LodTileState>,
	pub(crate)pending_lod_requests: HashSet<LodKey>,
	pub(crate)lod_index: LodIndex,
	pub(crate)uploading_lods_by_entity: HashMap<Entity, LodKey>,
}

#[derive(Component, Debug, Default)]
pub struct RequestChunkPresence;

#[derive(Component, Debug, Default)]
pub struct InflightChunkPresence;

impl GridStreaming {
	pub fn presence(&self) -> &ChunkPresence { &self.presence }
	#[cfg(test)]
	pub fn presence_mut(&mut self) -> &mut ChunkPresence { &mut self.presence }


	pub fn mark_present(&mut self, chunk: IVec3) { self.presence.mark_present(chunk); }
	pub fn mark_present_area(&mut self, min: IVec3, size: IVec3) { self.presence.mark_present_area(min, size); }

	pub fn state(&self, chunk: IVec3) -> Option<ChunkState> {
		self.presence.state(chunk)
	}

	pub fn is_loaded(&self, chunk: IVec3) -> bool {
		matches!(self.presence.state(chunk), Some(ChunkState::Loaded))
	}

	fn start_request(&mut self, grid: GridId, requests: &impl VoxelSourceRequestApi, chunk: IVec3) -> bool {
		match self.presence.state(chunk) {
			None => return false,
			Some(ChunkState::Available) => {
				self.presence.set_state(chunk, ChunkState::InFlight);
				requests.request_chunk(ChunkLoadRequest { grid, chunk });
			}
			Some(ChunkState::ExternalDirty) => {
				self.presence.set_state(chunk, ChunkState::ExternalDirtyInFlight);
				requests.request_chunk(ChunkLoadRequest { grid, chunk });
			}
			_ => {}
		}
		self.presence.add_request(chunk);
		true
	}

	pub fn fetch(&mut self, grid: GridId, requests: &impl VoxelSourceRequestApi, chunk: IVec3) {
		self.start_request(grid, requests, chunk);
	}

	pub fn fetch_needed<C: ChunkConsumer>(
		&mut self,
		grid: GridId,
		consumer: &mut C,
		requests: &impl VoxelSourceRequestApi,
		chunk: IVec3,
	) {
		if !self.start_request(grid, requests, chunk) { return; }
		let resident = matches!(self.presence.state(chunk), Some(ChunkState::Loaded | ChunkState::InternalDirty));
		if consumer.needed_mut().entry(grid).or_default().insert(chunk) && !resident {
			*consumer.outstanding_mut() += 1;
		}
	}

	pub fn release(&mut self, chunk: IVec3) {
		if self.presence.remove_request(chunk) > 0 { return; }
		if let Some(ChunkState::Loaded | ChunkState::InternalDirty) = self.presence.state(chunk) {
			self.pending_clears.push((chunk, CLEAR_DELAY_FRAMES));
		}
	}

	// Fine to call if the chunk was requested with `fetch`
	pub fn release_needed<C: ChunkConsumer>(
		&mut self,
		grid: GridId,
		consumer: &mut C,
		chunk: IVec3,
	) {
		let resident = matches!(self.presence.state(chunk), Some(ChunkState::Loaded | ChunkState::InternalDirty));
		let removed = consumer.needed_mut().get_mut(&grid).is_some_and(|set| set.remove(&chunk));
		if consumer.needed().get(&grid).is_some_and(|set| set.is_empty()) {
			consumer.needed_mut().remove(&grid);
		}
		if removed && !resident {
			*consumer.outstanding_mut() = consumer.outstanding().saturating_sub(1);
		}
		self.release(chunk);
	}

	pub fn fetch_lod(&mut self, requester: Entity, key: LodKey, priority: f32) -> bool {
		if !valid_lod_key(key) { return false; }
		if !self.lods.contains_key(&key) { self.lod_index.insert(key); }
		let state = self.lods.entry(key).or_insert_with(|| LodTileState {
			requesters: HashMap::new(),
			status: LodStatus::Requested,
			upload: LodUploadState::None,
			stale_entities: Vec::new(),
		});
		state.requesters.insert(requester, priority);
		if matches!(state.status, LodStatus::Requested | LodStatus::ExternalDirty) { self.pending_lod_requests.insert(key); }
		true
	}

	pub fn release_lod(&mut self, requester: Entity, key: LodKey) {
		let Some(state) = self.lods.get_mut(&key) else { return; };
		state.requesters.remove(&requester);
		if state.requesters.is_empty() {
			self.pending_lod_requests.remove(&key);
			self.lod_index.remove(key);
		}
	}

	pub(crate) fn current_chunk_generation(&self, chunk: IVec3) -> u64 {
		self.dirty_generations.get(&chunk).copied().unwrap_or(0)
	}

	pub(crate) fn mark_loaded(&mut self, chunk: IVec3) {
		self.presence.set_state(chunk, ChunkState::Loaded);
		self.dirty_generations.remove(&chunk);
	}

	pub(crate) fn mark_empty(&mut self, min: IVec3, size: IVec3) {
		for x in min.x..min.x + size.x {
			for y in min.y..min.y + size.y {
				for z in min.z..min.z + size.z {
					self.dirty_generations.remove(&IVec3::new(x, y, z));
				}
			}
		}
		self.presence.clear_present_area(min, size);
	}

	pub(crate) fn mark_external_changed(&mut self, min: IVec3, size: IVec3, generation: u64) {
		for x in min.x..min.x + size.x {
			for y in min.y..min.y + size.y {
				for z in min.z..min.z + size.z {
					let chunk = IVec3::new(x, y, z);
					match self.presence.state(chunk) {
						Some(ChunkState::Loaded) | Some(ChunkState::InternalDirty) | Some(ChunkState::Available) => {
							self.presence.set_state(chunk, ChunkState::ExternalDirty);
							self.newly_dirty.push(chunk);
						}
						Some(ChunkState::InFlight) | Some(ChunkState::ExternalDirtyInFlight) => {
							self.presence.set_state(chunk, ChunkState::ExternalDirtyInFlight);
							self.dirty_generations
								.entry(chunk)
								.and_modify(|current| *current = (*current).max(generation))
								.or_insert(generation);
						}
						Some(ChunkState::ExternalDirty) => {
							self.newly_dirty.push(chunk);
						}
						None => {
							self.presence.mark_present(chunk);
							self.presence.set_state(chunk, ChunkState::ExternalDirty);
							self.newly_dirty.push(chunk);
						}
					}
				}
			}
		}
	}

	pub(crate) fn refetch(&mut self, grid: GridId, requests: &impl VoxelSourceRequestApi, chunk: IVec3) {
		if self.presence.request_count(chunk) == 0 { return; }
		if matches!(self.presence.state(chunk), Some(ChunkState::ExternalDirty)) {
			self.presence.set_state(chunk, ChunkState::ExternalDirtyInFlight);
			requests.request_chunk(ChunkLoadRequest { grid, chunk });
		}
	}

	pub(crate) fn dirty_lods_covering(&mut self, chunk: IVec3, generation: Option<u64>) {
		for key in self.lod_index.lods_covering_chunk(chunk) {
			let Some(state) = self.lods.get_mut(&key) else { continue };
			if state.requesters.is_empty() { continue; }
			state.status = match (state.status, generation) {
				(LodStatus::InFlight, Some(generation)) => LodStatus::ExternalDirtyInFlight { generation },
				(LodStatus::ExternalDirtyInFlight { generation: current }, Some(generation)) => LodStatus::ExternalDirtyInFlight { generation: current.max(generation) },
				(LodStatus::Loaded, _) | (LodStatus::Empty, _) | (LodStatus::ExternalDirty, _) => LodStatus::ExternalDirty,
				(LodStatus::Requested, _) => LodStatus::Requested,
				(other, None) => other,
			};
			if matches!(state.status, LodStatus::ExternalDirty) {
				self.pending_lod_requests.insert(key);
			}
		}
	}
}

fn valid_lod_key(key: LodKey) -> bool {
	if key.lod == 0 || key.size.cmple(IVec3::ZERO).any() { return false; }
	let factor = 1i32 << key.lod;
	let coarse_extent = (key.size * CHUNK_SIZE) / factor;
	!coarse_extent.cmplt(IVec3::ONE).any() && !coarse_extent.cmpgt(IVec3::splat(CHUNK_SIZE)).any()
}
