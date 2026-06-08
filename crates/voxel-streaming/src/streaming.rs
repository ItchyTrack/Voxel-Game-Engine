use std::collections::{HashMap, HashSet};

use bevy::math::IVec3;
use bevy::prelude::*;

use voxel_data::grid::{reconcile_subgrids, Grid, GridId};
use voxel_data::subgrid::SubGrid;
use voxel_edit::{EditGate, GridEdit, GridEdits};

use crate::chunk::{chunk_of, chunk_origin, CHUNK_SIZE};
use crate::consumer::ChunkConsumer;
use crate::loader::{ChunkLoadRequest, ChunkLoaderChannel, ChunkRequestChannel, ChunkSaveChannel, ChunkSaveRequest};
use crate::presence::{ChunkPresence, ChunkState};

const CLEAR_DELAY_FRAMES: u8 = 20;

#[derive(Component, Default)]
pub struct GridStreaming {
	presence: ChunkPresence,
	pending_clears: Vec<(IVec3, u8)>,
	stalled_edits: HashMap<IVec3, Vec<GridEdit>>,
	stalled_pinned: HashSet<IVec3>,
}

impl GridStreaming {
	pub fn presence(&self) -> &ChunkPresence { &self.presence }
	pub fn presence_mut(&mut self) -> &mut ChunkPresence { &mut self.presence }

	pub fn state(&self, chunk: IVec3) -> Option<ChunkState> {
		self.presence.state(chunk)
	}

	pub fn is_loaded(&self, chunk: IVec3) -> bool {
		matches!(self.presence.state(chunk), Some(ChunkState::Loaded))
	}

	fn start_request(&mut self, grid: GridId, channel: &ChunkRequestChannel, chunk: IVec3) -> bool {
		match self.presence.state(chunk) {
			None => return false,
			Some(ChunkState::Available) => {
				self.presence.set_state(chunk, ChunkState::InFlight);
				channel.request(ChunkLoadRequest { grid, chunk });
			}
			Some(ChunkState::ExternalDirty) => {
				self.presence.set_state(chunk, ChunkState::ExternalDirtyInFlight);
				channel.request(ChunkLoadRequest { grid, chunk });
			}
			_ => {}
		}
		self.presence.add_request(chunk);
		true
	}

	pub fn fetch(&mut self, grid: GridId, channel: &ChunkRequestChannel, chunk: IVec3) {
		self.start_request(grid, channel, chunk);
	}

	pub fn fetch_needed<C: ChunkConsumer>(
		&mut self,
		grid: GridId,
		consumer: &mut C,
		channel: &ChunkRequestChannel,
		chunk: IVec3,
	) {
		if !self.start_request(grid, channel, chunk) { return; }
		if !matches!(self.presence.state(chunk), Some(ChunkState::Loaded | ChunkState::InternalDirty)) {
			consumer.needed_mut().entry(grid).or_default().insert(chunk);
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
		if let Some(set) = consumer.needed_mut().get_mut(&grid) {
			set.remove(&chunk);
			if set.is_empty() { consumer.needed_mut().remove(&grid); }
		}
		self.release(chunk);
	}

	fn mark_loaded(&mut self, chunk: IVec3) {
		self.presence.set_state(chunk, ChunkState::Loaded);
	}

	fn replay_stalled(&mut self, chunk: IVec3, edits: &mut Option<Mut<GridEdits>>) {
		let Some(stalled) = self.stalled_edits.remove(&chunk) else { return; };
		if let Some(edits) = edits.as_mut() {
			for edit in stalled {
				edits.push_edit(edit);
			}
		}
		if self.stalled_pinned.remove(&chunk) {
			self.release(chunk);
		}
	}

	fn mark_empty(&mut self, chunk: IVec3) {
		self.presence.clear_present(chunk);
	}
}

impl EditGate for GridStreaming {
	fn admit(&mut self, edit: &GridEdit) -> bool {
		let chunk = chunk_of(edit.voxel_pos());
		match self.presence.state(chunk) {
			Some(ChunkState::Available) => {
				self.stalled_edits.entry(chunk).or_default().push(*edit);
				false
			}
			Some(ChunkState::ExternalDirty) | Some(ChunkState::ExternalDirtyInFlight) => false,
			_ => true,
		}
	}

	fn touched(&mut self, voxel_pos: IVec3) {
		let chunk = chunk_of(voxel_pos);
		// `None` means the chunk had no presence entry: editing creates it, so
		// mark it present and dirty so it is tracked and saved on unload.
		if let None | Some(ChunkState::Loaded) | Some(ChunkState::InternalDirty) = self.presence.state(chunk) {
			self.presence.set_state(chunk, ChunkState::InternalDirty);
		}
	}
}

pub fn receive_results(
	mut commands: Commands,
	channel: Res<ChunkLoaderChannel>,
	mut grids: Query<(&mut GridStreaming, &mut Grid, Option<&mut GridEdits>)>,
	mut sub_grids: Query<&mut SubGrid>,
	mut consumers: Query<&mut dyn ChunkConsumer>,
) {
	while let Some(result) = channel.try_recv() {
		if let Ok((mut streaming, mut grid, mut edits)) = grids.get_mut(result.grid) {
			match streaming.presence.state(result.chunk) {
				Some(ChunkState::InFlight) | Some(ChunkState::ExternalDirtyInFlight) => {
					match result.voxels {
						Some(_) if streaming.presence.request_count(result.chunk) == 0 => {
							streaming.presence.set_state(result.chunk, ChunkState::Available);
						}
						Some(voxels) => {
							streaming.mark_loaded(result.chunk);
							let base = chunk_origin(result.chunk);
							let touched = grid.splat_voxels(base, &voxels);
							reconcile_subgrids(result.grid, grid.as_mut(), touched, &mut commands, &mut sub_grids);
							streaming.replay_stalled(result.chunk, &mut edits);
						}
						None => {
							streaming.mark_empty(result.chunk);
							streaming.replay_stalled(result.chunk, &mut edits);
						}
					}
				}
				_ => {
					bevy::log::warn!("receive_results received a chunk without anyone requesting it!");
				}
			}
		}
		for mut entity_consumers in consumers.iter_mut() {
			for mut consumer in &mut entity_consumers {
				let needed = consumer.needed_mut();
				let now_empty = match needed.get_mut(&result.grid) {
					Some(set) => {
						set.remove(&result.chunk);
						set.is_empty()
					}
					None => false,
				};
				if now_empty {
					needed.remove(&result.grid);
				}
			}
		}
	}
}

pub fn request_stalled_chunks(
	channel: Res<ChunkRequestChannel>,
	mut grids: Query<(GridId, &mut GridStreaming)>,
) {
	for (grid, mut streaming) in grids.iter_mut() {
		if streaming.stalled_edits.is_empty() { continue; }
		let chunks: Vec<IVec3> = streaming.stalled_edits.keys().copied().collect();
		for chunk in chunks {
			if matches!(streaming.presence.state(chunk), Some(ChunkState::Available))
				&& streaming.stalled_pinned.insert(chunk)
			{
				streaming.fetch(grid, &channel, chunk);
			}
		}
	}
}

pub fn flush_dirty_chunks(
	save_channel: Res<ChunkSaveChannel>,
	mut grids: Query<(Entity, &mut GridStreaming, &Grid)>,
) {
	for (grid_entity, mut streaming, grid) in grids.iter_mut() {
		let dirty: Vec<IVec3> = streaming
			.presence()
			.iter_states()
			.filter(|(_, _, state)| matches!(state, ChunkState::InternalDirty))
			.map(|(origin, _, _)| origin)
			.collect();
		for chunk in dirty {
			let voxels = grid.read_area(chunk_origin(chunk), IVec3::splat(CHUNK_SIZE));
			save_channel.save(ChunkSaveRequest { grid: grid_entity, chunk, voxels });
			streaming.presence.set_state(chunk, ChunkState::Loaded);
		}
	}
}

pub fn apply_chunk_clears(
	mut commands: Commands,
	save_channel: Res<ChunkSaveChannel>,
	mut grids: Query<(Entity, &mut GridStreaming, &mut Grid)>,
	mut sub_grids: Query<&mut SubGrid>,
) {
	for (grid_entity, mut streaming, mut grid) in grids.iter_mut() {
		if streaming.pending_clears.is_empty() { continue; }
		let mut still_pending = Vec::new();
		for (chunk, frames) in std::mem::take(&mut streaming.pending_clears) {
			if streaming.presence.request_count(chunk) > 0 { continue; }
			match streaming.presence.state(chunk) {
				Some(ChunkState::Loaded) | Some(ChunkState::ExternalDirty) | Some(ChunkState::ExternalDirtyInFlight) => {
					if frames > 0 {
						still_pending.push((chunk, frames - 1));
						continue;
					}
					let touched = grid.clear_area(chunk_origin(chunk), IVec3::splat(CHUNK_SIZE));
					reconcile_subgrids(grid_entity, grid.as_mut(), touched, &mut commands, &mut sub_grids);
					streaming.presence.set_state(chunk, ChunkState::Available);
				}
				Some(ChunkState::InternalDirty) => {
					if frames > 0 {
						still_pending.push((chunk, frames - 1));
						continue;
					}
					let (touched, voxels) = grid.read_clear_area(chunk_origin(chunk), IVec3::splat(CHUNK_SIZE));
						save_channel.save(ChunkSaveRequest { grid: grid_entity, chunk, voxels });
					reconcile_subgrids(grid_entity, grid.as_mut(), touched, &mut commands, &mut sub_grids);
					streaming.presence.set_state(chunk, ChunkState::Available);
				}
				_ => {}
			}
		}
		streaming.pending_clears = still_pending;
	}
}
