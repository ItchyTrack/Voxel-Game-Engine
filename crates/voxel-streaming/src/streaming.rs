use std::collections::{HashMap, HashSet};

use bevy::ecs::message::{MessageReader, MessageWriter};
use bevy::math::IVec3;
use bevy::prelude::*;
use gpu_voxel_data::{LodVoxels, VoxelGpuUploadFinished};

use tracy_client::span;

use voxel_data::grid::{reconcile_subgrids, Grid, GridId};
use voxel_data::splat::{splat_voxels_blocking, GridSplat};
use voxel_data::subgrid::SubGrid;
use voxel_edit::{EditGate, GridEdit, GridEdits};

use crate::chunk::{chunk_of, chunk_origin, CHUNK_SIZE};
use crate::consumer::ChunkConsumer;
use crate::loader::{ChunkLoadRequest, ChunkLoaderChannel, ChunkRequestChannel, ChunkSaveChannel, ChunkSaveRequest, LodKey, LodLoadRequest, LodLoadResult, LodLoaderChannel, LodRequestChannel};
use crate::lod_index::LodIndex;
use crate::ChunkLoadResolved;
use crate::presence::{ChunkPresence, ChunkState};

const CLEAR_DELAY_FRAMES: u8 = 20;

#[derive(Clone, Debug)]
struct LodTileState {
	requesters: HashMap<Entity, f32>,
	status: LodStatus,
	revision: u64,
	entity: Option<Entity>,
	waiting_entity: Option<Entity>,
	stale_entities: Vec<Entity>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LodStatus { Requested, Loading, Loaded, Empty }

#[derive(Component, Default)]
pub struct GridStreaming {
	presence: ChunkPresence,
	pending_clears: Vec<(IVec3, u8)>,
	stalled_edits: HashMap<IVec3, Vec<GridEdit>>,
	stalled_pinned: HashSet<IVec3>,
	newly_dirty: Vec<IVec3>,
	lods: HashMap<LodKey, LodTileState>,
	pending_lod_requests: HashSet<LodKey>,
	lod_index: LodIndex,
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

	pub fn fetch_lod(&mut self, requester: Entity, key: LodKey, priority: f32) -> bool {
		if !valid_lod_key(key) { return false; }
		if !self.lods.contains_key(&key) { self.lod_index.insert(key); }
		let state = self.lods.entry(key).or_insert_with(|| LodTileState { requesters: HashMap::new(), status: LodStatus::Requested, revision: 0, entity: None, waiting_entity: None, stale_entities: Vec::new() });
		state.requesters.insert(requester, priority);
		if matches!(state.status, LodStatus::Requested) { self.pending_lod_requests.insert(key); }
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

	pub fn fetch_needed<C: ChunkConsumer>(
		&mut self,
		grid: GridId,
		consumer: &mut C,
		channel: &ChunkRequestChannel,
		chunk: IVec3,
	) {
		if !self.start_request(grid, channel, chunk) { return; }
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

	fn mark_loaded(&mut self, chunk: IVec3) {
		self.presence.set_state(chunk, ChunkState::Loaded);
	}

	fn replay_stalled(&mut self, chunk: IVec3, edits: &mut Option<Mut<GridEdits>>) {
		let _zone = span!();
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

	pub fn mark_external_dirty(&mut self, chunk: IVec3) {
		match self.presence.state(chunk) {
			// Resident: consumers had it loaded, so queue them to be re-counted.
			Some(ChunkState::Loaded) | Some(ChunkState::InternalDirty) => {
				self.presence.set_state(chunk, ChunkState::ExternalDirty);
				self.newly_dirty.push(chunk);
			}
			Some(ChunkState::Available) => {
				self.presence.set_state(chunk, ChunkState::ExternalDirty);
			}
			_ => {}
		}
	}

	fn refetch(&mut self, grid: GridId, channel: &ChunkRequestChannel, chunk: IVec3) {
		if self.presence.request_count(chunk) == 0 { return; }
		if matches!(self.presence.state(chunk), Some(ChunkState::ExternalDirty)) {
			self.presence.set_state(chunk, ChunkState::ExternalDirtyInFlight);
			channel.request(ChunkLoadRequest { grid, chunk });
		}
	}

	fn dirty_lods_covering(&mut self, chunk: IVec3) {
		for key in self.lod_index.lods_covering_chunk(chunk) {
			let Some(state) = self.lods.get_mut(&key) else { continue };
			if state.requesters.is_empty() { continue; }
			state.revision += 1;
			state.status = LodStatus::Requested;
			self.pending_lod_requests.insert(key);
		}
	}
}

fn valid_lod_key(key: LodKey) -> bool {
	if key.lod == 0 || key.size.cmple(IVec3::ZERO).any() { return false; }
	let factor = 1i32 << key.lod;
	let coarse_extent = (key.size * CHUNK_SIZE) / factor;
	!coarse_extent.cmplt(IVec3::ONE).any() && !coarse_extent.cmpgt(IVec3::splat(CHUNK_SIZE)).any()
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
			self.newly_dirty.push(chunk);
		}
	}
}

pub fn receive_results(
	mut commands: Commands,
	channel: Res<ChunkLoaderChannel>,
	mut grids: Query<(&mut GridStreaming, &mut Grid, Option<&mut GridEdits>)>,
	mut sub_grids: Query<&mut SubGrid>,
	mut consumers: Query<&mut dyn ChunkConsumer>,
	mut chunk_resolved: MessageWriter<ChunkLoadResolved>,
) {
	let mut loaded_by_grid: HashMap<GridId, Vec<(IVec3, voxel_data::voxels::Voxels)>> = HashMap::new();

	while let Some(result) = channel.try_recv() {
		let _zone = span!("receive result");
		let result_grid = result.grid;
		let result_chunk = result.chunk;
		let was_empty = result.voxels.is_none();
		if let Ok((mut streaming, _grid, mut edits)) = grids.get_mut(result.grid) {
			match streaming.presence.state(result.chunk) {
				Some(ChunkState::InFlight) | Some(ChunkState::ExternalDirtyInFlight) => {
					match result.voxels {
						Some(_) if streaming.presence.request_count(result.chunk) == 0 => {
							streaming.presence.set_state(result.chunk, ChunkState::Available);
						}
						Some(voxels) => {
							streaming.mark_loaded(result.chunk);
							loaded_by_grid.entry(result.grid).or_default().push((result.chunk, voxels));
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
		if was_empty {
			chunk_resolved.write(ChunkLoadResolved { grid: result_grid, chunk: result_chunk, visible: false });
		}

		let _zone = span!("update consumers");
		for mut entity_consumers in consumers.iter_mut() {
			for mut consumer in &mut entity_consumers {
				if !consumer.needed().get(&result.grid).is_some_and(|set| set.contains(&result.chunk)) {
					continue;
				}
				*consumer.outstanding_mut() = consumer.outstanding().saturating_sub(1);
				if was_empty {
					if let Some(set) = consumer.needed_mut().get_mut(&result.grid) {
						set.remove(&result.chunk);
						if set.is_empty() { consumer.needed_mut().remove(&result.grid); }
					}
				}
			}
		}
	}

	for (grid_entity, loaded) in loaded_by_grid {
		let _zone = span!("apply grouped chunk splats");
		let Ok((mut streaming, mut grid, mut edits)) = grids.get_mut(grid_entity) else { continue };
		let splats: Vec<_> = loaded
			.iter()
			.map(|(chunk, voxels)| GridSplat { grid: 0, base: chunk_origin(*chunk), voxels })
			.collect();
		let mut touched_by_grid = splat_voxels_blocking(std::slice::from_mut(grid.as_mut()), &splats);
		let touched = touched_by_grid.remove(&0).unwrap_or_default();
		reconcile_subgrids(grid_entity, grid.as_mut(), touched.iter().copied(), &mut commands, &mut sub_grids);
		for (chunk, _) in loaded {
			streaming.replay_stalled(chunk, &mut edits);
			let min = chunk_origin(chunk);
			let visible = touched
				.iter()
				.any(|subgrid| grid.subgrid_owned_area_intersects(*subgrid, min, IVec3::splat(CHUNK_SIZE)));
			chunk_resolved.write(ChunkLoadResolved { grid: grid_entity, chunk, visible });
		}
	}
}

pub fn receive_lod_results(
	mut commands: Commands,
	channel: Res<LodLoaderChannel>,
	mut grids: Query<&mut GridStreaming>,
	mut consumers: Query<&mut dyn ChunkConsumer>,
) {
	let mut updates = Vec::new();
	while let Some(mut result) = channel.try_recv() {
		let Ok(mut streaming) = grids.get_mut(result.grid) else { continue };
		let Some(state) = streaming.lods.get_mut(&result.key) else { continue };
		if result.generation != state.revision { continue; }
		match result.voxels.take() {
			Some(voxels) if !voxels.is_empty() => {
				let entity = commands.spawn((
					LodVoxels { voxels, lod: result.key.lod as f32, priority: result.priority },
					Transform::from_translation((result.key.min * CHUNK_SIZE).as_vec3()),
					ChildOf(result.grid),
				)).id();
				if let Some(waiting) = state.waiting_entity.replace(entity) { commands.entity(waiting).despawn(); }
				state.status = LodStatus::Loading;
			}
			_ => {
				if let Some(entity) = state.entity.take() { commands.entity(entity).despawn(); }
				if let Some(entity) = state.waiting_entity.take() { commands.entity(entity).despawn(); }
				state.status = LodStatus::Empty;
				for &requester in state.requesters.keys() {
					let mut update = result.clone();
					update.requester = requester;
					update.entity = None;
					updates.push(update);
				}
			}
		}
	}
	for update in updates {
		let Ok(mut entity_consumers) = consumers.get_mut(update.requester) else { continue };
		for mut consumer in &mut entity_consumers { consumer.push_lod(update.clone()); }
	}
}

pub fn refresh_lod_uploads(
	mut gpu_events: MessageReader<VoxelGpuUploadFinished>,
	mut grids: Query<(GridId, &mut GridStreaming)>,
	mut consumers: Query<&mut dyn ChunkConsumer>,
) {
	let mut updates = Vec::new();
	for event in gpu_events.read().copied() {
		for (grid, mut streaming) in &mut grids {
			let Some((&key, state)) = streaming.lods.iter_mut().find(|(_, state)| state.waiting_entity == Some(event.entity)) else { continue };
			if let Some(old) = state.entity.replace(event.entity) { state.stale_entities.push(old); }
			state.waiting_entity = None;
			state.status = LodStatus::Loaded;
			for &requester in state.requesters.keys() {
				updates.push(LodLoadResult { grid, requester, key, priority: state.requesters[&requester], generation: state.revision, voxels: None, entity: Some(event.entity) });
			}
		}
	}
	for update in updates {
		let Ok(mut entity_consumers) = consumers.get_mut(update.requester) else { continue };
		for mut consumer in &mut entity_consumers { consumer.push_lod(update.clone()); }
	}
}

pub fn cleanup_released_lods(mut commands: Commands, mut grids: Query<&mut GridStreaming>) {
	for mut streaming in &mut grids {
		for state in streaming.lods.values_mut() {
			for entity in state.stale_entities.drain(..) { commands.entity(entity).despawn(); }
		}
		let released: Vec<_> = streaming.lods.iter().filter_map(|(&key, state)| state.requesters.is_empty().then_some(key)).collect();
		for key in released {
			let Some(state) = streaming.lods.remove(&key) else { continue };
			if let Some(entity) = state.entity { commands.entity(entity).despawn(); }
			if let Some(entity) = state.waiting_entity { commands.entity(entity).despawn(); }
		}
	}
}

pub fn request_lod_tiles(
	channel: Res<LodRequestChannel>,
	mut grids: Query<(GridId, &mut GridStreaming)>,
) {
	for (grid, mut streaming) in grids.iter_mut() {
		let pending: Vec<_> = std::mem::take(&mut streaming.pending_lod_requests).into_iter().collect();
		for key in pending {
			let Some(state) = streaming.lods.get_mut(&key) else { continue };
			if state.requesters.is_empty() { continue; }
			let requester = *state.requesters.keys().next().unwrap();
			let priority = state.requesters.values().copied().fold(f32::NEG_INFINITY, f32::max);
			state.status = LodStatus::Loading;
			channel.request(LodLoadRequest { grid, requester, key, priority, generation: state.revision });
		}
	}
}

pub fn handle_dirty_chunks(
	load_channel: Res<ChunkRequestChannel>,
	save_channel: Res<ChunkSaveChannel>,
	mut grids: Query<(GridId, &mut GridStreaming, &Grid)>,
	mut consumers: Query<&mut dyn ChunkConsumer>,
) {
	for (grid, mut streaming, grid_data) in grids.iter_mut() {
		if streaming.newly_dirty.is_empty() { continue; }
		let dirty: HashSet<_> = std::mem::take(&mut streaming.newly_dirty).into_iter().collect();
		for chunk in dirty {
			streaming.dirty_lods_covering(chunk);
			if matches!(streaming.presence.state(chunk), Some(ChunkState::InternalDirty)) {
				let voxels = grid_data.read_area(chunk_origin(chunk), IVec3::splat(CHUNK_SIZE));
				save_channel.save(ChunkSaveRequest { grid, chunk, voxels });
				streaming.presence.set_state(chunk, ChunkState::Loaded);
				continue;
			}
			for mut entity_consumers in consumers.iter_mut() {
				for mut consumer in &mut entity_consumers {
					if consumer.needed().get(&grid).is_some_and(|set| set.contains(&chunk)) {
						*consumer.outstanding_mut() += 1;
					}
				}
			}
			streaming.refetch(grid, &load_channel, chunk);
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
