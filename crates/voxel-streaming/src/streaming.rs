use std::collections::{HashMap, HashSet};

use bevy::ecs::message::{MessageReader, MessageWriter};
use bevy::math::IVec3;
use bevy::prelude::*;
use voxel_gpu::{LodVoxels, VoxelGpuUploadFinished};
use voxel_sources::{ChunkChangeKind, ChunkChanged, ChunkLoaded, LodLoaded, VoxelSourceRequestApi, VoxelSourceRequests, VoxelSources};

use tracy_client::span;

use voxel_data::grid::{reconcile_subgrids, Grid, GridId};
use voxel_data::splat::{splat_voxels_blocking, GridSplat};
use voxel_data::subgrid::SubGrid;
use voxel_edit::{EditGate, GridEdit, GridEdits};

use crate::chunk::{chunk_of, chunk_origin, CHUNK_SIZE};
use crate::consumer::ChunkConsumer;
use crate::lod_index::LodIndex;
use crate::{ChunkLoadRequest, ChunkSaveChannel, ChunkSaveRequest, LodKey, LodLoadRequest, PresenceLoadRequest};
use crate::{ChunkAvailabilityChangeKind, ChunkAvailabilityChanged, ChunkLoadResolved, LodLoadResult};
use crate::presence::{ChunkPresence, ChunkState};

const CLEAR_DELAY_FRAMES: u8 = 20;

#[derive(Clone, Debug)]
struct LodTileState {
	requesters: HashMap<Entity, f32>,
	status: LodStatus,
	upload: LodUploadState,
	stale_entities: Vec<Entity>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LodStatus {
	Requested,
	InFlight,
	Loaded,
	ExternalDirty,
	ExternalDirtyInFlight { generation: u64 },
	Empty,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LodUploadState {
	None,
	Uploading { entity: Entity, generation: u64, active: Option<Entity> },
	Active { entity: Entity, generation: u64 },
}

#[derive(Component, Default)]
pub struct GridStreaming {
	presence: ChunkPresence,
	dirty_generations: HashMap<IVec3, u64>,
	pending_clears: Vec<(IVec3, u8)>,
	stalled_edits: HashMap<IVec3, Vec<GridEdit>>,
	stalled_pinned: HashSet<IVec3>,
	newly_dirty: Vec<IVec3>,
	newly_present_dirty: Vec<IVec3>,
	lods: HashMap<LodKey, LodTileState>,
	pending_lod_requests: HashSet<LodKey>,
	lod_index: LodIndex,
	uploading_lods_by_entity: HashMap<Entity, LodKey>,
}

#[derive(Component, Debug, Default)]
pub struct RequestChunkPresence;

#[derive(Component, Debug, Default)]
pub struct InflightChunkPresence;

impl GridStreaming {
	pub fn presence(&self) -> &ChunkPresence { &self.presence }
	pub fn presence_mut(&mut self) -> &mut ChunkPresence { &mut self.presence }

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

	fn current_chunk_generation(&self, chunk: IVec3) -> u64 {
		self.dirty_generations.get(&chunk).copied().unwrap_or(0)
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

	fn mark_loaded(&mut self, chunk: IVec3) {
		self.presence.set_state(chunk, ChunkState::Loaded);
		self.dirty_generations.remove(&chunk);
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

	fn mark_empty(&mut self, min: IVec3, size: IVec3) {
		for x in min.x..min.x + size.x {
			for y in min.y..min.y + size.y {
				for z in min.z..min.z + size.z {
					self.dirty_generations.remove(&IVec3::new(x, y, z));
				}
			}
		}
		self.presence.clear_present_area(min, size);
	}

	pub fn mark_external_changed(&mut self, min: IVec3, size: IVec3, generation: u64) {
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

	fn refetch(&mut self, grid: GridId, requests: &impl VoxelSourceRequestApi, chunk: IVec3) {
		if self.presence.request_count(chunk) == 0 { return; }
		if matches!(self.presence.state(chunk), Some(ChunkState::ExternalDirty)) {
			self.presence.set_state(chunk, ChunkState::ExternalDirtyInFlight);
			requests.request_chunk(ChunkLoadRequest { grid, chunk });
		}
	}

	fn dirty_lods_covering(&mut self, chunk: IVec3, generation: Option<u64>) {
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

pub fn apply_source_events(
	mut changed_events: MessageReader<ChunkChanged>,
	mut grids: Query<&mut GridStreaming>,
	mut availability_events: MessageWriter<ChunkAvailabilityChanged>,
) {
	for event in changed_events.read().copied() {
		if let Ok(mut s) = grids.get_mut(event.grid) {
			let generation = match event.kind {
				ChunkChangeKind::Changed { generation } | ChunkChangeKind::Removed { generation } => generation,
			};
			for x in event.min.x..event.min.x + event.size.x {
				for y in event.min.y..event.min.y + event.size.y {
					for z in event.min.z..event.min.z + event.size.z {
						s.dirty_lods_covering(IVec3::new(x, y, z), Some(generation));
					}
				}
			}
		}
		if event.from_save { continue; }
		match event.kind {
			ChunkChangeKind::Changed { generation } => {
				let Ok(mut s) = grids.get_mut(event.grid) else {
					warn!(grid=?event.grid, min=?event.min, size=?event.size, "apply_source_events missing GridStreaming for changed event");
					continue;
				};
				availability_events.write(ChunkAvailabilityChanged {
					grid: event.grid,
					min: event.min,
					size: event.size,
					kind: ChunkAvailabilityChangeKind::BecamePresent,
				});
				s.mark_external_changed(event.min, event.size, generation);
			}
			ChunkChangeKind::Removed { .. } => {
				let Ok(mut s) = grids.get_mut(event.grid) else {
					warn!(grid=?event.grid, min=?event.min, size=?event.size, "apply_source_events missing GridStreaming for removed event");
					continue;
				};
				let mut any_removed = false;
				for x in event.min.x..event.min.x + event.size.x {
					for y in event.min.y..event.min.y + event.size.y {
						for z in event.min.z..event.min.z + event.size.z {
							let chunk = IVec3::new(x, y, z);
							let was_present = s.presence().is_present(chunk);
							if was_present {
								// TODO: this should also clear the grid data
								s.mark_empty(chunk, IVec3::ONE);
								any_removed = true;
							}
						}
					}
				}
				if any_removed {
					availability_events.write(ChunkAvailabilityChanged {
						grid: event.grid,
						min: event.min,
						size: event.size,
						kind: ChunkAvailabilityChangeKind::BecameEmpty,
					});
				}
			}
			
		}
	}
}

pub fn request_presence_for_new_grids(
	requests: VoxelSourceRequests,
	mut commands: Commands,
	grids: Query<(Entity, GridId), (With<RequestChunkPresence>, Without<InflightChunkPresence>)>,
) {
	for (entity, grid) in &grids {
		commands.entity(entity).insert(InflightChunkPresence);
		requests.request_presence(PresenceLoadRequest { grid });
	}
}

pub fn receive_chunk_presence_loaded(
	mut commands: Commands,
	mut loaded: MessageReader<voxel_sources::ChunkPresenceLoaded>,
) {
	for event in loaded.read() {
		let mut entity = commands.entity(event.grid);
		entity.remove::<InflightChunkPresence>();
		entity.remove::<RequestChunkPresence>();
	}
}

pub fn serve_saves(
	saves: Res<ChunkSaveChannel>,
	sources: VoxelSources,
) {
	while let Some(save) = saves.try_recv() {
		sources.route_save(save.grid, save.chunk, &save.voxels);
	}
}

impl EditGate for GridStreaming {
	fn admit(&mut self, edit: &GridEdit) -> bool {
		let (min, max) = edit.voxel_bounds();
		let chunk_min = chunk_of(min);
		let chunk_max = chunk_of(max - IVec3::ONE);
		let mut blocked_chunks = Vec::new();
		for z in chunk_min.z..=chunk_max.z {
			for y in chunk_min.y..=chunk_max.y {
				for x in chunk_min.x..=chunk_max.x {
					let chunk = IVec3::new(x, y, z);
					if matches!(
						self.presence.state(chunk),
						Some(ChunkState::Available | ChunkState::ExternalDirty | ChunkState::ExternalDirtyInFlight)
					) {
						blocked_chunks.push(chunk);
					}
				}
			}
		}
		if blocked_chunks.is_empty() {
			return true;
		}
		for chunk in blocked_chunks {
			self.stalled_edits.entry(chunk).or_default().push(edit.clone());
		}
		false
	}

	fn touched(&mut self, edit: &GridEdit) {
		let (min, max) = edit.voxel_bounds();
		let chunk_min = chunk_of(min);
		let chunk_max = chunk_of(max - IVec3::ONE);
		for z in chunk_min.z..=chunk_max.z {
			for y in chunk_min.y..=chunk_max.y {
				for x in chunk_min.x..=chunk_max.x {
					let chunk = IVec3::new(x, y, z);
					let was_absent = self.presence.state(chunk).is_none();
					if let None | Some(ChunkState::Loaded) | Some(ChunkState::InternalDirty) = self.presence.state(chunk) {
						self.presence.set_state(chunk, ChunkState::InternalDirty);
						self.newly_dirty.push(chunk);
						if was_absent {
							self.newly_present_dirty.push(chunk);
						}
					}
				}
			}
		}
	}
}

pub fn receive_results(
	mut commands: Commands,
	mut channel: MessageReader<ChunkLoaded>,
	mut grids: Query<(&mut GridStreaming, &mut Grid, Option<&mut GridEdits>)>,
	mut sub_grids: Query<&mut SubGrid>,
	mut consumers: Query<&mut dyn ChunkConsumer>,
	mut availability_events: MessageWriter<ChunkAvailabilityChanged>,
	mut chunk_resolved: MessageWriter<ChunkLoadResolved>,
) {
	let pending: Vec<_> = channel.read().cloned().collect();
	let mut loaded_by_grid: HashMap<GridId, Vec<(IVec3, voxel_data::voxels::Voxels)>> = HashMap::new();

	for result in pending {
		let _zone = span!("receive result");
		let result_grid = result.grid;
		let result_chunk = result.chunk;
		let was_empty = result.voxels.is_none();
		let mut became_empty = false;
		let mut accepted = false;
		if let Ok((mut streaming, _grid, mut edits)) = grids.get_mut(result.grid) {
			let state = streaming.presence.state(result.chunk);
			match state {
				Some(ChunkState::InFlight) | Some(ChunkState::ExternalDirtyInFlight) => {
					let stale = result.generation < streaming.current_chunk_generation(result.chunk);
					if stale {
						streaming.presence.set_state(result.chunk, ChunkState::ExternalDirty);
						streaming.newly_dirty.push(result.chunk);
					} else {
						accepted = true;
						match result.voxels {
							Some(_) if streaming.presence.request_count(result.chunk) == 0 => {
								streaming.presence.set_state(result.chunk, ChunkState::Available);
								streaming.dirty_generations.remove(&result.chunk);
							}
							Some(voxels) => {
								streaming.mark_loaded(result.chunk);
								loaded_by_grid.entry(result.grid).or_default().push((result.chunk, voxels));
							}
							None => {
								streaming.mark_empty(result.chunk, IVec3::ONE);
								streaming.replay_stalled(result.chunk, &mut edits);
								became_empty = true;
							}
						}
					}
				}
				_ => { /* chunk was not for voxel-streaming */ }
			}
		}
		if became_empty {
			availability_events.write(ChunkAvailabilityChanged {
				grid: result_grid,
				min: result_chunk,
				size: IVec3::ONE,
				kind: ChunkAvailabilityChangeKind::BecameEmpty,
			});
		}
		if accepted && was_empty {
			chunk_resolved.write(ChunkLoadResolved { grid: result_grid, chunk: result_chunk, visible: false });
		}

		if !accepted { continue; }

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
	mut channel: MessageReader<LodLoaded>,
	mut grids: Query<&mut GridStreaming>,
	mut consumers: Query<&mut dyn ChunkConsumer>,
) {
	let mut updates = Vec::new();
	for mut result in channel.read().cloned() {
		let Ok(mut streaming) = grids.get_mut(result.grid) else { continue };
		let Some(state) = streaming.lods.get_mut(&result.key) else { continue };
		if let LodStatus::ExternalDirtyInFlight { generation } = state.status {
			if result.generation < generation {
				state.status = LodStatus::ExternalDirty;
				streaming.pending_lod_requests.insert(result.key);
				continue;
			}
		}
		let prior_upload = state.upload;
		let requesters: Vec<_> = state.requesters.iter().map(|(&requester, &priority)| (requester, priority)).collect();
		match result.voxels.take() {
			Some(voxels) if !voxels.is_empty() => {
				let entity = commands.spawn((
					LodVoxels { voxels, lod: result.key.lod as f32, priority: result.priority },
					Transform::from_translation((result.key.min * CHUNK_SIZE).as_vec3()),
					ChildOf(result.grid),
				)).id();
				let active = match prior_upload {
					LodUploadState::Uploading { entity, active, .. } => {
						commands.entity(entity).despawn();
						active
					}
					LodUploadState::Active { entity, .. } => Some(entity),
					LodUploadState::None => None,
				};
				state.upload = LodUploadState::Uploading { entity, generation: result.generation, active };
				state.status = LodStatus::Loaded;
			}
			_ => {
				match prior_upload {
					LodUploadState::Active { entity, .. } => commands.entity(entity).despawn(),
					LodUploadState::Uploading { entity, .. } => commands.entity(entity).despawn(),
					LodUploadState::None => {}
				}
				state.upload = LodUploadState::None;
				state.status = LodStatus::Empty;
				for (requester, _) in requesters {
					let mut update = result.clone();
					update.requester = requester;
					update.entity = None;
					updates.push(update);
				}
			}
		}
		let current_upload = state.upload;
		let _ = state;
		if let LodUploadState::Uploading { entity, .. } = prior_upload {
			streaming.uploading_lods_by_entity.remove(&entity);
		}
		if let LodUploadState::Uploading { entity, .. } = current_upload {
			streaming.uploading_lods_by_entity.insert(entity, result.key);
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
	for entity in gpu_events.read().map(|event| event.entity) {
		for (grid, mut streaming) in &mut grids {
			let Some(key) = streaming.uploading_lods_by_entity.remove(&entity) else { continue };
			let Some(state) = streaming.lods.get_mut(&key) else { continue };
			let generation = match state.upload {
				LodUploadState::Uploading { entity, generation, active } => {
					if let Some(old) = active {
						state.stale_entities.push(old);
					}
					state.upload = LodUploadState::Active { entity, generation };
					generation
				}
				_ => continue,
			};
			state.status = LodStatus::Loaded;
			for &requester in state.requesters.keys() {
				updates.push(LodLoadResult { grid, requester, key, priority: state.requesters[&requester], generation, voxels: None, entity: Some(entity) });
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
			match state.upload {
				LodUploadState::Active { entity, .. } => commands.entity(entity).despawn(),
				LodUploadState::Uploading { entity, .. } => {
					streaming.uploading_lods_by_entity.remove(&entity);
					commands.entity(entity).despawn();
				}
				LodUploadState::None => {}
			}
		}
	}
}

pub fn request_lod_tiles(
	requests: VoxelSourceRequests,
	mut grids: Query<(GridId, &mut GridStreaming)>,
) {
	for (grid, mut streaming) in grids.iter_mut() {
		let pending: Vec<_> = std::mem::take(&mut streaming.pending_lod_requests).into_iter().collect();
		for key in pending {
			let Some(state) = streaming.lods.get_mut(&key) else { continue };
			if state.requesters.is_empty() { continue; }
			if !matches!(state.status, LodStatus::Requested | LodStatus::ExternalDirty) { continue; }
			let requester = *state.requesters.keys().next().unwrap();
			let priority = state.requesters.values().copied().fold(f32::NEG_INFINITY, f32::max);
			state.status = LodStatus::InFlight;
			requests.request_lod(LodLoadRequest { grid, requester, key, priority });
		}
	}
}

pub fn handle_dirty_chunks(
	requests: VoxelSourceRequests,
	save_channel: Res<ChunkSaveChannel>,
	mut grids: Query<(GridId, &mut GridStreaming, &Grid)>,
	mut consumers: Query<&mut dyn ChunkConsumer>,
	mut availability_events: MessageWriter<ChunkAvailabilityChanged>,
) {
	for (grid, mut streaming, grid_data) in grids.iter_mut() {
		let newly_present_dirty: HashSet<_> = std::mem::take(&mut streaming.newly_present_dirty).into_iter().collect();
		for chunk in newly_present_dirty {
			availability_events.write(ChunkAvailabilityChanged {
				grid,
				min: chunk,
				size: IVec3::ONE,
				kind: ChunkAvailabilityChangeKind::BecamePresent,
			});
		}
		if streaming.newly_dirty.is_empty() { continue; }
		let dirty: HashSet<_> = std::mem::take(&mut streaming.newly_dirty).into_iter().collect();
		for chunk in dirty {
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
			streaming.refetch(grid, &requests, chunk);
		}
	}
}

pub fn request_stalled_chunks(
	requests: VoxelSourceRequests,
	mut grids: Query<(GridId, &mut GridStreaming)>,
) {
	for (grid, mut streaming) in grids.iter_mut() {
		if streaming.stalled_edits.is_empty() { continue; }
		let chunks: Vec<IVec3> = streaming.stalled_edits.keys().copied().collect();
		for chunk in chunks {
			if matches!(streaming.presence.state(chunk), Some(ChunkState::Available))
				&& streaming.stalled_pinned.insert(chunk)
			{
				streaming.fetch(grid, &requests, chunk);
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
