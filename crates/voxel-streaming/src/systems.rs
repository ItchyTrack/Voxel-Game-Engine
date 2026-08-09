use std::{collections::{HashMap, HashSet}, sync::{Arc, Mutex}};

use bevy::ecs::message::{MessageReader, MessageWriter};
use bevy::math::IVec3;
use bevy::prelude::*;
use bevy::tasks::AsyncComputeTaskPool;
use voxel_edit::GridEdits;
use voxel_sources::{ChunkChangeKind, ChunkChanged, ChunkLoaded, SourceManager, VoxelSourcesRequestHandleGetter};
use voxel_tasks::CancellationToken;

use tracy_client::span;

use voxel_data::grid::{reconcile_subgrids, Grid, GridId};
use voxel_data::splat::{splat_voxels_blocking, GridSplat};
use voxel_data::subgrid::SubGrid;

use crate::chunk::{chunk_origin, CHUNK_SIZE};
use crate::consumer::ChunkConsumer;
use crate::generation::{
	TileGenerationCancellation, TileGenerationChannel, TileGenerationMetadata,
	TileGenerationResult, session as generation_session,
};
use crate::streaming::TileStatus;
use crate::{
	DynamicTileData, LoadedTile, StreamingSourceRequestHandle, TileGenerationContext,
	TileGeneratorRegistry, TileLoadStatus, TileLoadUpdate,
};
use crate::{ChunkSaveChannel, ChunkSaveRequest, GridStreaming, InflightChunkPresence, PresenceLoadRequest, RequestChunkPresence};
use crate::{ChunkAvailabilityChangeKind, ChunkAvailabilityChanged, ChunkLoadResolved};
use crate::presence::ChunkState;

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
			s.note_source_generation(generation);
			for x in event.min.x..event.min.x + event.size.x {
				for y in event.min.y..event.min.y + event.size.y {
					for z in event.min.z..event.min.z + event.size.z {
						s.dirty_tiles_covering(IVec3::new(x, y, z));
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
	requests: Res<StreamingSourceRequestHandle>,
	mut commands: Commands,
	grids: Query<(Entity, GridId), (With<RequestChunkPresence>, Without<InflightChunkPresence>)>,
) {
	for (entity, grid) in &grids {
		commands.entity(entity).insert(InflightChunkPresence);
		requests.0.request_presence(PresenceLoadRequest { grid });
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
	sources: Res<SourceManager>,
) {
	while let Some(save) = saves.try_recv() {
		sources.save_chunk(save.grid, save.chunk, &save.voxels);
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
					streaming.finish_chunk_request(result.chunk);
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

#[derive(Resource, Default)]
pub struct PendingTileUpdates(Vec<TileLoadUpdate>);

fn cleanup_tile_entity(world: &mut World, entity: Entity) {
	if let Ok(entity_mut) = world.get_entity_mut(entity) { entity_mut.despawn(); }
}

pub fn receive_tile_results(world: &mut World) {
	let results: Vec<_> = world.resource::<TileGenerationChannel>().drain().collect();
	for result in results {
		let Some(key) = world.get_mut::<GridStreaming>(result.grid).and_then(|mut streaming| streaming.inflight_tiles_by_tag.remove(&result.tag)) else { continue };
		let context_matches = world.get::<TileGenerationContext>(result.grid).is_some_and(|context| context.version() == result.context_version);
		let accepted = {
			let Some(mut streaming) = world.get_mut::<GridStreaming>(result.grid) else { continue };
			let stale_source = result.source_generation.is_some_and(|generation| generation < streaming.latest_source_generation);
			let Some(state) = streaming.tiles.get_mut(&key) else { continue };
			let status = std::mem::replace(&mut state.status, TileStatus::Requested);
			let matching_task = matches!(&status, TileStatus::InFlight { tag, .. } if *tag == result.tag);
			if !matching_task {
				state.status = status;
				false
			} else if !context_matches || stale_source {
				state.status = TileStatus::Dirty;
				streaming.pending_tile_requests.insert(key);
				false
			} else {
				true
			}
		};
		if !accepted { continue; }
		let (requesters, old) = {
			let mut streaming = world.get_mut::<GridStreaming>(result.grid).unwrap();
			streaming.tile_dependencies.replace(key, result.dependencies);
			let state = streaming.tiles.get(&key).unwrap();
			(state.requesters.keys().copied().collect::<Vec<_>>(), state.active)
		};
		match result.data {
			Some(data) => {
				let entity = world.spawn((
					DynamicTileData::new(data),
					LoadedTile { grid: result.grid, key },
					Transform::from_translation((key.min * CHUNK_SIZE).as_vec3()),
					ChildOf(result.grid),
				)).id();
				let mut streaming = world.get_mut::<GridStreaming>(result.grid).unwrap();
				let state = streaming.tiles.get_mut(&key).unwrap();
				state.active = Some(entity);
				state.status = TileStatus::Loaded;
				for requester in requesters {
					world.resource_mut::<PendingTileUpdates>().0.push(TileLoadUpdate { grid: result.grid, requester, key, status: TileLoadStatus::Ready(entity) });
				}
				if let Some(old) = old { cleanup_tile_entity(world, old); }
			}
			None => {
				let mut streaming = world.get_mut::<GridStreaming>(result.grid).unwrap();
				let state = streaming.tiles.get_mut(&key).unwrap();
				state.active = None;
				state.status = TileStatus::Empty;
				for requester in requesters {
					world.resource_mut::<PendingTileUpdates>().0.push(TileLoadUpdate { grid: result.grid, requester, key, status: TileLoadStatus::Empty });
				}
				if let Some(old) = old { cleanup_tile_entity(world, old); }
			}
		}
	}
}

pub fn publish_tile_updates(
	mut updates: ResMut<PendingTileUpdates>,
	mut grids: Query<(GridId, &mut GridStreaming)>,
	mut consumers: Query<&mut dyn ChunkConsumer>,
) {
	for (grid, mut streaming) in &mut grids {
		for mut update in std::mem::take(&mut streaming.queued_tile_updates) {
			update.grid = grid;
			updates.0.push(update);
		}
	}
	for update in std::mem::take(&mut updates.0) {
		let Ok(mut entity_consumers) = consumers.get_mut(update.requester) else { continue };
		for mut consumer in &mut entity_consumers { consumer.push_tile(update); }
	}
}

pub fn cleanup_released_tiles(world: &mut World) {
	let mut released_entities = Vec::new();
	let mut query = world.query::<&mut GridStreaming>();
	for mut streaming in query.iter_mut(world) {
		let released: Vec<_> = streaming.tiles.iter().filter_map(|(&key, state)| state.requesters.is_empty().then_some(key)).collect();
		for key in released {
			streaming.tile_dependencies.remove(key);
			if let Some(state) = streaming.tiles.remove(&key) {
				if let Some(entity) = state.active { released_entities.push(entity); }
			}
		}
	}
	for entity in released_entities { cleanup_tile_entity(world, entity); }
}

pub fn invalidate_changed_generation_contexts(
	mut grids: Query<&mut GridStreaming, Changed<TileGenerationContext>>,
) {
	for mut streaming in &mut grids { streaming.invalidate_generation_context(); }
}

pub(crate) fn request_tiles(
	request_handles: Res<VoxelSourcesRequestHandleGetter>,
	generators: Res<TileGeneratorRegistry>,
	results: Res<TileGenerationChannel>,
	mut grids: Query<(GridId, &Grid, Option<&TileGenerationContext>, &mut GridStreaming)>,
) {
	for (grid, grid_data, context, mut streaming) in grids.iter_mut() {
		let pending: Vec<_> = std::mem::take(&mut streaming.pending_tile_requests).into_iter().collect();
		if !pending.is_empty() {
			assert!(context.is_some(), "grid {grid:?} has tile requests but no generation context");
		}
		let Some(context) = context else { continue };
		for key in pending {
			let Some(state) = streaming.tiles.get(&key) else { continue };
			if state.requesters.is_empty() { continue; }
			if !matches!(&state.status, TileStatus::Requested | TileStatus::Dirty) { continue; }
			let priority = state.requesters.values().copied().fold(f32::NEG_INFINITY, f32::max);
			streaming.next_tile_tag = streaming.next_tile_tag.wrapping_add(1).max(1);
			let tag = streaming.next_tile_tag;
			let context_version = context.version();
			let generator = generators.generator(key.class, grid_data.voxel_type_info().id);
			let cancellation = CancellationToken::new();
			let metadata = Arc::new(Mutex::new(TileGenerationMetadata::default()));
			let (session, wake) = generation_session(
				grid,
				key,
				context.clone(),
				priority,
				request_handles.get(),
				cancellation.clone(),
				metadata.clone(),
			);
			let result_tx = results.sender();
			let task_cancellation = cancellation.clone();
			AsyncComputeTaskPool::get().spawn(async move {
				let data = generator.generate(session).await;
				if task_cancellation.is_cancelled() { return; }
				let mut metadata = metadata.lock().unwrap();
				let dependencies = std::mem::take(&mut metadata.dependencies);
				let source_generation = metadata.source_generation;
				drop(metadata);
				let _ = result_tx.send(TileGenerationResult {
					grid,
					tag,
					context_version,
					source_generation,
					dependencies,
					data,
				});
			}).detach();
			streaming.inflight_tiles_by_tag.insert(tag, key);
			streaming.tiles.get_mut(&key).unwrap().status = TileStatus::InFlight {
				tag,
				cancellation: TileGenerationCancellation::new(cancellation, wake),
			};
		}
	}
}

pub fn handle_dirty_chunks(
	requests: Res<StreamingSourceRequestHandle>,
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
			streaming.refetch(grid, &requests.0, chunk);
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

pub fn request_stalled_chunks(
	requests: Res<StreamingSourceRequestHandle>,
	mut grids: Query<(GridId, &mut GridStreaming)>,
) {
	for (grid, mut streaming) in grids.iter_mut() {
		if streaming.stalled_edits.is_empty() { continue; }
		let chunks: Vec<IVec3> = streaming.stalled_edits.keys().copied().collect();
		for chunk in chunks {
			if matches!(streaming.presence.state(chunk), Some(ChunkState::Available))
				&& streaming.stalled_pinned.insert(chunk)
			{
				streaming.fetch(grid, &requests.0, chunk);
			}
		}
	}
}
