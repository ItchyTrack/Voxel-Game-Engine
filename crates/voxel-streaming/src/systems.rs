use std::{collections::{HashMap, HashSet}, sync::{Arc, Mutex}};

use bevy::ecs::message::{MessageReader, MessageWriter};
use bevy::math::IVec3;
use bevy::prelude::*;
use bevy::tasks::AsyncComputeTaskPool;
use voxel_edit::apply_grid_edit;
use voxel_sources::{SourceManager, SourceResult, SourceResultData};
use voxel_tasks::CancellationToken;

use tracy_client::span;

use voxel_data::grid::{reconcile_subgrids, Grid, GridId};
use voxel_data::grid_tree::NonZeroVoxelRegion;
use voxel_data::splat::{splat_voxels_blocking, GridSplat};
use voxel_data::subgrid::SubGrid;

use tile_data::{NonZeroChunkRegion, chunk_origin, chunks_covering_voxel_region};
use tile_data::CHUNK_SIZE;
use crate::consumer::ChunkConsumer;
use crate::generation::{
	TileGenerationChannel, TileGenerationMetadata, TileGenerationResult,
	session as generation_session,
};
use crate::streaming::TileStatus;
use crate::{
	DynamicTileData, LoadedTile, TileGenerationParameters,
	TileGeneratorRegistry, TileLoadStatus, TileLoadUpdate,
};
use crate::{GridStreaming, InflightChunkPresence, RequestChunkPresence};
use crate::{ChunkAvailabilityChangeKind, ChunkAvailabilityChanged, ChunkEditInterestChanged, ChunkLoadResolved, GridAreaEdited};
use crate::presence::ChunkState;
use crate::grid_source::StreamingGridSource;

pub fn apply_source_presence(
	mut source_results: MessageReader<SourceResult>,
	mut grids: Query<&mut GridStreaming>,
	mut availability_events: MessageWriter<ChunkAvailabilityChanged>,
) {
	for result in source_results.read() {
		let SourceResultData::Presence { grid, region } = &result.data else { continue };
		let Ok(mut streaming) = grids.get_mut(*grid) else {
			warn!(grid=?grid, region=?region, "source presence missing GridStreaming");
			continue;
		};
		let mut any_new = false;
		for x in region.min().x..region.end().x {
			for y in region.min().y..region.end().y {
				for z in region.min().z..region.end().z {
					let chunk = IVec3::new(x, y, z);
					if !streaming.presence().is_present(chunk) {
						streaming.mark_present(chunk);
						any_new = true;
					}
				}
			}
		}
		if any_new {
			availability_events.write(ChunkAvailabilityChanged {
				grid: *grid,
				region: *region,
				kind: ChunkAvailabilityChangeKind::BecamePresent,
			});
		}
	}
}

pub fn materialize_authoritative_commands(
	mut ecs_commands: Commands,
	mut commands: MessageReader<crate::AuthoritativeGridCommand>,
	mut grids: Query<(&mut GridStreaming, &mut Grid)>,
	mut sub_grids: Query<&mut SubGrid>,
) {
	for command in commands.read() {
		let Ok((mut streaming, mut grid_data)) = grids.get_mut(command.grid) else { continue };
		if !streaming.command_follows(command.region, command.generation) { continue; }
		let mut touched = HashSet::new();
		for z in command.region.min().z..command.region.end().z {
			for y in command.region.min().y..command.region.end().y {
				for x in command.region.min().x..command.region.end().x {
					let chunk = IVec3::new(x, y, z);
					let limit = NonZeroVoxelRegion::from_min_size(chunk_origin(chunk), IVec3::splat(CHUNK_SIZE)).unwrap();
					let Some(edit) = command.edit.clipped_to(limit) else { continue };
					if streaming.is_chunk_data_resident(chunk) {
						touched.extend(apply_grid_edit(grid_data.as_mut(), &edit));
					} else {
						streaming.pending_authoritative_edits.entry(chunk).or_default().push((command.generation, edit));
					}
				}
			}
		}
		if !touched.is_empty() {
			reconcile_subgrids(command.grid, grid_data.as_mut(), touched, &mut ecs_commands, &mut sub_grids);
		}
		streaming.note_source_generation(command.region, command.generation);
		streaming.dirty_stale_tiles(command.region, command.generation);
	}
}

pub(crate) fn request_edit_takes(
	mut commands: Commands,
	mut sources: ResMut<SourceManager>,
	source: Res<StreamingGridSource>,
	mut grids: Query<(
		GridId,
		Has<RequestChunkPresence>,
		Has<InflightChunkPresence>,
		&mut GridStreaming,
		&mut Grid,
	)>,
	mut sub_grids: Query<&mut SubGrid>,
	mut edited: MessageWriter<GridAreaEdited>,
) {
	let Some(source_id) = source.handle().map(|handle| handle.id()) else { return };
	for (grid, presence_requested, presence_inflight, mut streaming, mut grid_data) in &mut grids {
		let presence_pending = presence_requested || presence_inflight;
		let mut touched = HashSet::new();
		let mut deferred = Vec::new();
		let mut pending = std::mem::take(&mut streaming.pending_take_edits).into_iter();
		while let Some(edit) = pending.next() {
			let Ok(region) = NonZeroChunkRegion::try_from(chunks_covering_voxel_region(edit.voxel_bounds())) else { continue };
			let mut waiting = false;
			for z in region.min().z..region.end().z { for y in region.min().y..region.end().y { for x in region.min().x..region.end().x {
				let chunk = IVec3::new(x, y, z);
				if streaming.is_chunk_data_resident(chunk) { continue; }
				if streaming.state(chunk).is_none() {
					if presence_pending {
						waiting = true;
						continue;
					}
					streaming.mark_present(chunk);
					streaming.mark_loaded(chunk);
					streaming.pending_newly_present_edits.insert(chunk);
					continue;
				}
				waiting = true;
				if streaming.stalled_pinned.insert(chunk) {
					streaming.fetch(&mut sources, grid, chunk);
				}
			}}}
			if waiting {
				deferred.push(edit);
				deferred.extend(pending);
				break;
			}

			source.claim(grid, region);
			sources.transfer_onwership(source_id, grid, region);
			touched.extend(apply_grid_edit(grid_data.as_mut(), &edit));
			for z in region.min().z..region.end().z { for y in region.min().y..region.end().y { for x in region.min().x..region.end().x {
				let chunk = IVec3::new(x, y, z);
				streaming.presence.set_state(chunk, ChunkState::InternalDirty);
				streaming.newly_dirty.push(chunk);
				if streaming.pending_newly_present_edits.remove(&chunk) {
					streaming.newly_present_dirty.push(chunk);
				}
				if streaming.stalled_pinned.remove(&chunk) {
					streaming.release_completed(chunk);
				}
			}}}
			let generation = streaming.next_local_generation(region);
			streaming.dirty_stale_tiles(region, generation);
			edited.write(GridAreaEdited { grid, region, generation, edit });
		}
		streaming.pending_take_edits.extend(deferred);
		if !touched.is_empty() {
			reconcile_subgrids(grid, grid_data.as_mut(), touched, &mut commands, &mut sub_grids);
		}
	}
}

pub(crate) fn publish_edit_interest_changes(
	mut events: MessageWriter<ChunkEditInterestChanged>,
	mut grids: Query<(GridId, &mut GridStreaming)>,
) {
	for (grid, mut streaming) in &mut grids {
		for (chunk, (version, interested)) in std::mem::take(&mut streaming.queued_edit_interest) {
			events.write(ChunkEditInterestChanged { grid, region: NonZeroChunkRegion::from_single(chunk), version, interested });
		}
	}
}

pub fn request_presence_for_new_grids(
	mut sources: ResMut<SourceManager>,
	mut commands: Commands,
	grids: Query<(Entity, GridId), (With<RequestChunkPresence>, Without<InflightChunkPresence>)>,
) {
	for (entity, grid) in &grids {
		let request_id = sources.request_presence(grid);
		commands.entity(entity).insert(InflightChunkPresence(request_id));
	}
}

pub fn receive_chunk_presence_loaded(
	mut commands: Commands,
	mut results: MessageReader<SourceResult>,
	grids: Query<(Entity, &InflightChunkPresence)>,
) {
	for result in results.read() {
		if !matches!(result.data, SourceResultData::PresenceLoaded) { continue; }
		for (entity, request) in &grids {
			if request.0 != result.request_id { continue; }
			let mut entity = commands.entity(entity);
			entity.remove::<InflightChunkPresence>();
			entity.remove::<RequestChunkPresence>();
			break;
		}
	}
}

pub fn receive_results(
	mut commands: Commands,
	mut source_results: MessageReader<SourceResult>,
	mut grids: Query<(GridId, &mut GridStreaming, &mut Grid)>,
	mut sub_grids: Query<&mut SubGrid>,
	mut consumers: Query<&mut dyn ChunkConsumer>,
	mut availability_events: MessageWriter<ChunkAvailabilityChanged>,
	mut chunk_resolved: MessageWriter<ChunkLoadResolved>,
) {
	let mut completed = Vec::new();
	for result in source_results.read() {
		match &result.data {
			SourceResultData::Voxels { grid, region, lod, generation, voxels } if *lod == 0 && region.size() == UVec3::ONE => {
				let Ok((_, mut streaming, _)) = grids.get_mut(*grid) else { continue };
				let chunk = region.min();
				if streaming.inflight_chunk_cancellations.get(&chunk) != Some(&result.request_id) { continue; }
				let replace = streaming.pending_chunk_results.get(&result.request_id)
					.is_none_or(|(known_generation, _)| *known_generation <= *generation);
				if replace {
					streaming.pending_chunk_results.insert(result.request_id, (*generation, voxels.clone()));
				}
			}
			SourceResultData::VoxelsLoaded => {
				for (grid, mut streaming, _) in &mut grids {
					let Some(chunk) = streaming.inflight_chunk_cancellations.iter()
						.find_map(|(chunk, request_id)| (*request_id == result.request_id).then_some(*chunk))
					else { continue };
					streaming.inflight_chunk_cancellations.remove(&chunk);
					let data = streaming.pending_chunk_results.remove(&result.request_id);
					let (generation, voxels) = data.map_or((0, None), |(generation, voxels)| (generation, Some(voxels)));
					completed.push((grid, chunk, generation, voxels));
					break;
				}
			}
			_ => {}
		}
	}

	let mut loaded_by_grid: HashMap<GridId, Vec<(IVec3, u64, voxel_data::voxels::Voxels)>> = HashMap::new();
	for (result_grid, result_chunk, generation, voxels) in completed {
		let _zone = span!("receive result");
		let was_empty = voxels.is_none();
		let mut materialized_empty = false;
		let mut became_empty = false;
		let mut accepted = false;
		if let Ok((_, mut streaming, grid)) = grids.get_mut(result_grid) {
			if matches!(streaming.presence.state(result_chunk), Some(ChunkState::InFlight)) {
				streaming.note_source_generation(NonZeroChunkRegion::from_single(result_chunk), generation);
				accepted = true;
				match voxels {
					Some(_) if streaming.presence.request_count(result_chunk) == 0 => {
						streaming.presence.set_state(result_chunk, ChunkState::Available);
					}
					Some(voxels) => {
						streaming.mark_loaded(result_chunk);
						loaded_by_grid.entry(result_grid).or_default().push((result_chunk, generation, voxels));
					}
					None if streaming.stalled_pinned.contains(&result_chunk)
						|| streaming.pending_authoritative_edits.get(&result_chunk)
							.is_some_and(|edits| edits.iter().any(|(command_generation, _)| *command_generation > generation)) => {
						streaming.mark_loaded(result_chunk);
						loaded_by_grid.entry(result_grid).or_default().push((
							result_chunk,
							generation,
							voxel_data::voxels::Voxels::new_with_type(grid.voxel_type_info()),
						));
						materialized_empty = true;
					}
					None => {
						streaming.pending_authoritative_edits.remove(&result_chunk);
						streaming.mark_empty(NonZeroChunkRegion::from_single(result_chunk));
						became_empty = true;
					}
				}
			}
		}
		if became_empty {
			availability_events.write(ChunkAvailabilityChanged {
				grid: result_grid,
				region: NonZeroChunkRegion::from_single(result_chunk),
				kind: ChunkAvailabilityChangeKind::BecameEmpty,
			});
		}
		if accepted && was_empty && !materialized_empty {
			chunk_resolved.write(ChunkLoadResolved { grid: result_grid, chunk: result_chunk, visible: false });
		}
		if !accepted { continue; }

		let _zone = span!("update consumers");
		for mut entity_consumers in consumers.iter_mut() {
			for mut consumer in &mut entity_consumers {
				if !consumer.needed().get(&result_grid).is_some_and(|set| set.contains(&result_chunk)) { continue; }
				*consumer.outstanding_mut() = consumer.outstanding().saturating_sub(1);
				if was_empty && !materialized_empty
					&& let Some(set) = consumer.needed_mut().get_mut(&result_grid)
				{
					set.remove(&result_chunk);
					if set.is_empty() { consumer.needed_mut().remove(&result_grid); }
				}
			}
		}
	}

	for (grid_entity, loaded) in loaded_by_grid {
		let _zone = span!("apply grouped chunk splats");
		let Ok((_, mut streaming, mut grid)) = grids.get_mut(grid_entity) else { continue };
		let chunk_region = NonZeroVoxelRegion::from_min_size(IVec3::ZERO, IVec3::splat(CHUNK_SIZE)).unwrap();
		let splats: Vec<_> = loaded.iter()
			.map(|(chunk, _, voxels)| GridSplat { grid: 0, base: chunk_origin(*chunk), voxels, replace: Some(chunk_region) })
			.collect();
		let mut touched_by_grid = splat_voxels_blocking(std::slice::from_mut(grid.as_mut()), &splats);
		let mut touched = touched_by_grid.remove(&0).unwrap_or_default();
		for (chunk, generation, _) in &loaded {
			if let Some(commands) = streaming.pending_authoritative_edits.remove(chunk) {
				for (command_generation, edit) in commands {
					if *generation < command_generation { touched.extend(apply_grid_edit(grid.as_mut(), &edit)); }
				}
			}
		}
		reconcile_subgrids(grid_entity, grid.as_mut(), touched.iter().copied(), &mut commands, &mut sub_grids);
		for (chunk, _, _) in loaded {
			let min = chunk_origin(chunk);
			let visible = touched.iter()
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
		let context_matches = world.get::<TileGenerationParameters>(result.grid) == Some(&result.context);
		let accepted = {
			let Some(mut streaming) = world.get_mut::<GridStreaming>(result.grid) else { continue };
			let stale_source = result.dependencies.iter().any(|dependency| {
				streaming.region_generation(dependency.area) > dependency.generation
			});
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
					Transform::from_translation((key.min() * CHUNK_SIZE).as_vec3()),
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
	mut grids: Query<&mut GridStreaming, Changed<TileGenerationParameters>>,
) {
	for mut streaming in &mut grids { streaming.invalidate_generation_context(); }
}

pub(crate) fn request_tiles(
	bridge: Res<crate::generation::TileVoxelSourceBridge>,
	generators: Res<TileGeneratorRegistry>,
	results: Res<TileGenerationChannel>,
	mut grids: Query<(GridId, Option<&TileGenerationParameters>, &mut GridStreaming)>,
) {
	let requests = crate::generation::bridge_sender(&bridge);
	for (grid, context, mut streaming) in grids.iter_mut() {
		let pending: Vec<_> = std::mem::take(&mut streaming.pending_tile_requests).into_iter().collect();
		if !pending.is_empty() {
			assert!(context.is_some(), "grid {grid:?} has tile requests but no generation context");
		}
		let Some(context) = context else { continue };
		for key in pending {
			let Some(state) = streaming.tiles.get(&key) else { continue };
			if state.requesters.is_empty() { continue; }
			if !matches!(&state.status, TileStatus::Requested | TileStatus::Dirty) { continue; }
			streaming.next_tile_tag = streaming.next_tile_tag.wrapping_add(1).max(1);
			let tag = streaming.next_tile_tag;
			let context = context.clone();
			let generator = generators.generator(key.class);
			let cancellation = CancellationToken::new();
			let metadata = Arc::new(Mutex::new(TileGenerationMetadata::default()));
			let (session, generation_cancellation) = generation_session(
				grid,
				key,
				context.clone(),
				requests.clone(),
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
				drop(metadata);
				let _ = result_tx.send(TileGenerationResult {
					grid,
					tag,
					context,
					dependencies,
					data,
				});
			}).detach();
			streaming.inflight_tiles_by_tag.insert(tag, key);
			streaming.tiles.get_mut(&key).unwrap().status = TileStatus::InFlight {
				tag,
				cancellation: generation_cancellation,
			};
		}
	}
}

pub(crate) fn handle_dirty_chunks(
	mut grids: Query<(GridId, &mut GridStreaming)>,
	mut availability_events: MessageWriter<ChunkAvailabilityChanged>,
) {
	for (grid, mut streaming) in grids.iter_mut() {
		let newly_present_dirty: HashSet<_> = std::mem::take(&mut streaming.newly_present_dirty).into_iter().collect();
		for chunk in newly_present_dirty {
			availability_events.write(ChunkAvailabilityChanged {
				grid,
				region: NonZeroChunkRegion::from_single(chunk),
				kind: ChunkAvailabilityChangeKind::BecamePresent,
			});
		}
		streaming.newly_dirty.clear();
	}
}

pub(crate) fn apply_chunk_clears(
	mut commands: Commands,
	mut grids: Query<(Entity, &mut GridStreaming, &mut Grid)>,
	mut sub_grids: Query<&mut SubGrid>,
) {
	for (grid_entity, mut streaming, mut grid) in grids.iter_mut() {
		if streaming.pending_clears.is_empty() { continue; }
		let mut still_pending = Vec::new();
		for (chunk, frames) in std::mem::take(&mut streaming.pending_clears) {
			if streaming.presence.request_count(chunk) > 0 { continue; }
			match streaming.presence.state(chunk) {
				Some(ChunkState::Loaded) => {
					if frames > 0 {
						still_pending.push((chunk, frames - 1));
						continue;
					}
					let touched = grid.set_area(chunk_origin(chunk), IVec3::splat(CHUNK_SIZE), None);
					reconcile_subgrids(grid_entity, grid.as_mut(), touched, &mut commands, &mut sub_grids);
					streaming.presence.set_state(chunk, ChunkState::Available);
				}
				_ => {}
			}
		}
		streaming.pending_clears = still_pending;
	}
}
