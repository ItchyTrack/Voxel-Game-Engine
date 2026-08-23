use std::{collections::HashMap, sync::{Arc, Mutex}};

use bevy::{ecs::message::{MessageReader, MessageWriter}, log::tracing::Instrument};
use bevy::math::IVec3;
use bevy::prelude::*;
use bevy::tasks::AsyncComputeTaskPool;
use voxel_sources::{SourceManager, SourceResult, SourceResultData};
use voxel_tasks::CancellationToken;

use tracy_client::span;

use voxel_data::grid::{Grid, GridId};
use tile_data::{NonZeroChunkRegion};
use tile_data::CHUNK_SIZE;
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
use crate::{ChunkAvailabilityChangeKind, ChunkAvailabilityChanged, ChunkEditInterestChanged};

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
) {
	for command in commands.read() {
		let Ok((mut streaming, mut grid_data)) = grids.get_mut(command.grid) else { continue };
		if !streaming.command_follows(command.region, command.generation) { continue; }
		streaming.note_source_generation(command.region, command.generation);
		streaming.dirty_stale_tiles(command.region, command.generation);
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
	mut availability_events: MessageWriter<ChunkAvailabilityChanged>,
) {
	let mut completed = Vec::new();
	for result in source_results.read() {
		match &result.data {
			SourceResultData::Voxels { grid, region, lod, generation, voxels } if *lod == 0 && region.size() == UVec3::ONE => {
				let Ok((_, mut streaming, _)) = grids.get_mut(*grid) else { continue };
				let chunk = region.min();
			}
			SourceResultData::VoxelsLoaded => {
				for (grid, mut streaming, _) in &mut grids {
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
			// if matches!(streaming.presence.state(result_chunk), Some(ChunkState::InFlight)) {
			// 	streaming.note_source_generation(NonZeroChunkRegion::from_single(result_chunk), generation);
			// 	accepted = true;
			// 	match voxels {
			// 		Some(_) if streaming.presence.request_count(result_chunk) == 0 => {
			// 			streaming.presence.set_state(result_chunk, ChunkState::Available);
			// 		}
			// 		Some(voxels) => {
			// 			streaming.mark_loaded(result_chunk);
			// 			loaded_by_grid.entry(result_grid).or_default().push((result_chunk, generation, voxels));
			// 		}
			// 		None if streaming.stalled_pinned.contains(&result_chunk)
			// 			|| streaming.pending_authoritative_edits.get(&result_chunk)
			// 				.is_some_and(|edits| edits.iter().any(|(command_generation, _)| *command_generation > generation)) => {
			// 			streaming.mark_loaded(result_chunk);
			// 			loaded_by_grid.entry(result_grid).or_default().push((
			// 				result_chunk,
			// 				generation,
			// 				voxel_data::voxels::Voxels::new_with_type(grid.voxel_type_info()),
			// 			));
			// 			materialized_empty = true;
			// 		}
			// 		None => {
			// 			streaming.pending_authoritative_edits.remove(&result_chunk);
			// 			streaming.mark_empty(NonZeroChunkRegion::from_single(result_chunk));
			// 			became_empty = true;
			// 		}
			// 	}
			// }
		}
		if became_empty {
			availability_events.write(ChunkAvailabilityChanged {
				grid: result_grid,
				region: NonZeroChunkRegion::from_single(result_chunk),
				kind: ChunkAvailabilityChangeKind::BecameEmpty,
			});
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
				streaming.region_generation(dependency.region) > dependency.generation
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
					Transform::from_translation((key.min() * CHUNK_SIZE as i32).as_vec3()),
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
	let requests = bridge.sender();
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
			}.instrument(bevy::log::info_span!("build tile"))).detach();
			streaming.inflight_tiles_by_tag.insert(tag, key);
			streaming.tiles.get_mut(&key).unwrap().status = TileStatus::InFlight {
				tag,
				cancellation: generation_cancellation,
			};
		}
	}
}
