use bevy::{ecs::message::{MessageReader, MessageWriter}};
use bevy::math::IVec3;
use bevy::prelude::*;
use voxel_sources::{SourceManager, SourceResult, SourceResultData};

use voxel_data::grid::GridId;
use tile_data::NonZeroChunkRegion;
use tile_data::CHUNK_SIZE;
use crate::{tile_building::TileBuildingChannel, tile_requester::TileRequester};
use crate::streaming::TileStatus;
use tile_data::{DynamicTileData, LoadedTile, TileBuildingParameters};
use crate::{GridStreaming, InflightChunkPresence, RequestChunkPresence, TileLoadStatus, TileLoadUpdate};
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
						any_new = true;
					}
				}
			}
		}
		if any_new {
			streaming.mark_present_area(*region);
			availability_events.write(ChunkAvailabilityChanged {
				grid: *grid,
				region: *region,
				kind: ChunkAvailabilityChangeKind::BecamePresent,
			});
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

#[derive(Resource, Default)]
pub struct PendingTileUpdates(Vec<TileLoadUpdate>);

fn cleanup_tile_entity(world: &mut World, entity: Entity) {
	if let Ok(entity_mut) = world.get_entity_mut(entity) { entity_mut.despawn(); }
}

pub fn cleanup_released_tiles(world: &mut World) {
	let mut released_entities = Vec::new();
	let mut query = world.query::<&mut GridStreaming>();
	for mut streaming in query.iter_mut(world) {
		let released: Vec<_> = streaming.tiles.iter().filter_map(|(&key, state)| state.requesters.is_empty().then_some(key)).collect();
		for key in released {
			streaming.tile_dependencies.remove(key);
			if let Some(state) = streaming.tiles.remove(&key) {
				if let Some(entity) = state.entity { released_entities.push(entity); }
			}
		}
	}
	for entity in released_entities { cleanup_tile_entity(world, entity); }
}

pub(crate) fn invalidate_changed_generation_contexts(
	mut tile_builder: TileRequester,
	grids: Query<(GridId, Option<&TileBuildingParameters>), Changed<TileBuildingParameters>>,
) {
	for (grid, context) in &grids {
		tile_builder.invalidate_building_context(grid, context);
	}
}

pub fn receive_tile_results(world: &mut World) {
	let results: Vec<_> = world.resource::<TileBuildingChannel>().drain().collect();
	for result in results {
		let key = result.tile_key;
		let accepted = world.get::<GridStreaming>(result.grid)
			.and_then(|streaming| streaming.tiles.get(&key))
			.is_some_and(|tile_state| matches!(
				tile_state.status,
				TileStatus::InFlight { generation, .. } if generation == result.generation
			));
		if !accepted { continue; }

		let (requesters, old) = {
			let streaming = world.get::<GridStreaming>(result.grid).unwrap();
			let tile_state = streaming.tiles.get(&key).unwrap();
			(tile_state.requesters.keys().copied().collect::<Vec<_>>(), tile_state.entity)
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
				let tile_state = streaming.tiles.get_mut(&key).unwrap();
				tile_state.entity = Some(entity);
				tile_state.status = TileStatus::Loaded;
				for requester in requesters {
					world.resource_mut::<PendingTileUpdates>().0.push(TileLoadUpdate { grid: result.grid, requester, key, status: TileLoadStatus::Ready(entity) });
				}
				if let Some(old) = old { cleanup_tile_entity(world, old); }
			}
			None => {
				let mut streaming = world.get_mut::<GridStreaming>(result.grid).unwrap();
				let tile_state = streaming.tiles.get_mut(&key).unwrap();
				tile_state.entity = None;
				tile_state.status = TileStatus::Loaded;
				for requester in requesters {
					world.resource_mut::<PendingTileUpdates>().0.push(TileLoadUpdate { grid: result.grid, requester, key, status: TileLoadStatus::Empty });
				}
				if let Some(old) = old { cleanup_tile_entity(world, old); }
			}
		}
	}
}