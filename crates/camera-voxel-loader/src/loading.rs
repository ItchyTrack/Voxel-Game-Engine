use std::collections::HashSet;

use bevy::ecs::message::MessageReader;
use bevy::prelude::*;
use gpu_voxel_data::{LodVoxels, SubGridGpuState, VoxelGpuUploadFinished};
use voxel_data::grid::{Grid, GridId};
use voxel_data::subgrid::SubGrid;
use voxel_renderer::voxel_camera::VoxelCamera;
use voxel_streaming::{ChunkBecameDirty, ChunkBecamePresent, ChunkConsumer, ChunkLoadResolved, ChunkRequestChannel, GridStreaming, LodRequestChannel, CHUNK_SIZE};

use crate::camera_voxel_loader::CameraVoxelLoader;
use crate::lod_policy::{add_lod_tiles, add_near_chunks, is_lod_tile_wanted, is_near_chunk_wanted, nearest_chunk_center, tile_key_covering_chunk, update_lod_tiles_delta, update_near_chunks_delta};
use crate::coverage::{chunks_in_bounds, remove_source, renew_source_request, request_source, resolve_empty, resolve_visible, retiring_visible_chunks, undesire_source, CoverageSource, SourceResolution, SourceState};
use crate::types::{ChunkKey, TileKey, TileRecord, TileStatus};
use crate::CameraVoxelLoaderConsumer;

pub(crate) fn update_camera_voxel_loader_requests(
	mut commands: Commands,
	chunk_channel: Res<ChunkRequestChannel>, lod_channel: Res<LodRequestChannel>,
	mut present_events: MessageReader<ChunkBecamePresent>, mut dirty_events: MessageReader<ChunkBecameDirty>,
	mut cameras: Query<(Entity, &Camera, &GlobalTransform, &mut CameraVoxelLoader, &mut CameraVoxelLoaderConsumer), With<Camera3d>>,
	mut grids: Query<(GridId, &GlobalTransform, &mut GridStreaming)>, grid_transforms: Query<&GlobalTransform, With<GridStreaming>>,
	grid_data: Query<&Grid>, subgrids: Query<&SubGrid>, subgrid_gpu: Query<&SubGridGpuState, With<SubGrid>>,
) {
	let present_events: Vec<_> = present_events.read().copied().collect();
	let dirty_events: Vec<_> = dirty_events.read().copied().collect();
	for (camera_entity, camera, camera_global, mut camera_voxel_loader, mut consumer) in &mut cameras {
		if !camera.is_active {
			continue;
		}
		let camera_world = camera_global.translation();
		let settings_changed = camera_voxel_loader.applied_settings.as_ref() != Some(&camera_voxel_loader.settings);
		let settings = camera_voxel_loader.settings.clone();
		let mut desired_chunks = if settings_changed { HashSet::new() } else { camera_voxel_loader.desired_chunks.clone() };
		let mut desired_tiles = if settings_changed { HashSet::new() } else { camera_voxel_loader.desired_tiles.clone() };
		let mut desired_changed = settings_changed;

		for (grid, grid_global, streaming) in &mut grids {
			let local = grid_global.affine().inverse().transform_point3(camera_world);
			let camera_chunk = nearest_chunk_center(local);
			let previous_center = if settings_changed { None } else { camera_voxel_loader.grid_centers.get(&grid).copied() };
			let rebuild_grid = previous_center.map_or(true, |old| should_rebuild_policy(old, camera_chunk));

			if rebuild_grid {
				desired_changed = true;
				desired_chunks.retain(|key| key.grid != grid);
				desired_tiles.retain(|key| key.grid != grid);
				add_near_chunks(&mut desired_chunks, grid, camera_chunk, &camera_voxel_loader, streaming.as_ref());
				add_lod_tiles(&mut desired_tiles, grid, camera_chunk, &camera_voxel_loader, streaming.as_ref());
			} else if let Some(old_center) = previous_center {
				if old_center != camera_chunk {
					desired_changed = true;
					update_near_chunks_delta(&mut desired_chunks, grid, old_center, camera_chunk, &settings, streaming.as_ref());
					update_lod_tiles_delta(&mut desired_tiles, grid, old_center, camera_chunk, &settings, streaming.as_ref());
				}
			}
			let before_retain = desired_chunks.len();
			desired_chunks.retain(|key| key.grid != grid || streaming.presence().is_present(key.chunk));
			desired_changed |= desired_chunks.len() != before_retain;

			for event in present_events.iter().filter(|event| event.grid == grid) {
				if is_near_chunk_wanted(&settings, streaming.as_ref(), camera_chunk, event.chunk) {
					desired_changed |= desired_chunks.insert(ChunkKey { grid, chunk: event.chunk });
				}
				for lod in 1..=settings.max_lod {
					let key = tile_key_covering_chunk(grid, event.chunk, lod);
					if is_lod_tile_wanted(&settings, streaming.as_ref(), camera_chunk, key) {
						desired_changed |= desired_tiles.insert(key);
					}
				}
			}

			camera_voxel_loader.grid_centers.insert(grid, camera_chunk);
		}

		camera_voxel_loader.applied_settings = Some(settings);
		let chunks_to_fetch = if desired_changed {
			let chunks_to_fetch = update_desired_chunks(&mut camera_voxel_loader, desired_chunks.clone());
			camera_voxel_loader.desired_tiles = desired_tiles.clone();
			let ready = sync_desired_coverage(&mut camera_voxel_loader, &desired_chunks, &desired_tiles);
			retire_sources_from_request_query(&mut commands, &mut camera_voxel_loader, consumer.as_mut(), &mut grids, ready);
			chunks_to_fetch
		} else {
			Vec::new()
		};
		apply_chunk_change_events(&mut commands, &mut camera_voxel_loader, &desired_tiles, &present_events, &dirty_events);

		for chunk_key in chunks_to_fetch {
			if let Ok((_, _, mut streaming)) = grids.get_mut(chunk_key.grid) {
				streaming.fetch_needed(chunk_key.grid, consumer.as_mut(), &chunk_channel, chunk_key.chunk);
				if matches!(streaming.state(chunk_key.chunk), Some(voxel_streaming::ChunkState::Loaded | voxel_streaming::ChunkState::InternalDirty)) {
					let ready = resolve_chunk_source_if_ready(&mut camera_voxel_loader, &grid_data, &subgrids, &subgrid_gpu, chunk_key);
					retire_sources_from_request_query(&mut commands, &mut camera_voxel_loader, consumer.as_mut(), &mut grids, ready);
				}
			}
		}

		if desired_changed {
			let old_tiles: Vec<TileKey> = camera_voxel_loader.tiles.keys().copied().collect();
			for key in old_tiles {
				if desired_tiles.contains(&key) {
					let visible_entity = if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
						if record.status == TileStatus::Retiring {
							record.status = TileStatus::Ready;
						}
						(record.status == TileStatus::Ready).then_some(record.entity).flatten()
					} else {
						None
					};
					if let Some(entity) = visible_entity {
						request_source(&mut camera_voxel_loader, CoverageSource::Tile(key));
						let ready = resolve_visible(&mut camera_voxel_loader, CoverageSource::Tile(key), entity);
						retire_sources_from_request_query(&mut commands, &mut camera_voxel_loader, consumer.as_mut(), &mut grids, ready);
					}
					continue;
				}

				let ready = handle_non_desired_tile(&mut camera_voxel_loader, key);
				retire_sources_from_request_query(&mut commands, &mut camera_voxel_loader, consumer.as_mut(), &mut grids, ready);
			}

			for key in desired_tiles {
				queue_tile_if_missing(&mut camera_voxel_loader, key);
			}
		}

		let mut sent = 0usize;
		while sent < camera_voxel_loader.settings.requests_per_frame && in_flight_count(&camera_voxel_loader) < camera_voxel_loader.settings.max_in_flight {
			let Some(key) = camera_voxel_loader.queue.pop_front() else {
				break;
			};
			if !matches!(camera_voxel_loader.tiles.get(&key).map(|r| r.status), Some(TileStatus::Queued)) {
				continue;
			}
			let priority = tile_priority(camera_world, key, &grid_transforms);
			let generation = camera_voxel_loader.tiles.get(&key).map_or(0, |record| record.generation);
			let requested = grids
				.get(key.grid)
				.map(|(_, _, streaming)| streaming.fetch_lod(key.grid, camera_entity, &lod_channel, key.min, key.size(), key.lod as f32, priority, generation))
				.unwrap_or(false);
			if requested {
				if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
					record.status = TileStatus::Loading;
				}
				sent += 1;
			} else {
				if let Some(mut record) = camera_voxel_loader.tiles.remove(&key) {
					despawn_stale_entity(&mut commands, &mut record);
				}
				let ready = resolve_empty(&mut camera_voxel_loader, CoverageSource::Tile(key));
				retire_sources_from_request_query(&mut commands, &mut camera_voxel_loader, consumer.as_mut(), &mut grids, ready);
			}
		}
	}
}

fn in_flight_count(camera_voxel_loader: &CameraVoxelLoader) -> usize {
	camera_voxel_loader.tiles.values().filter(|r| r.status == TileStatus::Loading).count()
}

fn retire_sources(
	commands: &mut Commands,
	camera_voxel_loader: &mut CameraVoxelLoader,
	consumer: &mut CameraVoxelLoaderConsumer,
	grids: &mut Query<&mut GridStreaming>,
	sources: impl IntoIterator<Item = CoverageSource>,
) {
	for source in sources {
		match source {
			CoverageSource::Tile(key) => {
				remove_source(camera_voxel_loader, source);
				if let Some(mut record) = camera_voxel_loader.tiles.remove(&key) {
					despawn_tile_entities(commands, &mut record);
				}
			}
			CoverageSource::Chunk(chunk) => {
				remove_source(camera_voxel_loader, source);
				if let Ok(mut streaming) = grids.get_mut(chunk.grid) {
					streaming.release_needed(chunk.grid, consumer, chunk.chunk);
				}
			}
		}
	}
}

fn retire_sources_from_request_query(
	commands: &mut Commands,
	camera_voxel_loader: &mut CameraVoxelLoader,
	consumer: &mut CameraVoxelLoaderConsumer,
	grids: &mut Query<(GridId, &GlobalTransform, &mut GridStreaming)>,
	sources: impl IntoIterator<Item = CoverageSource>,
) {
	for source in sources {
		match source {
			CoverageSource::Tile(key) => {
				remove_source(camera_voxel_loader, source);
				if let Some(mut record) = camera_voxel_loader.tiles.remove(&key) {
					despawn_tile_entities(commands, &mut record);
				}
			}
			CoverageSource::Chunk(chunk) => {
				remove_source(camera_voxel_loader, source);
				if let Ok((_, _, mut streaming)) = grids.get_mut(chunk.grid) {
					streaming.release_needed(chunk.grid, consumer, chunk.chunk);
				}
			}
		}
	}
}

fn apply_chunk_change_events(
	commands: &mut Commands,
	camera_voxel_loader: &mut CameraVoxelLoader,
	desired_tiles: &HashSet<TileKey>,
	present_events: &[ChunkBecamePresent],
	dirty_events: &[ChunkBecameDirty],
) {
	let max_lod = camera_voxel_loader.settings.max_lod;
	for event in present_events {
		for lod in 1..=max_lod {
			let key = tile_key_covering_chunk(event.grid, event.chunk, lod);
			if desired_tiles.contains(&key) {
				requeue_empty_or_missing_tile(camera_voxel_loader, key);
			}
		}
	}
	for event in dirty_events {
		for lod in 1..=max_lod {
			let key = tile_key_covering_chunk(event.grid, event.chunk, lod);
			invalidate_dirty_tile(commands, camera_voxel_loader, key, desired_tiles.contains(&key));
		}
	}
}

fn queue_tile_if_missing(camera_voxel_loader: &mut CameraVoxelLoader, key: TileKey) {
	if camera_voxel_loader.tiles.contains_key(&key) || tile_coverage_is_desired_empty(camera_voxel_loader, key) {
		return;
	}
	queue_tile_for_request(camera_voxel_loader, key);
}

fn queue_tile_for_request(camera_voxel_loader: &mut CameraVoxelLoader, key: TileKey) {
	camera_voxel_loader.tiles.insert(key, TileRecord::queued());
	request_source(camera_voxel_loader, CoverageSource::Tile(key));
	camera_voxel_loader.queue.push_back(key);
}

fn requeue_empty_or_missing_tile(camera_voxel_loader: &mut CameraVoxelLoader, key: TileKey) {
	if camera_voxel_loader.tiles.contains_key(&key) {
		return;
	}
	invalidate_empty_tile_coverage(camera_voxel_loader, key);
	queue_tile_for_request(camera_voxel_loader, key);
}

fn invalidate_empty_tile_coverage(camera_voxel_loader: &mut CameraVoxelLoader, key: TileKey) {
	if tile_coverage_is_desired_empty(camera_voxel_loader, key) {
		remove_source(camera_voxel_loader, CoverageSource::Tile(key));
	}
}

fn tile_coverage_is_desired_empty(camera_voxel_loader: &CameraVoxelLoader, key: TileKey) -> bool {
	matches!(
		camera_voxel_loader.coverage_sources.get(&CoverageSource::Tile(key)).map(|record| record.state),
		Some(SourceState::Desired(SourceResolution::Empty))
	)
}

fn invalidate_dirty_tile(commands: &mut Commands, camera_voxel_loader: &mut CameraVoxelLoader, key: TileKey, desired: bool) {
	let Some(record) = camera_voxel_loader.tiles.get_mut(&key) else {
		if desired {
			invalidate_empty_tile_coverage(camera_voxel_loader, key);
			queue_tile_if_missing(camera_voxel_loader, key);
		}
		return;
	};
	record.generation += 1;
	match record.status {
		TileStatus::Ready | TileStatus::Retiring => {
			if desired {
				record.stale_entity = record.stale_entity.or(record.entity.take());
				record.status = TileStatus::Queued;
				renew_source_request(camera_voxel_loader, CoverageSource::Tile(key));
				camera_voxel_loader.queue.push_back(key);
			} else {
				despawn_tile_entities(commands, record);
				camera_voxel_loader.tiles.remove(&key);
				remove_source(camera_voxel_loader, CoverageSource::Tile(key));
			}
		}
		TileStatus::Queued | TileStatus::Loading | TileStatus::LoadedWaitingGpu => {
			if desired {
				if record.status == TileStatus::LoadedWaitingGpu {
					if let Some(entity) = record.entity.take() {
						camera_voxel_loader.waiting_gpu_lods.remove(&entity);
						commands.entity(entity).despawn();
					}
				}
				record.status = TileStatus::Queued;
				renew_source_request(camera_voxel_loader, CoverageSource::Tile(key));
				camera_voxel_loader.queue.push_back(key);
			}
		}
	}
}

fn despawn_tile_entities(commands: &mut Commands, record: &mut TileRecord) {
	if let Some(entity) = record.entity.take() {
		commands.entity(entity).despawn();
	}
	despawn_stale_entity(commands, record);
}

fn despawn_stale_entity(commands: &mut Commands, record: &mut TileRecord) {
	if let Some(entity) = record.stale_entity.take() {
		commands.entity(entity).despawn();
	}
}

fn should_rebuild_policy(old_center: IVec3, new_center: IVec3) -> bool {
	const MAX_INCREMENTAL_DELTA_CHUNKS: i32 = 8;
	(new_center - old_center).abs().max_element() > MAX_INCREMENTAL_DELTA_CHUNKS
}

fn sync_desired_coverage(camera_voxel_loader: &mut CameraVoxelLoader, desired_chunks: &HashSet<ChunkKey>, desired_tiles: &HashSet<TileKey>) -> Vec<CoverageSource> {
	let desired_sources: HashSet<_> = desired_chunks
		.iter()
		.copied()
		.map(CoverageSource::Chunk)
		.chain(desired_tiles.iter().copied().map(CoverageSource::Tile))
		.collect();

	for source in &desired_sources {
		request_source(camera_voxel_loader, *source);
	}

	let old_desired: Vec<_> = camera_voxel_loader
		.coverage_sources
		.iter()
		.filter_map(|(&source, record)| matches!(record.state, SourceState::Desired(_)).then_some(source))
		.collect();
	let mut ready = Vec::new();
	for source in old_desired {
		if !desired_sources.contains(&source) && !keep_undesired_source_until_result(camera_voxel_loader, source) {
			ready.extend(undesire_source(camera_voxel_loader, source));
		}
	}
	ready
}

fn keep_undesired_source_until_result(camera_voxel_loader: &CameraVoxelLoader, source: CoverageSource) -> bool {
	let CoverageSource::Tile(key) = source else { return false };
	matches!(camera_voxel_loader.tiles.get(&key).map(|record| record.status), Some(TileStatus::Loading | TileStatus::LoadedWaitingGpu))
}

fn update_desired_chunks(camera_voxel_loader: &mut CameraVoxelLoader, desired_chunks: HashSet<ChunkKey>) -> Vec<ChunkKey> {
	let old_chunks = camera_voxel_loader.desired_chunks.clone();
	let chunks_to_fetch: Vec<ChunkKey> = desired_chunks.difference(&old_chunks).copied().collect();
	camera_voxel_loader.desired_chunks = desired_chunks;
	chunks_to_fetch
}

fn handle_non_desired_tile(camera_voxel_loader: &mut CameraVoxelLoader, key: TileKey) -> Vec<CoverageSource> {
	let Some(status) = camera_voxel_loader.tiles.get(&key).map(|record| record.status) else { return Vec::new() };
	match status {
		// Already-renderable tiles must stay visible until replacement coverage is ready.
		TileStatus::Ready | TileStatus::Retiring => {
			if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
				record.status = TileStatus::Retiring;
			}
			undesire_source(camera_voxel_loader, CoverageSource::Tile(key))
		}
		// In-flight work may still produce valid voxel data. Keep the record so the
		// late result can be applied, then retire it through the same no-gap path.
		TileStatus::Loading | TileStatus::LoadedWaitingGpu => Vec::new(),
		// Queued work has not been sent yet, so it can be safely dropped.
		TileStatus::Queued => {
			camera_voxel_loader.tiles.remove(&key);
			remove_source(camera_voxel_loader, CoverageSource::Tile(key));
			Vec::new()
		}
	}
}

fn tile_priority(camera_world: Vec3, key: TileKey, grid_transforms: &Query<&GlobalTransform, With<GridStreaming>>) -> f32 {
	let Ok(grid_global) = grid_transforms.get(key.grid) else { return 0.0 };
	let center_local = ((key.min + key.size() / 2) * CHUNK_SIZE).as_vec3();
	let center_world = grid_global.transform_point(center_local);
	-camera_world.distance(center_world)
}

fn resolve_chunk_source_if_ready(
	camera_voxel_loader: &mut CameraVoxelLoader,
	grids: &Query<&Grid>,
	subgrids: &Query<&SubGrid>,
	subgrid_gpu: &Query<&SubGridGpuState, With<SubGrid>>,
	chunk: ChunkKey,
) -> Vec<CoverageSource> {
	let Ok(grid) = grids.get(chunk.grid) else { return Vec::new() };
	let mut ready = Vec::new();
	let min = chunk.chunk * CHUNK_SIZE;
	let mut found_subgrid = false;
	for entity in grid.subgrid_entities_in_area(min, IVec3::splat(CHUNK_SIZE)) {
		if subgrids.get(entity).is_err() {
			continue;
		}
		found_subgrid = true;
		if subgrid_gpu.get(entity).is_ok() {
			ready.extend(resolve_visible(camera_voxel_loader, CoverageSource::Chunk(chunk), entity));
		}
	}
	if !found_subgrid {
		ready.extend(resolve_empty(camera_voxel_loader, CoverageSource::Chunk(chunk)));
	}
	ready
}

pub(crate) fn receive_camera_voxel_loader_results(
	mut commands: Commands,
	mut camera_voxel_loaders: Query<&mut CameraVoxelLoader>,
	mut consumers: Query<&mut CameraVoxelLoaderConsumer>,
	mut grids: Query<&mut GridStreaming>,
) {
	for mut consumer in &mut consumers {
		let results = consumer.drain_lod();
		if results.is_empty() {
			continue;
		}
		// The consumer lives on the same entity as the controller.
		// Query iteration order is not reliable, so use the requester embedded in each result.
		for result in results {
			let Ok(mut camera_voxel_loader) = camera_voxel_loaders.get_mut(result.requester) else { continue };
			let key = TileKey { grid: result.grid, lod: result.lod.max(0.0).floor() as u8, min: result.min };
			let Some(record) = camera_voxel_loader.tiles.get(&key) else { continue };
			if record.generation != result.generation || record.entity.is_some() {
				continue;
			}
			match result.voxels {
				Some(voxels) if !voxels.is_empty() => {
					let entity = commands
						.spawn((
							LodVoxels { voxels, lod: result.lod, priority: result.priority },
							Transform::from_translation((result.min * CHUNK_SIZE).as_vec3()),
							ChildOf(result.grid),
						))
						.id();
					if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
						record.entity = Some(entity);
						record.status = TileStatus::LoadedWaitingGpu;
					}
					camera_voxel_loader.waiting_gpu_lods.insert(entity, key);
				}
				_ => {
					if let Some(mut record) = camera_voxel_loader.tiles.remove(&key) {
						despawn_stale_entity(&mut commands, &mut record);
					}
					let ready = resolve_empty(&mut camera_voxel_loader, CoverageSource::Tile(key));
					retire_sources(&mut commands, &mut camera_voxel_loader, consumer.as_mut(), &mut grids, ready);
				}
			}
		}
	}
}

pub(crate) fn refresh_camera_voxel_loader_visibility(
	mut commands: Commands,
	mut gpu_events: MessageReader<VoxelGpuUploadFinished>,
	mut chunk_events: MessageReader<ChunkLoadResolved>,
	mut cameras: Query<(&mut CameraVoxelLoader, &mut CameraVoxelLoaderConsumer, &mut VoxelCamera)>, grids: Query<&Grid>,
	mut streaming_grids: Query<&mut GridStreaming>, subgrids: Query<&SubGrid>, subgrid_gpu: Query<&SubGridGpuState, With<SubGrid>>,
) {
	let gpu_events: Vec<_> = gpu_events.read().copied().collect();
	let chunk_events: Vec<_> = chunk_events.read().copied().collect();
	for (mut camera_voxel_loader, mut consumer, mut request_map) in &mut cameras {
		for event in &chunk_events {
			let source = CoverageSource::Chunk(ChunkKey { grid: event.grid, chunk: event.chunk });
			if !event.visible {
				let ready = resolve_empty(&mut camera_voxel_loader, source);
				retire_sources(&mut commands, &mut camera_voxel_loader, consumer.as_mut(), &mut streaming_grids, ready);
			}
		}
		for event in &gpu_events {
			if let (Ok(subgrid), Ok(gpu_state)) = (subgrids.get(event.entity), subgrid_gpu.get(event.entity)) {
				let placement = gpu_state.placement();
				for chunk in chunks_in_bounds(
					subgrid.grid(),
					subgrid.sub_grid_pos() + placement.bounds_min.as_ivec3(),
					subgrid.sub_grid_pos() + placement.bounds_max.as_ivec3(),
				) {
					if camera_voxel_loader.desired_chunks.contains(&chunk) {
						let ready = resolve_visible(&mut camera_voxel_loader, CoverageSource::Chunk(chunk), event.entity);
						retire_sources(&mut commands, &mut camera_voxel_loader, consumer.as_mut(), &mut streaming_grids, ready);
					}
				}
			}
			let Some(key) = camera_voxel_loader.waiting_gpu_lods.remove(&event.entity) else { continue };
			let mut resolved = false;
			if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
				if record.status == TileStatus::LoadedWaitingGpu && record.entity == Some(event.entity) {
					record.status = TileStatus::Ready;
					despawn_stale_entity(&mut commands, record);
					resolved = true;
				}
			}
			if !resolved {
				continue;
			}
			let ready = resolve_visible(&mut camera_voxel_loader, CoverageSource::Tile(key), event.entity);
			retire_sources(&mut commands, &mut camera_voxel_loader, consumer.as_mut(), &mut streaming_grids, ready);
			if !camera_voxel_loader.desired_tiles.contains(&key) {
				if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
					record.status = TileStatus::Retiring;
				}
				let ready = undesire_source(&mut camera_voxel_loader, CoverageSource::Tile(key));
				retire_sources(&mut commands, &mut camera_voxel_loader, consumer.as_mut(), &mut streaming_grids, ready);
			}
		}

		let mut subgrids_to_render = Vec::new();
		let mut lods_to_render = Vec::new();
		let mut seen_subgrids = HashSet::new();
		let mut seen_lods = HashSet::new();
		for chunk in camera_voxel_loader.desired_chunks.iter().copied().chain(retiring_visible_chunks(&camera_voxel_loader)) {
			let Ok(grid) = grids.get(chunk.grid) else { continue };
			let min = chunk.chunk * CHUNK_SIZE;
			for entity in grid.subgrid_entities_in_area(min, IVec3::splat(CHUNK_SIZE)) {
				if subgrids.get(entity).is_ok() && seen_subgrids.insert(entity) {
					subgrids_to_render.push(entity);
				}
			}
		}

		for record in camera_voxel_loader.tiles.values() {
			if let Some(entity) = record.stale_entity {
				if seen_lods.insert(entity) {
					lods_to_render.push(entity);
				}
			}
			if matches!(record.status, TileStatus::Ready | TileStatus::Retiring) {
				if let Some(entity) = record.entity {
					if seen_lods.insert(entity) {
						lods_to_render.push(entity);
					}
				}
			}
		}
		request_map.subgrids_to_render = subgrids_to_render;
		request_map.lods_to_render = lods_to_render;
	}
}

pub(crate) fn retire_replaced_tiles() {}

pub(crate) fn retire_replaced_chunks() {}
