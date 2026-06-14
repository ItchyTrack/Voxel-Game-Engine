use std::collections::HashSet;

use bevy::ecs::message::MessageReader;
use bevy::prelude::*;
use gpu_voxel_data::{LodVoxels, SubGridGpuState, VoxelGpuUploadFinished};
use voxel_data::grid::{Grid, GridId};
use voxel_data::subgrid::SubGrid;
use voxel_renderer::voxel_camera::VoxelCamera;
use voxel_streaming::{ChunkBecameDirty, ChunkBecamePresent, ChunkConsumer, ChunkLoadResolved, ChunkRequestChannel, GridStreaming, LodRequestChannel, CHUNK_SIZE};

use crate::camera_voxel_loader::CameraVoxelLoader;
use crate::lod_policy::{add_lod_tiles, add_near_chunks, is_lod_tile_wanted, nearest_chunk_center, tile_key_covering_chunk, update_lod_tiles_delta, update_near_chunks_delta};
use crate::coverage::{chunks_for_subgrid_bounds, ready_retiring_sources, remove_source, request_source, resolve_empty, resolve_visible, undesire_source, CoverageSource};
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
		let mut policy_debug_boxes = Vec::new();
		let mut desired_chunks = if settings_changed { HashSet::new() } else { camera_voxel_loader.desired_chunks.clone() };
		let mut desired_tiles = if settings_changed { HashSet::new() } else { camera_voxel_loader.desired_tiles.clone() };

		for (grid, grid_global, streaming) in &mut grids {
			let local = grid_global.affine().inverse().transform_point3(camera_world);
			let camera_chunk = nearest_chunk_center(local);
			let previous_center = if settings_changed { None } else { camera_voxel_loader.grid_centers.get(&grid).copied() };
			let rebuild_grid = previous_center.map_or(true, |old| should_rebuild_policy(old, camera_chunk));

			if rebuild_grid {
				desired_chunks.retain(|key| key.grid != grid);
				desired_tiles.retain(|key| key.grid != grid);
				add_near_chunks(&mut desired_chunks, grid, camera_chunk, &camera_voxel_loader, streaming.as_ref());
				add_lod_tiles(&mut desired_tiles, grid, camera_chunk, &camera_voxel_loader, streaming.as_ref());
			} else if let Some(old_center) = previous_center {
				update_near_chunks_delta(&mut desired_chunks, &mut policy_debug_boxes, grid, old_center, camera_chunk, &settings, streaming.as_ref());
				update_lod_tiles_delta(&mut desired_tiles, &mut policy_debug_boxes, grid, old_center, camera_chunk, &settings, streaming.as_ref());
			}
			desired_chunks.retain(|key| key.grid != grid || streaming.presence().is_present(key.chunk));

			for event in present_events.iter().filter(|event| event.grid == grid) {
				if chunk_inside_near_box(camera_chunk, event.chunk, settings.near_radius_chunks) {
					desired_chunks.insert(ChunkKey { grid, chunk: event.chunk });
				}
				for lod in 1..=settings.max_lod {
					let key = tile_key_covering_chunk(grid, event.chunk, lod);
					if is_lod_tile_wanted(&settings, streaming.as_ref(), camera_chunk, key) {
						desired_tiles.insert(key);
					}
				}
			}

			camera_voxel_loader.grid_centers.insert(grid, camera_chunk);
		}

		camera_voxel_loader.applied_settings = Some(settings);
		camera_voxel_loader.policy_debug_boxes = policy_debug_boxes;
		camera_voxel_loader.desired_tiles = desired_tiles.clone();
		apply_chunk_change_events(&mut commands, &mut camera_voxel_loader, &desired_tiles, &present_events, &dirty_events);

		let chunks_to_fetch = update_desired_chunks(&mut camera_voxel_loader, desired_chunks);
		for chunk_key in chunks_to_fetch {
			if let Ok((_, _, mut streaming)) = grids.get_mut(chunk_key.grid) {
				streaming.fetch_needed(chunk_key.grid, consumer.as_mut(), &chunk_channel, chunk_key.chunk);
				if matches!(streaming.state(chunk_key.chunk), Some(voxel_streaming::ChunkState::Loaded | voxel_streaming::ChunkState::InternalDirty)) {
					resolve_chunk_source_if_ready(&mut camera_voxel_loader, &grid_data, &subgrids, &subgrid_gpu, chunk_key);
				}
			}
		}

		let old_tiles: Vec<TileKey> = camera_voxel_loader.tiles.keys().copied().collect();
		for key in old_tiles {
			if desired_tiles.contains(&key) {
				if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
					if record.status == TileStatus::Retiring {
						record.status = if record.entity.is_some() { TileStatus::Ready } else { TileStatus::Empty };
						request_source(&mut camera_voxel_loader, CoverageSource::Tile(key));
					}
				}
				continue;
			}

			handle_non_desired_tile(&mut camera_voxel_loader, key);
		}

		for key in desired_tiles {
			queue_tile_if_missing(&mut camera_voxel_loader, key);
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
				if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
					record.status = TileStatus::Empty;
					despawn_stale_entity(&mut commands, record);
				}
				resolve_empty(&mut camera_voxel_loader, CoverageSource::Tile(key));
			}
		}
	}
}

fn in_flight_count(camera_voxel_loader: &CameraVoxelLoader) -> usize {
	camera_voxel_loader.tiles.values().filter(|r| r.status == TileStatus::Loading).count()
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
	if camera_voxel_loader.tiles.contains_key(&key) {
		return;
	}
	camera_voxel_loader.tiles.insert(key, TileRecord::queued());
	request_source(camera_voxel_loader, CoverageSource::Tile(key));
	camera_voxel_loader.queue.push_back(key);
}

fn requeue_empty_or_missing_tile(camera_voxel_loader: &mut CameraVoxelLoader, key: TileKey) {
	match camera_voxel_loader.tiles.get_mut(&key) {
		Some(record) if record.status == TileStatus::Empty => {
			record.status = TileStatus::Queued;
			record.generation += 1;
			request_source(camera_voxel_loader, CoverageSource::Tile(key));
			camera_voxel_loader.queue.push_back(key);
		}
		Some(_) => {}
		None => queue_tile_if_missing(camera_voxel_loader, key),
	}
}

fn invalidate_dirty_tile(commands: &mut Commands, camera_voxel_loader: &mut CameraVoxelLoader, key: TileKey, desired: bool) {
	let Some(record) = camera_voxel_loader.tiles.get_mut(&key) else {
		if desired {
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
				request_source(camera_voxel_loader, CoverageSource::Tile(key));
				camera_voxel_loader.queue.push_back(key);
			} else {
				despawn_tile_entities(commands, record);
				camera_voxel_loader.tiles.remove(&key);
				remove_source(camera_voxel_loader, CoverageSource::Tile(key));
			}
		}
		TileStatus::Empty => {
			if desired {
				record.status = TileStatus::Queued;
				request_source(camera_voxel_loader, CoverageSource::Tile(key));
				camera_voxel_loader.queue.push_back(key);
			} else {
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
				request_source(camera_voxel_loader, CoverageSource::Tile(key));
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

fn chunk_inside_near_box(center: IVec3, chunk: IVec3, radius: i32) -> bool {
	(chunk - center).abs().max_element() <= radius
}

fn update_desired_chunks(camera_voxel_loader: &mut CameraVoxelLoader, desired_chunks: HashSet<ChunkKey>) -> Vec<ChunkKey> {
	let old_chunks = camera_voxel_loader.desired_chunks.clone();
	for &chunk_key in old_chunks.difference(&desired_chunks) {
		undesire_source(camera_voxel_loader, CoverageSource::Chunk(chunk_key));
	}

	let chunks_to_fetch: Vec<ChunkKey> = desired_chunks.difference(&old_chunks).copied().collect();
	for chunk_key in &chunks_to_fetch {
		request_source(camera_voxel_loader, CoverageSource::Chunk(*chunk_key));
	}
	camera_voxel_loader.desired_chunks = desired_chunks;
	chunks_to_fetch
}

fn handle_non_desired_tile(camera_voxel_loader: &mut CameraVoxelLoader, key: TileKey) {
	let Some(status) = camera_voxel_loader.tiles.get(&key).map(|record| record.status) else { return };
	match status {
		// Already-renderable tiles must stay visible until replacement coverage is ready.
		TileStatus::Ready | TileStatus::Retiring => {
			if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
				record.status = TileStatus::Retiring;
			}
			undesire_source(camera_voxel_loader, CoverageSource::Tile(key));
		}
		// In-flight work may still produce valid voxel data. Keep the record so the
		// late result can be applied, then retire it through the same no-gap path.
		TileStatus::Loading | TileStatus::LoadedWaitingGpu => {}
		// Queued work has not been sent yet, so it can be safely dropped.
		TileStatus::Queued => {
			camera_voxel_loader.tiles.remove(&key);
			remove_source(camera_voxel_loader, CoverageSource::Tile(key));
		}
		// Empty records render nothing. Drop them instead of treating them as coverage.
		TileStatus::Empty => {
			camera_voxel_loader.tiles.remove(&key);
			remove_source(camera_voxel_loader, CoverageSource::Tile(key));
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
) {
	let Ok(grid) = grids.get(chunk.grid) else { return };
	let min = chunk.chunk * CHUNK_SIZE;
	let mut found_subgrid = false;
	for entity in grid.subgrid_entities_in_area(min, IVec3::splat(CHUNK_SIZE)) {
		if subgrids.get(entity).is_err() {
			continue;
		}
		found_subgrid = true;
		if subgrid_gpu.get(entity).is_ok() {
			resolve_visible(camera_voxel_loader, CoverageSource::Chunk(chunk), entity);
		}
	}
	if !found_subgrid {
		resolve_empty(camera_voxel_loader, CoverageSource::Chunk(chunk));
	}
}

pub(crate) fn receive_camera_voxel_loader_results(
	mut commands: Commands,
	mut camera_voxel_loaders: Query<&mut CameraVoxelLoader>,
	mut consumers: Query<&mut CameraVoxelLoaderConsumer>,
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
					if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
						record.status = TileStatus::Empty;
						despawn_stale_entity(&mut commands, record);
					}
					resolve_empty(&mut camera_voxel_loader, CoverageSource::Tile(key));
				}
			}
		}
	}
}

pub(crate) fn refresh_camera_voxel_loader_visibility(
	mut commands: Commands,
	mut gpu_events: MessageReader<VoxelGpuUploadFinished>,
	mut chunk_events: MessageReader<ChunkLoadResolved>,
	mut cameras: Query<(&mut CameraVoxelLoader, &mut VoxelCamera)>, grids: Query<&Grid>,
	subgrids: Query<&SubGrid>, subgrid_gpu: Query<&SubGridGpuState, With<SubGrid>>,
) {
	let gpu_events: Vec<_> = gpu_events.read().copied().collect();
	let chunk_events: Vec<_> = chunk_events.read().copied().collect();
	for (mut camera_voxel_loader, mut request_map) in &mut cameras {
		for event in &chunk_events {
			let source = CoverageSource::Chunk(ChunkKey { grid: event.grid, chunk: event.chunk });
			if !event.visible {
				resolve_empty(&mut camera_voxel_loader, source);
			}
		}
		for event in &gpu_events {
			if let (Ok(subgrid), Ok(gpu_state)) = (subgrids.get(event.entity), subgrid_gpu.get(event.entity)) {
				let placement = gpu_state.placement();
				for chunk in chunks_for_subgrid_bounds(
					subgrid.grid(),
					subgrid.sub_grid_pos(),
					placement.bounds_min.as_ivec3(),
					placement.bounds_max.as_ivec3(),
				) {
					if camera_voxel_loader.desired_chunks.contains(&chunk) {
						resolve_visible(&mut camera_voxel_loader, CoverageSource::Chunk(chunk), event.entity);
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
			resolve_visible(&mut camera_voxel_loader, CoverageSource::Tile(key), event.entity);
			if !camera_voxel_loader.desired_tiles.contains(&key) {
				if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
					record.status = TileStatus::Retiring;
				}
				undesire_source(&mut camera_voxel_loader, CoverageSource::Tile(key));
			}
		}

		let mut subgrids_to_render = Vec::new();
		let mut lods_to_render = Vec::new();
		let mut seen_subgrids = HashSet::new();
		let mut seen_lods = HashSet::new();
		for chunk in camera_voxel_loader.desired_chunks.iter() {
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

pub(crate) fn retire_replaced_tiles(mut commands: Commands, mut camera_voxel_loaders: Query<&mut CameraVoxelLoader>) {
	for mut camera_voxel_loader in &mut camera_voxel_loaders {
		for source in ready_retiring_sources(&camera_voxel_loader) {
			let CoverageSource::Tile(key) = source else { continue };
			if let Some(mut record) = camera_voxel_loader.tiles.remove(&key) {
				despawn_tile_entities(&mut commands, &mut record);
			}
			remove_source(&mut camera_voxel_loader, source);
		}
	}
}

pub(crate) fn retire_replaced_chunks(mut camera_voxel_loaders: Query<(&mut CameraVoxelLoader, &mut CameraVoxelLoaderConsumer)>, mut grids: Query<&mut GridStreaming>) {
	for (mut camera_voxel_loader, mut consumer) in &mut camera_voxel_loaders {
		for source in ready_retiring_sources(&camera_voxel_loader) {
			let CoverageSource::Chunk(chunk) = source else { continue };
			if let Ok(mut streaming) = grids.get_mut(chunk.grid) {
				streaming.release_needed(chunk.grid, consumer.as_mut(), chunk.chunk);
			}
			remove_source(&mut camera_voxel_loader, source);
		}
	}
}
