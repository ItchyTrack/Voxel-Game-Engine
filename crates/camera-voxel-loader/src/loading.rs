use std::collections::HashSet;

use bevy::ecs::message::MessageReader;
use bevy::prelude::*;
use gpu_voxel_data::{LodVoxels, SubGridGpuState, VoxelGpuUploadFinished};
use voxel_data::grid::{Grid, GridId};
use voxel_data::subgrid::SubGrid;
use voxel_renderer::voxel_camera::VoxelCamera;
use voxel_streaming::{
	ChunkBecameDirty, ChunkBecamePresent, ChunkConsumer, ChunkLoadResolved, ChunkRequestChannel, GridStreaming, LodRequestChannel, CHUNK_SIZE,
};

use crate::camera_voxel_loader::CameraVoxelLoader;
use crate::coverage::{
	remove_source, renew_source_request, request_source, resolve_empty, resolve_visible, undesire_source, CoverageSource, SourceResolution,
	SourceState,
};
use crate::lod_policy::{is_tile_wanted, nearest_chunk_center, tile_key_covering_chunk, update_desired_sources_delta};
use crate::subgrid_interface::{chunks_in_bounds, collect_subgrids_to_render, resolve_chunk_source_if_ready};
use crate::types::{TileKey, TileRecord, TileStatus};
use crate::CameraVoxelLoaderConsumer;

pub(crate) fn update_camera_voxel_loader_requests(
	mut commands: Commands, chunk_channel: Res<ChunkRequestChannel>, lod_channel: Res<LodRequestChannel>,
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
		let mut desired_delta = DesiredDelta::default();

		for (grid, grid_global, streaming) in &mut grids {
			let local = grid_global.affine().inverse().transform_point3(camera_world);
			let camera_chunk = nearest_chunk_center(local);
			let previous_center = camera_voxel_loader.grid_centers.get(&grid).copied();
			let loader = camera_voxel_loader.as_mut();
			let source_delta = update_desired_sources_delta(
				&mut loader.desired_tiles,
				grid,
				previous_center,
				camera_chunk,
				settings_changed,
				&settings,
				streaming.as_ref(),
			);
			desired_delta.add_tiles.extend(source_delta.tiles.added);
			desired_delta.remove_tiles.extend(source_delta.tiles.removed);

			for event in present_events.iter().filter(|event| event.grid == grid) {
				let key = TileKey::chunk(grid, event.chunk);
				if is_tile_wanted(&settings, streaming.as_ref(), camera_chunk, key) && camera_voxel_loader.desired_tiles.insert(key) {
					desired_delta.add_tiles.push(key);
				}
				for lod in 1..=settings.max_lod {
					let key = tile_key_covering_chunk(grid, event.chunk, lod);
					if is_tile_wanted(&settings, streaming.as_ref(), camera_chunk, key) {
						if camera_voxel_loader.desired_tiles.insert(key) {
							desired_delta.add_tiles.push(key);
						} else {
							requeue_empty_or_missing_tile(&mut camera_voxel_loader, key);
						}
					}
				}
			}

			camera_voxel_loader.grid_centers.insert(grid, camera_chunk);
		}

		camera_voxel_loader.applied_settings = Some(settings);
		apply_desired_delta(
			&mut commands,
			&mut camera_voxel_loader,
			consumer.as_mut(),
			&mut grids,
			&chunk_channel,
			&grid_data,
			&subgrids,
			&subgrid_gpu,
			desired_delta,
		);
		apply_chunk_change_events(&mut commands, &mut camera_voxel_loader, &dirty_events);

		let mut sent = 0usize;
		while sent < camera_voxel_loader.settings.requests_per_frame
			&& in_flight_count(&camera_voxel_loader) < camera_voxel_loader.settings.max_in_flight
		{
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
				.map(|(_, _, streaming)| {
					streaming.fetch_lod(key.grid, camera_entity, &lod_channel, key.min, key.size(), key.lod as f32, priority, generation)
				})
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
				let ready = resolve_empty(&mut camera_voxel_loader, key);
				retire_sources_from_request_query(&mut commands, &mut camera_voxel_loader, consumer.as_mut(), &mut grids, ready);
			}
		}
	}
}

#[derive(Default)]
struct DesiredDelta {
	add_tiles: Vec<TileKey>,
	remove_tiles: Vec<TileKey>,
}

fn apply_desired_delta(
	commands: &mut Commands, camera_voxel_loader: &mut CameraVoxelLoader, consumer: &mut CameraVoxelLoaderConsumer,
	grids: &mut Query<(GridId, &GlobalTransform, &mut GridStreaming)>, chunk_channel: &ChunkRequestChannel, grid_data: &Query<&Grid>,
	subgrids: &Query<&SubGrid>, subgrid_gpu: &Query<&SubGridGpuState, With<SubGrid>>, delta: DesiredDelta,
) {
	for key in delta.add_tiles {
		if key.is_chunk() {
			request_source(camera_voxel_loader, key);
			if let Ok((_, _, mut streaming)) = grids.get_mut(key.grid) {
				streaming.fetch_needed(key.grid, consumer, chunk_channel, key.min);
				if matches!(streaming.state(key.min), Some(voxel_streaming::ChunkState::Loaded | voxel_streaming::ChunkState::InternalDirty)) {
					let ready = resolve_chunk_source_if_ready(camera_voxel_loader, grid_data, subgrids, subgrid_gpu, key);
					retire_sources_from_request_query(commands, camera_voxel_loader, consumer, grids, ready);
				}
			}
		} else if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
			if record.status == TileStatus::Retiring {
				record.status = TileStatus::Ready;
			}
			if record.status == TileStatus::Ready {
				if let Some(entity) = record.entity {
					request_source(camera_voxel_loader, key);
					let ready = resolve_visible(camera_voxel_loader, key, entity);
					retire_sources_from_request_query(commands, camera_voxel_loader, consumer, grids, ready);
				}
			}
		} else {
			queue_tile_if_missing(camera_voxel_loader, key);
		}
	}

	for key in delta.remove_tiles {
		let ready = handle_non_desired_tile(camera_voxel_loader, key);
		retire_sources_from_request_query(commands, camera_voxel_loader, consumer, grids, ready);
	}
}

fn in_flight_count(camera_voxel_loader: &CameraVoxelLoader) -> usize {
	camera_voxel_loader.tiles.values().filter(|r| r.status == TileStatus::Loading).count()
}

fn retire_sources(
	commands: &mut Commands, camera_voxel_loader: &mut CameraVoxelLoader, consumer: &mut CameraVoxelLoaderConsumer,
	grids: &mut Query<&mut GridStreaming>, sources: impl IntoIterator<Item = CoverageSource>,
) {
	retire_sources_with(commands, camera_voxel_loader, sources, |chunk| {
		if let Ok(mut streaming) = grids.get_mut(chunk.grid) {
			streaming.release_needed(chunk.grid, consumer, chunk.min);
		}
	});
}

fn retire_sources_from_request_query(
	commands: &mut Commands, camera_voxel_loader: &mut CameraVoxelLoader, consumer: &mut CameraVoxelLoaderConsumer,
	grids: &mut Query<(GridId, &GlobalTransform, &mut GridStreaming)>, sources: impl IntoIterator<Item = CoverageSource>,
) {
	retire_sources_with(commands, camera_voxel_loader, sources, |chunk| {
		if let Ok((_, _, mut streaming)) = grids.get_mut(chunk.grid) {
			streaming.release_needed(chunk.grid, consumer, chunk.min);
		}
	});
}

fn retire_sources_with(
	commands: &mut Commands, camera_voxel_loader: &mut CameraVoxelLoader, sources: impl IntoIterator<Item = CoverageSource>,
	mut release_chunk: impl FnMut(TileKey),
) {
	for source in sources {
		remove_source(camera_voxel_loader, source);
		if source.is_chunk() {
			release_chunk(source);
		} else if let Some(mut record) = camera_voxel_loader.tiles.remove(&source) {
			despawn_tile_entities(commands, &mut record);
		}
	}
}

fn apply_chunk_change_events(commands: &mut Commands, camera_voxel_loader: &mut CameraVoxelLoader, dirty_events: &[ChunkBecameDirty]) {
	let max_lod = camera_voxel_loader.settings.max_lod;
	for event in dirty_events {
		for lod in 1..=max_lod {
			let key = tile_key_covering_chunk(event.grid, event.chunk, lod);
			invalidate_dirty_tile(commands, camera_voxel_loader, key, camera_voxel_loader.desired_tiles.contains(&key));
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
	request_source(camera_voxel_loader, key);
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
		remove_source(camera_voxel_loader, key);
	}
}

fn tile_coverage_is_desired_empty(camera_voxel_loader: &CameraVoxelLoader, key: TileKey) -> bool {
	matches!(camera_voxel_loader.coverage_sources.get(&key).map(|record| record.state), Some(SourceState::Desired(SourceResolution::Empty)))
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
				renew_source_request(camera_voxel_loader, key);
				camera_voxel_loader.queue.push_back(key);
			} else {
				despawn_tile_entities(commands, record);
				camera_voxel_loader.tiles.remove(&key);
				remove_source(camera_voxel_loader, key);
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
				renew_source_request(camera_voxel_loader, key);
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

fn handle_non_desired_tile(camera_voxel_loader: &mut CameraVoxelLoader, key: TileKey) -> Vec<CoverageSource> {
	if key.is_chunk() {
		return undesire_source(camera_voxel_loader, key);
	}
	let Some(status) = camera_voxel_loader.tiles.get(&key).map(|record| record.status) else { return Vec::new() };
	match status {
		// Already-renderable tiles must stay visible until replacement coverage is ready.
		TileStatus::Ready | TileStatus::Retiring => {
			if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
				record.status = TileStatus::Retiring;
			}
			undesire_source(camera_voxel_loader, key)
		}
		// In-flight work may still produce valid voxel data. Keep the record so the
		// late result can be applied, then retire it through the same no-gap path.
		TileStatus::Loading | TileStatus::LoadedWaitingGpu => Vec::new(),
		// Queued work has not been sent yet, so it can be safely dropped.
		TileStatus::Queued => {
			camera_voxel_loader.tiles.remove(&key);
			remove_source(camera_voxel_loader, key);
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

pub(crate) fn receive_camera_voxel_loader_results(
	mut commands: Commands, mut camera_voxel_loaders: Query<&mut CameraVoxelLoader>, mut consumers: Query<&mut CameraVoxelLoaderConsumer>,
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
					let ready = resolve_empty(&mut camera_voxel_loader, key);
					retire_sources(&mut commands, &mut camera_voxel_loader, consumer.as_mut(), &mut grids, ready);
				}
			}
		}
	}
}

pub(crate) fn refresh_camera_voxel_loader_visibility(
	mut commands: Commands, mut gpu_events: MessageReader<VoxelGpuUploadFinished>, mut chunk_events: MessageReader<ChunkLoadResolved>,
	mut cameras: Query<(&mut CameraVoxelLoader, &mut CameraVoxelLoaderConsumer, &mut VoxelCamera)>, grids: Query<&Grid>,
	mut streaming_grids: Query<&mut GridStreaming>, subgrids: Query<&SubGrid>, subgrid_gpu: Query<&SubGridGpuState, With<SubGrid>>,
) {
	let gpu_events: Vec<_> = gpu_events.read().copied().collect();
	let chunk_events: Vec<_> = chunk_events.read().copied().collect();
	for (mut camera_voxel_loader, mut consumer, mut request_map) in &mut cameras {
		for event in &chunk_events {
			let source = TileKey::chunk(event.grid, event.chunk);
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
					if camera_voxel_loader.desired_tiles.contains(&chunk) {
						let ready = resolve_visible(&mut camera_voxel_loader, chunk, event.entity);
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
			let ready = resolve_visible(&mut camera_voxel_loader, key, event.entity);
			retire_sources(&mut commands, &mut camera_voxel_loader, consumer.as_mut(), &mut streaming_grids, ready);
			if !camera_voxel_loader.desired_tiles.contains(&key) {
				if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
					record.status = TileStatus::Retiring;
				}
				let ready = undesire_source(&mut camera_voxel_loader, key);
				retire_sources(&mut commands, &mut camera_voxel_loader, consumer.as_mut(), &mut streaming_grids, ready);
			}
		}

		let subgrids_to_render = collect_subgrids_to_render(&camera_voxel_loader, &grids, &subgrids);
		let mut lods_to_render = Vec::new();
		let mut seen_lods = HashSet::new();
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
