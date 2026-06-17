use std::collections::HashSet;

use bevy::ecs::message::MessageReader;
use bevy::prelude::*;
use gpu_voxel_data::{SubGridGpuState, VoxelGpuUploadFinished};
use tracy_client::span;
use voxel_data::grid::{Grid, GridId};
use voxel_data::subgrid::SubGrid;
use voxel_renderer::voxel_camera::VoxelCamera;
use voxel_streaming::{ChunkConsumer, ChunkLoadResolved, ChunkRequestChannel, GridStreaming, LodKey, CHUNK_SIZE};

use crate::camera_voxel_loader::CameraVoxelLoader;
use crate::coverage::{
	remove_source, request_source, resolve_empty, resolve_visible, undesire_source, CoverageSource, SourceResolution,
	SourceState,
};
use crate::lod_policy::{nearest_chunk_center, update_desired_sources_delta};
use crate::subgrid_interface::{chunks_in_bounds, collect_subgrids_to_render, resolve_chunk_source_if_ready};
use crate::types::{TileKey, TileRecord, TileStatus};
use crate::CameraVoxelLoaderConsumer;

pub(crate) fn update_camera_voxel_loader_requests(
	chunk_channel: Res<ChunkRequestChannel>,
	mut cameras: Query<(Entity, &Camera, &GlobalTransform, &mut CameraVoxelLoader), With<Camera3d>>,
	mut grids: Query<(GridId, &GlobalTransform, &mut GridStreaming)>, grid_transforms: Query<&GlobalTransform, With<GridStreaming>>,
	grid_data: Query<&Grid>, subgrids: Query<&SubGrid>, subgrid_gpu: Query<&SubGridGpuState, With<SubGrid>>,
) {
	for (camera_entity, camera, camera_global, mut camera_voxel_loader) in &mut cameras {
		if !camera.is_active {
			continue;
		}
		let camera_world = camera_global.translation();
		let settings = camera_voxel_loader.settings.clone();
		let mut add_tiles = Vec::new();
		let mut remove_tiles = Vec::new();

		for (grid, grid_global, streaming) in &mut grids {
			let local = grid_global.affine().inverse().transform_point3(camera_world);
			let camera_chunk = nearest_chunk_center(local);
			let source_delta = update_desired_sources_delta(&mut camera_voxel_loader, grid, camera_chunk, &settings, streaming.as_ref());
			add_tiles.extend(source_delta.tiles.added);
			remove_tiles.extend(source_delta.tiles.removed);
		}

		apply_desired_delta(
			&mut camera_voxel_loader,
			&mut grids.transmute_lens::<&mut GridStreaming>().query(),
			&chunk_channel,
			&grid_data,
			&subgrids,
			&subgrid_gpu,
			add_tiles,
			remove_tiles,
			camera_entity,
		);
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
			let requested = grids
				.get_mut(key.grid)
				.map(|(_, _, mut streaming)| streaming.fetch_lod(camera_entity, lod_key(key), priority))
				.unwrap_or(false);
			if requested {
				if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
					record.status = TileStatus::Loading;
				}
				sent += 1;
			} else {
				camera_voxel_loader.tiles.remove(&key);
				let ready = resolve_empty(&mut camera_voxel_loader, key);
				retire_sources_from_request_query(&mut camera_voxel_loader, &mut grids.transmute_lens::<&mut GridStreaming>().query(), camera_entity, ready);
			}
		}
	}
}

fn apply_desired_delta(
	camera_voxel_loader: &mut CameraVoxelLoader,
	grids: &mut Query<&mut GridStreaming>, chunk_channel: &ChunkRequestChannel, grid_data: &Query<&Grid>,
	subgrids: &Query<&SubGrid>, subgrid_gpu: &Query<&SubGridGpuState, With<SubGrid>>, add_tiles: Vec<TileKey>, remove_tiles: Vec<TileKey>, camera_entity: Entity,
) {
	for key in add_tiles {
		if key.is_chunk() {
			request_source(camera_voxel_loader, key);
			if let Ok(mut streaming) = grids.get_mut(key.grid) {
				streaming.fetch(key.grid, chunk_channel, key.min);
				if matches!(streaming.state(key.min), Some(voxel_streaming::ChunkState::Loaded | voxel_streaming::ChunkState::InternalDirty)) {
					let ready = resolve_chunk_source_if_ready(camera_voxel_loader, grid_data, subgrids, subgrid_gpu, key);
					retire_sources_from_request_query(camera_voxel_loader, grids, camera_entity, ready);
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
					retire_sources_from_request_query(camera_voxel_loader, grids, camera_entity, ready);
				}
			}
		} else {
			queue_tile_if_missing(camera_voxel_loader, key);
		}
	}

	for key in remove_tiles {
		let ready = handle_non_desired_tile(camera_voxel_loader, key);
		retire_sources_from_request_query(camera_voxel_loader, grids, camera_entity, ready);
	}
}

fn in_flight_count(camera_voxel_loader: &CameraVoxelLoader) -> usize {
	let _span = span!("in_flight_count");
	camera_voxel_loader.tiles.values().filter(|r| r.status == TileStatus::Loading).count()
}

fn retire_sources(
	camera_voxel_loader: &mut CameraVoxelLoader,
	grids: &mut Query<&mut GridStreaming>,
	requester: Entity,
	sources: impl IntoIterator<Item = CoverageSource>,
) {
	for source in sources {
		remove_source(camera_voxel_loader, source);
		if source.is_chunk() {
			if let Ok(mut streaming) = grids.get_mut(source.grid) { streaming.release(source.min); }
		} else {
			if let Ok(mut streaming) = grids.get_mut(source.grid) { streaming.release_lod(requester, lod_key(source)); }
			camera_voxel_loader.tiles.remove(&source);
		}
	}
}

fn retire_sources_from_request_query(
	camera_voxel_loader: &mut CameraVoxelLoader,
	grid_streamings: &mut Query<&mut GridStreaming>,
	requester: Entity,
	sources: impl IntoIterator<Item = CoverageSource>,
) {
	for source in sources {
		remove_source(camera_voxel_loader, source);
		if source.is_chunk() {
			if let Ok(mut streaming) = grid_streamings.get_mut(source.grid) { streaming.release(source.min); }
		} else {
			if let Ok(mut streaming) = grid_streamings.get_mut(source.grid) { streaming.release_lod(requester, lod_key(source)); }
			camera_voxel_loader.tiles.remove(&source);
		}
	}
}

fn queue_tile_if_missing(camera_voxel_loader: &mut CameraVoxelLoader, key: TileKey) {
	if camera_voxel_loader.tiles.contains_key(&key) || tile_coverage_is_desired_empty(camera_voxel_loader, key) { return; }
	camera_voxel_loader.tiles.insert(key, TileRecord::queued());
	request_source(camera_voxel_loader, key);
	camera_voxel_loader.queue.push_back(key);
}

fn tile_coverage_is_desired_empty(camera_voxel_loader: &CameraVoxelLoader, key: TileKey) -> bool {
	matches!(camera_voxel_loader.coverage_sources.get(&key).map(|record| record.state), Some(SourceState::Desired(SourceResolution::Empty)))
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
		TileStatus::Loading => Vec::new(),
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

fn lod_key(key: TileKey) -> LodKey {
	LodKey { min: key.min, size: key.size(), lod: key.lod }
}

pub(crate) fn receive_camera_voxel_loader_results(
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
			let key = TileKey { grid: result.grid, lod: result.key.lod, min: result.key.min };
			let Some(_) = camera_voxel_loader.tiles.get(&key) else { continue };
			match result.entity {
				Some(entity) => {
					if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
						record.entity = Some(entity);
						record.status = TileStatus::Ready;
					}
					let ready = resolve_visible(&mut camera_voxel_loader, key, entity);
					retire_sources(&mut camera_voxel_loader, &mut grids, result.requester, ready);
					if !camera_voxel_loader.desired_tiles.contains(&key) {
						if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) { record.status = TileStatus::Retiring; }
						let ready = undesire_source(&mut camera_voxel_loader, key);
						retire_sources(&mut camera_voxel_loader, &mut grids, result.requester, ready);
					}
				}
				None => {
					camera_voxel_loader.tiles.remove(&key);
					let ready = resolve_empty(&mut camera_voxel_loader, key);
					retire_sources(&mut camera_voxel_loader, &mut grids, result.requester, ready);
					if !camera_voxel_loader.desired_tiles.contains(&key) {
						remove_source(&mut camera_voxel_loader, key);
						if let Ok(mut streaming) = grids.get_mut(key.grid) { streaming.release_lod(result.requester, lod_key(key)); }
					}
				}
			}
		}
	}
}

pub(crate) fn refresh_camera_voxel_loader_visibility(
	mut gpu_events: MessageReader<VoxelGpuUploadFinished>,
	mut chunk_events: MessageReader<ChunkLoadResolved>,
	mut cameras: Query<(Entity, &mut CameraVoxelLoader, &mut VoxelCamera)>,
	grids: Query<&Grid>,
	mut streaming_grids: Query<&mut GridStreaming>,
	subgrids: Query<&SubGrid>,
	subgrid_gpu: Query<&SubGridGpuState, With<SubGrid>>,
) {
	let gpu_events: Vec<_> = gpu_events.read().copied().collect();
	let chunk_events: Vec<_> = chunk_events.read().copied().collect();
	for (camera_entity, mut camera_voxel_loader, mut request_map) in &mut cameras {
		for event in &chunk_events {
			let source = TileKey::chunk(event.grid, event.chunk);
			if !event.visible {
				let ready = resolve_empty(&mut camera_voxel_loader, source);
				retire_sources(&mut camera_voxel_loader, &mut streaming_grids, camera_entity, ready);
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
						retire_sources(&mut camera_voxel_loader, &mut streaming_grids, camera_entity, ready);
					}
				}
			}
		}

		let subgrids_to_render = collect_subgrids_to_render(&camera_voxel_loader, &grids, &subgrids);
		let mut lods_to_render = Vec::new();
		let mut seen_lods = HashSet::new();
		for record in camera_voxel_loader.tiles.values() {
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

#[cfg(test)]
mod tests {
	use bevy::prelude::*;

	use super::*;
	use crate::camera_voxel_loader::CameraVoxelLoader;
	use crate::coverage::request_source;

	#[test]
	fn undesired_loading_lod_waits_for_result_until_dependency_retirement() {
		let grid = Entity::from_bits(1);
		let key = TileKey { grid, lod: 1, min: IVec3::ZERO };
		let mut loader = CameraVoxelLoader::default();
		loader.tiles.insert(key, TileRecord { status: TileStatus::Loading, entity: None });
		request_source(&mut loader, key);

		let ready = handle_non_desired_tile(&mut loader, key);

		assert!(ready.is_empty());
		assert!(loader.tiles.contains_key(&key), "loading LOD records stay until the in-flight result resolves because release_lod is delayed until dependency retirement");
		assert!(loader.coverage_sources.contains_key(&key), "coverage remains requested so the late result can satisfy replacement dependencies");
	}
}
