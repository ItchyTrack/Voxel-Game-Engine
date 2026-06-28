use std::collections::HashSet;

use bevy::ecs::message::MessageReader;
use bevy::prelude::*;
use voxel_gpu::{SubGridGpuState, VoxelGpuUploadFinished};
use tracy_client::span;
use voxel_data::grid::{Grid, GridId};
use voxel_data::subgrid::SubGrid;
use voxel_streaming::{ChunkAvailabilityChangeKind, ChunkAvailabilityChanged, ChunkConsumer, ChunkLoadResolved, GridStreaming, LodKey, VoxelSourceRequestApi, VoxelSourceRequests, CHUNK_SIZE};

use crate::camera_voxel_loader::CameraVoxelLoader;
use crate::coverage::{
	remove_source, request_source, resolve_empty, resolve_visible, undesire_source, CoverageSource, SourceResolution,
	SourceState,
};
use crate::lod_policy::{nearest_chunk_center, tile_has_present_source, update_desired_sources_delta};
use crate::subgrid_interface::{chunks_in_bounds, collect_subgrids_to_render, resolve_chunk_source_if_ready};
use crate::types::{TileKey, TileRecord, TileStatus};
use crate::{CameraVoxelLoaderConsumer, CameraVoxelRenderState};

pub(crate) fn update_camera_voxel_loader_requests(
	requests: VoxelSourceRequests,
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
			&requests,
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
	grids: &mut Query<&mut GridStreaming>, requests: &impl VoxelSourceRequestApi, grid_data: &Query<&Grid>,
	subgrids: &Query<&SubGrid>, subgrid_gpu: &Query<&SubGridGpuState, With<SubGrid>>, add_tiles: Vec<TileKey>, remove_tiles: Vec<TileKey>, camera_entity: Entity,
) {
	for key in add_tiles {
		if key.is_chunk() {
			request_source(camera_voxel_loader, key);
			if let Ok(mut streaming) = grids.get_mut(key.grid) {
				streaming.fetch(key.grid, requests, key.min);
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
	requests: VoxelSourceRequests,
	mut gpu_events: MessageReader<VoxelGpuUploadFinished>,
	mut chunk_events: MessageReader<ChunkLoadResolved>,
	mut availability_events: MessageReader<ChunkAvailabilityChanged>,
	mut cameras: Query<(Entity, &mut CameraVoxelLoader, &mut CameraVoxelRenderState)>,
	grids: Query<&Grid>,
	mut streaming_grids: Query<&mut GridStreaming>,
	subgrids: Query<&SubGrid>,
	subgrid_gpu: Query<&SubGridGpuState, With<SubGrid>>,
) {
	let gpu_events: Vec<_> = gpu_events.read().copied().collect();
	let chunk_events: Vec<_> = chunk_events.read().copied().collect();
	let availability_events: Vec<_> = availability_events.read().copied().collect();
	for (camera_entity, mut camera_voxel_loader, mut request_map) in &mut cameras {
		for event in &availability_events {
			handle_chunk_availability_changed(camera_entity, &mut camera_voxel_loader, &mut streaming_grids, &requests, *event);
		}
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

fn handle_chunk_availability_changed(
	camera_entity: Entity,
	camera_voxel_loader: &mut CameraVoxelLoader,
	streaming_grids: &mut Query<&mut GridStreaming>,
	requests: &impl VoxelSourceRequestApi,
	event: ChunkAvailabilityChanged,
) {
	match event.kind {
		ChunkAvailabilityChangeKind::BecamePresent => {
			let candidates = newly_desired_tiles_in_area(camera_voxel_loader, event.grid, event.min, event.size);
			for key in candidates {
				let Ok(mut streaming) = streaming_grids.get_mut(key.grid) else { continue; };
				let has_present = tile_has_present_source(streaming.as_ref(), key);
				if !has_present {
					continue;
				}
				let inserted = camera_voxel_loader.desired_tiles.insert(key);
				if !inserted {
					continue;
				}
				if key.is_chunk() {
					request_source(camera_voxel_loader, key);
					streaming.fetch(key.grid, requests, key.min);
				} else {
					queue_tile_if_missing(camera_voxel_loader, key);
				}
			}
		}
		ChunkAvailabilityChangeKind::BecameEmpty => {
			let affected: Vec<_> = camera_voxel_loader
				.desired_tiles
				.iter()
				.copied()
				.filter(|key| key.grid == event.grid && tiles_overlap_area(*key, event.min, event.size))
				.collect();
			for key in affected {
				let Ok(streaming) = streaming_grids.get_mut(key.grid) else { continue; };
				let still_present = tile_has_present_source(streaming.as_ref(), key);
				if still_present {
					continue;
				}
				camera_voxel_loader.desired_tiles.remove(&key);
				if key.is_chunk() {
					let ready = resolve_empty(camera_voxel_loader, key);
					retire_sources_from_request_query(camera_voxel_loader, streaming_grids, camera_entity, ready);
					if let Ok(mut streaming) = streaming_grids.get_mut(key.grid) { streaming.release(key.min); }
				} else {
					camera_voxel_loader.tiles.remove(&key);
					let ready = resolve_empty(camera_voxel_loader, key);
					retire_sources_from_request_query(camera_voxel_loader, streaming_grids, camera_entity, ready);
					if let Ok(mut streaming) = streaming_grids.get_mut(key.grid) { streaming.release_lod(camera_entity, lod_key(key)); }
				}
			}
		}
	}
}

fn newly_desired_tiles_in_area(camera_voxel_loader: &CameraVoxelLoader, grid: GridId, min: IVec3, size: IVec3) -> Vec<TileKey> {
	let Some(bands) = camera_voxel_loader.bands.get(&grid) else {
		return Vec::new();
	};
	let mut candidates = Vec::new();
	for band in bands {
		let tile_size = 1i32 << band.lod;
		let event_box_min = min;
		let event_box_max = min + size;
		let band_min = band.outer.min;
		let band_max = band.outer.max;
		let overlap_min = event_box_min.max(band_min);
		let overlap_max = event_box_max.min(band_max);
		if overlap_min.cmpge(overlap_max).any() {
			continue;
		}
		let mut x = overlap_min.x.div_euclid(tile_size) * tile_size;
		while x < overlap_max.x {
			let mut y = overlap_min.y.div_euclid(tile_size) * tile_size;
			while y < overlap_max.y {
				let mut z = overlap_min.z.div_euclid(tile_size) * tile_size;
				while z < overlap_max.z {
					let key = TileKey { grid, lod: band.lod, min: IVec3::new(x, y, z) };
					let already_desired = camera_voxel_loader.desired_tiles.contains(&key);
					let in_band = is_tile_in_band(*band, key.min);
					if !already_desired && in_band {
						candidates.push(key);
					}
					z += tile_size;
				}
				y += tile_size;
			}
			x += tile_size;
		}
	}
	candidates
}

fn is_tile_in_band(band: crate::lod_bands::LodBand, min: IVec3) -> bool {
	let tile_size = 1i32 << band.lod;
	let tile_max = min + IVec3::splat(tile_size);
	let in_outer = min.cmpge(band.outer.min).all() && tile_max.cmple(band.outer.max).all();
	let outside_inner = band.inner.is_none_or(|inner| !(min.cmpge(inner.min).all() && tile_max.cmple(inner.max).all()));
	in_outer && outside_inner
}

fn tiles_overlap_area(key: TileKey, min: IVec3, size: IVec3) -> bool {
	let key_min = key.min;
	let key_max = key.min + key.size();
	let area_max = min + size;
	key_min.cmplt(area_max).all() && min.cmplt(key_max).all()
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
