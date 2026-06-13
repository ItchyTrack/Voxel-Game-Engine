use std::collections::HashSet;

use bevy::prelude::*;
use gpu_voxel_data::{LodVoxels, SubGridGpuState};
use voxel_data::grid::{Grid, GridId};
use voxel_data::subgrid::SubGrid;
use voxel_renderer::voxel_camera::VoxelCamera;
use voxel_streaming::{ChunkConsumer, ChunkRequestChannel, GridStreaming, LodRequestChannel, CHUNK_SIZE};

use crate::camera_voxel_loader::CameraVoxelLoader;
use crate::lod_policy::{add_lod_tiles, add_near_chunks, nearest_chunk_center};
use crate::retirement::{cancel_retirement, notify_tile_resolved, ready_retiring_chunks, ready_retiring_tiles, register_chunk_retirement, register_tile_retirement, RetireTarget};
use crate::types::{ChunkKey, TileKey, TileRecord, TileStatus};
use crate::CameraVoxelLoaderConsumer;

pub(crate) fn update_camera_voxel_loader_requests(
	chunk_channel: Res<ChunkRequestChannel>, lod_channel: Res<LodRequestChannel>,
	mut cameras: Query<(Entity, &Camera, &GlobalTransform, &mut CameraVoxelLoader, &mut CameraVoxelLoaderConsumer), With<Camera3d>>,
	mut grids: Query<(GridId, &GlobalTransform, &mut GridStreaming)>, grid_transforms: Query<&GlobalTransform, With<GridStreaming>>,
) {
	for (camera_entity, camera, camera_global, mut camera_voxel_loader, mut consumer) in &mut cameras {
		if !camera.is_active {
			continue;
		}
		let camera_world = camera_global.translation();
		let mut desired_chunks = HashSet::new();
		let mut desired_tiles = HashSet::new();

		for (grid, grid_global, streaming) in &mut grids {
			let local = grid_global.affine().inverse().transform_point3(camera_world);
			let camera_chunk = nearest_chunk_center(local);
			camera_voxel_loader.grid_centers.insert(grid, camera_chunk);
			add_near_chunks(&mut desired_chunks, grid, camera_chunk, &camera_voxel_loader);
			add_lod_tiles(&mut desired_tiles, grid, camera_chunk, &camera_voxel_loader, streaming.as_ref());
		}

		camera_voxel_loader.desired_tiles = desired_tiles.clone();

		let chunks_to_fetch = update_desired_chunks(&mut camera_voxel_loader, desired_chunks);
		for chunk_key in chunks_to_fetch {
			if let Ok((_, _, mut streaming)) = grids.get_mut(chunk_key.grid) {
				streaming.fetch_needed(chunk_key.grid, consumer.as_mut(), &chunk_channel, chunk_key.chunk);
			}
		}

		let old_tiles: Vec<TileKey> = camera_voxel_loader.tiles.keys().copied().collect();
		for key in old_tiles {
			if desired_tiles.contains(&key) {
				if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
					if record.status == TileStatus::Retiring {
						record.status = if record.entity.is_some() { TileStatus::Ready } else { TileStatus::Empty };
						cancel_retirement(&mut camera_voxel_loader, RetireTarget::Tile(key));
					}
				}
				continue;
			}

			handle_non_desired_tile(&mut camera_voxel_loader, key);
		}

		for key in desired_tiles {
			if camera_voxel_loader.tiles.contains_key(&key) {
				continue;
			}
			camera_voxel_loader.tiles.insert(key, TileRecord { status: TileStatus::Queued, entity: None });
			camera_voxel_loader.queue.push_back(key);
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
			let requested = grids
				.get(key.grid)
				.map(|(_, _, streaming)| streaming.fetch_lod(key.grid, camera_entity, &lod_channel, key.min, key.size(), key.lod as f32, priority))
				.unwrap_or(false);
			if requested {
				if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
					record.status = TileStatus::Loading;
				}
				sent += 1;
			} else {
				if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
					record.status = TileStatus::Empty;
				}
				notify_tile_resolved(&mut camera_voxel_loader, key);
			}
		}
	}
}

fn in_flight_count(camera_voxel_loader: &CameraVoxelLoader) -> usize {
	camera_voxel_loader.tiles.values().filter(|r| r.status == TileStatus::Loading).count()
}

fn update_desired_chunks(camera_voxel_loader: &mut CameraVoxelLoader, desired_chunks: HashSet<ChunkKey>) -> Vec<ChunkKey> {
	let old_chunks = camera_voxel_loader.desired_chunks.clone();
	for &chunk_key in old_chunks.difference(&desired_chunks) {
		register_chunk_retirement(camera_voxel_loader, chunk_key);
	}

	let chunks_to_fetch: Vec<ChunkKey> = desired_chunks.difference(&old_chunks).copied().collect();
	for chunk_key in &chunks_to_fetch {
		cancel_retirement(camera_voxel_loader, RetireTarget::Chunk(*chunk_key));
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
			register_tile_retirement(camera_voxel_loader, key);
		}
		// In-flight work may still produce valid voxel data. Keep the record so the
		// late result can be applied, then retire it through the same no-gap path.
		TileStatus::Loading | TileStatus::LoadedWaitingGpu => {}
		// Queued work has not been sent yet, so it can be safely dropped.
		TileStatus::Queued => {
			camera_voxel_loader.tiles.remove(&key);
			cancel_retirement(camera_voxel_loader, RetireTarget::Tile(key));
		}
		// Empty records render nothing. Drop them instead of treating them as coverage.
		TileStatus::Empty => {
			camera_voxel_loader.tiles.remove(&key);
			cancel_retirement(camera_voxel_loader, RetireTarget::Tile(key));
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
			if record.entity.is_some() {
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
				}
				_ => {
					if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
						record.status = TileStatus::Empty;
					}
					notify_tile_resolved(&mut camera_voxel_loader, key);
				}
			}
		}
	}
}

pub(crate) fn refresh_camera_voxel_loader_visibility(
	mut cameras: Query<(&mut CameraVoxelLoader, &mut VoxelCamera)>, lod_gpu: Query<&SubGridGpuState, With<LodVoxels>>, grids: Query<&Grid>,
	subgrids: Query<&SubGrid>,
) {
	for (mut camera_voxel_loader, mut request_map) in &mut cameras {
		let waiting_for_gpu: Vec<TileKey> = camera_voxel_loader
			.tiles
			.iter()
			.filter_map(|(key, record)| {
				(record.status == TileStatus::LoadedWaitingGpu && record.entity.is_some_and(|e| lod_gpu.get(e).is_ok())).then_some(*key)
			})
			.collect();
		for key in waiting_for_gpu {
			if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
				record.status = TileStatus::Ready;
			}
			notify_tile_resolved(&mut camera_voxel_loader, key);
			if !camera_voxel_loader.desired_tiles.contains(&key) {
				if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
					record.status = TileStatus::Retiring;
				}
				register_tile_retirement(&mut camera_voxel_loader, key);
			}
		}

		let mut subgrids_to_render = Vec::new();
		let mut lods_to_render = Vec::new();
		let mut seen_subgrids = HashSet::new();
		let mut seen_lods = HashSet::new();
		for chunk in camera_voxel_loader.desired_chunks.iter().chain(camera_voxel_loader.retiring_chunks.keys()) {
			let Ok(grid) = grids.get(chunk.grid) else { continue };
			let min = chunk.chunk * CHUNK_SIZE;
			for entity in grid.subgrid_entities_in_area(min, IVec3::splat(CHUNK_SIZE)) {
				if subgrids.get(entity).is_ok() && seen_subgrids.insert(entity) {
					subgrids_to_render.push(entity);
				}
			}
		}

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

pub(crate) fn retire_replaced_tiles(mut commands: Commands, mut camera_voxel_loaders: Query<&mut CameraVoxelLoader>) {
	for mut camera_voxel_loader in &mut camera_voxel_loaders {
		for key in ready_retiring_tiles(&camera_voxel_loader) {
			cancel_retirement(&mut camera_voxel_loader, RetireTarget::Tile(key));
			if let Some(record) = camera_voxel_loader.tiles.remove(&key) {
				if let Some(entity) = record.entity {
					commands.entity(entity).despawn();
				}
			}
		}
	}
}

pub(crate) fn retire_replaced_chunks(mut camera_voxel_loaders: Query<(&mut CameraVoxelLoader, &mut CameraVoxelLoaderConsumer)>, mut grids: Query<&mut GridStreaming>) {
	for (mut camera_voxel_loader, mut consumer) in &mut camera_voxel_loaders {
		for chunk in ready_retiring_chunks(&camera_voxel_loader) {
			cancel_retirement(&mut camera_voxel_loader, RetireTarget::Chunk(chunk));
			if let Ok(mut streaming) = grids.get_mut(chunk.grid) {
				streaming.release_needed(chunk.grid, consumer.as_mut(), chunk.chunk);
			}
		}
	}
}
