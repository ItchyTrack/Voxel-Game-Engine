use bevy::{ecs::message::MessageReader, prelude::*};
use voxel_data::{grid::{Grid, GridId}, subgrid::SubGrid};
use voxel_gpu::{VoxelGpuFormat, VoxelGpuState, VoxelGpuUploadFinished};
use voxel_streaming::{
	CHUNK_SIZE,
	ChunkAvailabilityChangeKind,
	ChunkAvailabilityChanged,
	ChunkConsumer,
	ChunkLoadResolved,
	GridStreaming,
	VoxelSourceRequests,
};

use crate::{
	CameraVoxelLoaderConsumer,
	camera_voxel_loader::CameraVoxelLoader,
	lod_bands::for_each_tile_in_bands,
	lod_policy::{nearest_chunk_center, tile_has_present_source, update_desired_sources_delta},
	runtime::{acquire_source, chunk_resolution, release_sources},
	tile_lifecycle::ResolvedTile,
	types::TileKey,
};

pub(crate) fn update_camera_voxel_loader_requests(
	requests: VoxelSourceRequests,
	mut cameras: Query<(Entity, &Camera, &GlobalTransform, Option<&VoxelGpuFormat>, &mut CameraVoxelLoader), With<Camera3d>>,
	mut grids: Query<(GridId, &GlobalTransform, &Grid, &mut GridStreaming)>, subgrids: Query<&SubGrid>, gpu_state: Query<&VoxelGpuState>,
) {
	for (camera_entity, camera, camera_global, format, mut loader) in &mut cameras {
		if !camera.is_active { continue; }
		let camera_world = camera_global.translation();
		let settings = loader.settings.clone();
		let format = format.copied().unwrap_or_default();

		let mut acquire = Vec::new();
		let mut release = Vec::new();
		for (grid_id, grid_global, grid, streaming) in &mut grids {
			let camera_local = grid_global.affine().inverse().transform_point3(camera_world);
			let delta = update_desired_sources_delta(&mut loader, grid_id, nearest_chunk_center(camera_local), &settings, streaming.as_ref());
			let streaming = streaming.into_inner();
			loader.tiles.apply_delta(&delta.added, &delta.removed, &mut acquire, &mut release);
			release_sources(streaming, &requests, camera_entity, release.drain(..));

			for &key in &acquire {
				let center_local = ((key.min + key.size() / 2) * CHUNK_SIZE).as_vec3();
				let priority = -camera_world.distance(grid_global.transform_point(center_local));
				acquire_source(
					&mut loader.tiles, &requests, camera_entity, key, priority, grid, streaming, format, &subgrids, &gpu_state,
				);
			}
		}
	}
}

pub(crate) fn receive_camera_voxel_loader_results(
	requests: VoxelSourceRequests,
	mut loaders: Query<&mut CameraVoxelLoader>, mut consumers: Query<&mut CameraVoxelLoaderConsumer>, mut grids: Query<&mut GridStreaming>,
) {
	for mut consumer in &mut consumers {
		for result in consumer.drain_lod() {
			let Ok(mut loader) = loaders.get_mut(result.requester) else { continue };
			let key = TileKey { grid: result.grid, lod: result.key.lod, min: result.key.min };
			if !loader.tiles.contains_source(key) { continue; }
			let Ok(streaming) = grids.get_mut(result.grid) else { continue };
			let streaming = streaming.into_inner();
			let resolution = result.entity.map_or(ResolvedTile::Empty, ResolvedTile::Lod);
			release_sources(streaming, &requests, result.requester, loader.tiles.resolve(key, resolution));
		}
	}
}

pub(crate) fn refresh_camera_voxel_loader_visibility(
	requests: VoxelSourceRequests,
	mut availability_events: MessageReader<ChunkAvailabilityChanged>,
	mut chunk_events: MessageReader<ChunkLoadResolved>,
	mut upload_events: MessageReader<VoxelGpuUploadFinished>,
	mut cameras: Query<(Entity, Option<&VoxelGpuFormat>, &mut CameraVoxelLoader)>,
	mut grids: Query<(&Grid, &mut GridStreaming)>,
	subgrids: Query<&SubGrid>,
	gpu_state: Query<&VoxelGpuState>,
) {
	let availability_events: Vec<_> = availability_events.read().copied().collect();
	let chunk_events: Vec<_> = chunk_events.read().copied().collect();
	let upload_events: Vec<_> = upload_events.read().copied().collect();

	for (camera_entity, format, mut loader) in &mut cameras {
		let format = format.copied().unwrap_or_default();
		let mut changed = Vec::new();
		let mut acquire = Vec::new();
		let mut release = Vec::new();

		// Availability changes establish the desired source set before results resolve it.
		for event in &availability_events {
			match event.kind {
				ChunkAvailabilityChangeKind::BecamePresent => {
					let Some(bands) = loader.bands.get(&event.grid) else { continue };
					let Ok((grid, streaming)) = grids.get_mut(event.grid) else { continue };
					let streaming = streaming.into_inner();
					changed.clear();
					for_each_tile_in_bands(bands, event.min, event.size, |lod, min| {
						let key = TileKey { grid: event.grid, lod, min };
						if !loader.tiles.contains_desired(key) && tile_has_present_source(streaming, key) {
							changed.push(key);
						}
					});
					loader.tiles.apply_delta(&changed, &[], &mut acquire, &mut release);
					release_sources(streaming, &requests, camera_entity, release.drain(..));
					for &key in &acquire {
						acquire_source(
							&mut loader.tiles, &requests, camera_entity, key, 0.0, grid, streaming, format, &subgrids, &gpu_state,
						);
					}
				}
				ChunkAvailabilityChangeKind::BecameEmpty => {
					let Ok((_, streaming)) = grids.get_mut(event.grid) else { continue };
					let streaming = streaming.into_inner();
					loader.tiles.desired_in_area(event.grid, event.min, event.size, loader.settings.max_lod, &mut changed);
					changed.retain(|&key| !tile_has_present_source(streaming, key));
					loader.tiles.apply_delta(&[], &changed, &mut acquire, &mut release);
					release_sources(streaming, &requests, camera_entity, release.drain(..));
				}
			}
		}

		// Invisible chunk results resolve pending CPU coverage without renderer data.
		for event in &chunk_events {
			if event.visible { continue; }
			let key = TileKey::chunk(event.grid, event.chunk);
			if !loader.tiles.contains_source(key) { continue; }
			let Ok((_, streaming)) = grids.get_mut(event.grid) else { continue };
			release_sources(streaming.into_inner(), &requests, camera_entity, loader.tiles.resolve(key, ResolvedTile::Empty));
		}

		// Completed uploads replace the full chunk render snapshot atomically.
		for event in &upload_events {
			let Ok(subgrid) = subgrids.get(event.entity) else { continue };
			let Ok(state) = gpu_state.get(event.entity) else { continue };
			if !state.matches(format) { continue; }
			let Some(bounds) = state.bounds() else { continue };
			let min_chunk = (subgrid.sub_grid_pos() + bounds.min.as_ivec3()).div_euclid(IVec3::splat(CHUNK_SIZE));
			let max_chunk = (subgrid.sub_grid_pos() + bounds.max.as_ivec3()).div_euclid(IVec3::splat(CHUNK_SIZE));
			for x in min_chunk.x..=max_chunk.x {
				for y in min_chunk.y..=max_chunk.y {
					for z in min_chunk.z..=max_chunk.z {
						let key = TileKey::chunk(subgrid.grid(), IVec3::new(x, y, z));
						if !loader.tiles.contains_source(key) { continue; }
						let Ok((grid, streaming)) = grids.get_mut(key.grid) else { continue };
						let Some(resolution) = chunk_resolution(grid, key, format, &subgrids, &gpu_state) else { continue };
						release_sources(streaming.into_inner(), &requests, camera_entity, loader.tiles.resolve(key, resolution));
					}
				}
			}
		}
	}
}
