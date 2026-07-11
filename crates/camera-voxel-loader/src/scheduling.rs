use bevy::ecs::message::MessageReader;
use bevy::prelude::*;
use voxel_gpu::{VoxelGpuFormat, VoxelGpuState, VoxelGpuUploadFinished};
use voxel_data::grid::{Grid, GridId};
use voxel_data::subgrid::SubGrid;
use voxel_streaming::{ChunkAvailabilityChangeKind, ChunkAvailabilityChanged, ChunkLoadResolved, GridStreaming, VoxelSourceRequests, CHUNK_SIZE};

use crate::camera_voxel_loader::CameraVoxelLoader;
use crate::lod_bands::for_each_tile_in_bands;
use crate::loading::{resolve_and_retire, unwant_source, want_source};
use crate::lod_policy::{nearest_chunk_center, tile_has_present_source, update_desired_sources_delta};
use crate::subgrid_interface::{chunks_in_bounds, insert_chunk_render_entity};
use crate::types::{SourceResolution, TileKey};

pub(crate) fn update_camera_voxel_loader_requests(
	requests: VoxelSourceRequests,
	mut cameras: Query<(Entity, &Camera, &GlobalTransform, Option<&VoxelGpuFormat>, &mut CameraVoxelLoader), With<Camera3d>>,
	mut grids: Query<(GridId, &GlobalTransform, &Grid, &mut GridStreaming)>,
	subgrids: Query<&SubGrid>, gpu_state: Query<&VoxelGpuState>,
) {
	for (camera_entity, camera, camera_global, format, mut camera_voxel_loader) in &mut cameras {
		if !camera.is_active {
			continue;
		}
		let camera_world = camera_global.translation();
		let settings = camera_voxel_loader.settings.clone();
		let format = format.copied().unwrap_or_default();

		for (grid, grid_global, grid_component, streaming) in &mut grids {
			let local = grid_global.affine().inverse().transform_point3(camera_world);
			let camera_chunk = nearest_chunk_center(local);
			let source_delta = update_desired_sources_delta(&mut camera_voxel_loader, grid, camera_chunk, &settings, streaming.as_ref());
			let streaming = streaming.into_inner();
			for key in source_delta.added {
				let priority = tile_priority(camera_world, key, grid_global);
				want_source(&mut camera_voxel_loader, format, streaming, &requests, grid_component, &subgrids, &gpu_state, camera_entity, key, priority);
			}
			for key in source_delta.removed {
				unwant_source(&mut camera_voxel_loader, streaming, camera_entity, key);
			}
		}
	}
}

pub(crate) fn refresh_camera_voxel_loader_visibility(
	requests: VoxelSourceRequests,
	mut upload_events: MessageReader<VoxelGpuUploadFinished>,
	mut chunk_events: MessageReader<ChunkLoadResolved>,
	mut availability_events: MessageReader<ChunkAvailabilityChanged>,
	mut cameras: Query<(Entity, Option<&VoxelGpuFormat>, &mut CameraVoxelLoader)>,
	mut grids: Query<(&Grid, &mut GridStreaming)>,
	subgrids: Query<&SubGrid>,
	gpu_state: Query<&VoxelGpuState>,
) {
	let upload_events: Vec<_> = upload_events.read().copied().collect();
	let chunk_events: Vec<_> = chunk_events.read().copied().collect();
	let availability_events: Vec<_> = availability_events.read().copied().collect();
	for (camera_entity, format, mut camera_voxel_loader) in &mut cameras {
		let format = format.copied().unwrap_or_default();
		for event in &availability_events {
			match event.kind {
				ChunkAvailabilityChangeKind::BecamePresent => {
					let Some(bands) = camera_voxel_loader.bands.get(&event.grid).cloned() else { continue };
					for_each_tile_in_bands(&bands, event.min, event.size, |lod, min| {
						let key = TileKey { grid: event.grid, lod, min };
						let Ok((grid, streaming)) = grids.get_mut(key.grid) else { return; };
						let streaming = streaming.into_inner();
						if !tile_has_present_source(streaming, key) { return; }
						if !camera_voxel_loader.insert_desired_tile(key) { return; }
						want_source(&mut camera_voxel_loader, format, streaming, &requests, grid, &subgrids, &gpu_state, camera_entity, key, 0.0);
					});
				}
				ChunkAvailabilityChangeKind::BecameEmpty => {
					let affected = camera_voxel_loader.desired_tiles_in_area(event.grid, event.min, event.size);
					for key in affected {
						let Ok((_, streaming)) = grids.get_mut(key.grid) else { continue; };
						let streaming = streaming.into_inner();
						if tile_has_present_source(streaming, key) { continue; }
						camera_voxel_loader.remove_desired_tile(key);
						unwant_source(&mut camera_voxel_loader, streaming, camera_entity, key);
					}
				}
			}
		}
		for event in &chunk_events {
			let source = TileKey::chunk(event.grid, event.chunk);
			if !event.visible {
				if let Ok((_, streaming)) = grids.get_mut(event.grid) {
					resolve_and_retire(&mut camera_voxel_loader, streaming.into_inner(), camera_entity, source, SourceResolution::Empty);
				}
			}
		}
		for event in &upload_events {
			let Ok(subgrid) = subgrids.get(event.entity) else { continue; };
			let Ok(state) = gpu_state.get(event.entity) else { continue; };
			if !state.matches(format) { continue; }
			let Some(bounds) = state.bounds() else { continue; };
			for chunk in chunks_in_bounds(
				subgrid.grid(),
				subgrid.sub_grid_pos() + bounds.min.as_ivec3(),
				subgrid.sub_grid_pos() + bounds.max.as_ivec3(),
			) {
				if camera_voxel_loader.desired_tiles.contains(&chunk) {
					insert_chunk_render_entity(&mut camera_voxel_loader, chunk, event.entity);
					if let Ok((_, streaming)) = grids.get_mut(chunk.grid) {
						resolve_and_retire(&mut camera_voxel_loader, streaming.into_inner(), camera_entity, chunk, SourceResolution::Visible(event.entity));
					}
				}
			}
		}

	}
}

fn tile_priority(camera_world: Vec3, key: TileKey, grid_global: &GlobalTransform) -> f32 {
	let center_local = ((key.min + key.size() / 2) * CHUNK_SIZE).as_vec3();
	let center_world = grid_global.transform_point(center_local);
	-camera_world.distance(center_world)
}
