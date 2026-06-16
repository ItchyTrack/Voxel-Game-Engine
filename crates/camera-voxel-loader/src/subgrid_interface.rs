use std::collections::HashSet;

use bevy::prelude::*;
use gpu_voxel_data::SubGridGpuState;
use voxel_data::grid::Grid;
use voxel_data::subgrid::SubGrid;
use voxel_streaming::CHUNK_SIZE;

use crate::camera_voxel_loader::CameraVoxelLoader;
use crate::coverage::{resolve_empty, resolve_visible, retiring_visible_chunks, CoverageSource};
use crate::types::TileKey;

pub(crate) fn chunks_in_bounds(grid: Entity, min: IVec3, max: IVec3) -> Vec<TileKey> {
	let min = min.div_euclid(IVec3::splat(CHUNK_SIZE));
	let max = max.div_euclid(IVec3::splat(CHUNK_SIZE));
	let mut chunks = Vec::new();
	for x in min.x..=max.x {
		for y in min.y..=max.y {
			for z in min.z..=max.z {
				chunks.push(TileKey::chunk(grid, IVec3::new(x, y, z)));
			}
		}
	}
	chunks
}

pub(crate) fn resolve_chunk_source_if_ready(
	camera_voxel_loader: &mut CameraVoxelLoader, grids: &Query<&Grid>, subgrids: &Query<&SubGrid>,
	subgrid_gpu: &Query<&SubGridGpuState, With<SubGrid>>, chunk: TileKey,
) -> Vec<CoverageSource> {
	debug_assert!(chunk.is_chunk());
	let Ok(grid) = grids.get(chunk.grid) else { return Vec::new() };
	let mut ready = Vec::new();
	let min = chunk.min * CHUNK_SIZE;
	let mut found_subgrid = false;
	for entity in grid.subgrid_entities_in_area(min, IVec3::splat(CHUNK_SIZE)) {
		if subgrids.get(entity).is_err() {
			continue;
		}
		found_subgrid = true;
		if subgrid_gpu.get(entity).is_ok() {
			ready.extend(resolve_visible(camera_voxel_loader, chunk, entity));
		}
	}
	if !found_subgrid {
		ready.extend(resolve_empty(camera_voxel_loader, chunk));
	}
	ready
}

pub(crate) fn collect_subgrids_to_render(camera_voxel_loader: &CameraVoxelLoader, grids: &Query<&Grid>, subgrids: &Query<&SubGrid>) -> Vec<Entity> {
	let mut subgrids_to_render = Vec::new();
	let mut seen_subgrids = HashSet::new();
	let chunks = camera_voxel_loader.desired_tiles.iter().copied().filter(|key| key.is_chunk()).chain(retiring_visible_chunks(camera_voxel_loader));
	for chunk in chunks {
		let Ok(grid) = grids.get(chunk.grid) else { continue };
		let min = chunk.min * CHUNK_SIZE;
		for entity in grid.subgrid_entities_in_area(min, IVec3::splat(CHUNK_SIZE)) {
			if subgrids.get(entity).is_ok() && seen_subgrids.insert(entity) {
				subgrids_to_render.push(entity);
			}
		}
	}
	subgrids_to_render
}
