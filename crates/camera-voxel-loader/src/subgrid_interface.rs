use std::collections::HashSet;

use bevy::prelude::*;
use voxel_gpu::SubGridGpuState;
use voxel_data::{grid::{Grid, GridId}, subgrid::SubGrid};
use voxel_streaming::CHUNK_SIZE;

use crate::{camera_voxel_loader::CameraVoxelLoader, coverage::{resolve_empty, resolve_visible, retiring_visible_chunks}, types::TileKey};

pub(crate) fn chunks_in_bounds(grid: GridId, min: IVec3, max: IVec3) -> Vec<TileKey> {
	let min = min.div_euclid(IVec3::splat(CHUNK_SIZE));
	let max = max.div_euclid(IVec3::splat(CHUNK_SIZE));
	let mut chunks = Vec::new();
	for x in min.x..=max.x { for y in min.y..=max.y { for z in min.z..=max.z { chunks.push(TileKey::chunk(grid, IVec3::new(x, y, z))); } } }
	chunks
}

pub(crate) fn resolve_chunk_source_if_ready(
	loader: &mut CameraVoxelLoader, grids: &Query<&Grid>, subgrids: &Query<&SubGrid>, subgrid_gpu: &Query<&SubGridGpuState, With<SubGrid>>, chunk: TileKey,
) -> Vec<TileKey> {
	let Ok(grid) = grids.get(chunk.grid) else { return Vec::new() };
	let mut ready = Vec::new();
	let mut found = false;
	for entity in grid.subgrid_entities_in_area(chunk.min * CHUNK_SIZE, IVec3::splat(CHUNK_SIZE)) {
		if subgrids.get(entity).is_err() { continue; }
		found = true;
		if subgrid_gpu.get(entity).is_ok() { ready.extend(resolve_visible(loader, chunk, entity)); }
	}
	if !found { ready.extend(resolve_empty(loader, chunk)); }
	ready
}

pub(crate) fn collect_subgrids_to_render(loader: &CameraVoxelLoader, grids: &Query<&Grid>, subgrids: &Query<&SubGrid>) -> Vec<Entity> {
	let mut out = Vec::new();
	let mut seen = HashSet::new();
	for chunk in loader.desired_tiles.iter().copied().filter(|key| key.is_chunk()).chain(retiring_visible_chunks(loader)) {
		let Ok(grid) = grids.get(chunk.grid) else { continue };
		for entity in grid.subgrid_entities_in_area(chunk.min * CHUNK_SIZE, IVec3::splat(CHUNK_SIZE)) {
			if subgrids.get(entity).is_ok() && seen.insert(entity) { out.push(entity); }
		}
	}
	out
}
