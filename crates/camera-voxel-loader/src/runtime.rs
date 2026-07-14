use std::collections::HashSet;

use bevy::prelude::*;
use voxel_data::{grid::Grid, subgrid::SubGrid};
use voxel_gpu::{VoxelGpuFormat, VoxelGpuState};
use voxel_streaming::{CHUNK_SIZE, GridStreaming, LodKey, VoxelSourceRequests};

use crate::{tile_lifecycle::{ResolvedTile, TileLifecycle}, types::TileKey};

pub(crate) fn chunk_resolution(
	grid: &Grid,
	key: TileKey,
	format: VoxelGpuFormat,
	subgrids: &Query<&SubGrid>,
	gpu_state: &Query<&VoxelGpuState>,
) -> Option<ResolvedTile> {
	debug_assert!(key.is_chunk());
	let mut found_subgrid = false;
	let mut render_entities = HashSet::new();
	for entity in grid.subgrid_entities_in_area(key.min * CHUNK_SIZE, IVec3::splat(CHUNK_SIZE)) {
		if subgrids.get(entity).is_err() { continue; }
		found_subgrid = true;
		if gpu_state.get(entity).is_ok_and(|state| state.matches(format)) {
			render_entities.insert(entity);
		}
	}
	if !render_entities.is_empty() {
		Some(ResolvedTile::Chunk(render_entities))
	} else if !found_subgrid {
		Some(ResolvedTile::Empty)
	} else {
		None
	}
}

pub(crate) fn acquire_source(
	lifecycle: &mut TileLifecycle,
	requests: &VoxelSourceRequests,
	camera: Entity,
	key: TileKey,
	lod_priority: f32,
	grid: &Grid,
	streaming: &mut GridStreaming,
	format: VoxelGpuFormat,
	subgrids: &Query<&SubGrid>,
	gpu_state: &Query<&VoxelGpuState>,
) {
	if key.is_chunk() {
		streaming.fetch(key.grid, requests, key.min);
		if !matches!(streaming.state(key.min), Some(voxel_streaming::ChunkState::Loaded | voxel_streaming::ChunkState::InternalDirty)) {
			return;
		}
		if let Some(resolution) = chunk_resolution(grid, key, format, subgrids, gpu_state) {
			release_sources(streaming, camera, lifecycle.resolve(key, resolution));
		}
	} else if !streaming.fetch_lod(camera, LodKey { min: key.min, size: key.size(), lod: key.lod }, lod_priority) {
		release_sources(streaming, camera, lifecycle.resolve(key, ResolvedTile::Empty));
	}
}

pub(crate) fn release_sources(streaming: &mut GridStreaming, camera: Entity, sources: impl IntoIterator<Item = TileKey>) {
	for source in sources {
		if source.is_chunk() {
			streaming.release(source.min);
		} else {
			streaming.release_lod(camera, LodKey { min: source.min, size: source.size(), lod: source.lod });
		}
	}
}
