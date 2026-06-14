use std::collections::HashSet;

use bevy::prelude::*;
use voxel_data::grid::GridId;
use voxel_streaming::{GridStreaming, CHUNK_SIZE};

use crate::camera_voxel_loader::{CameraVoxelLoader, CameraVoxelLoaderSettings};
use crate::types::{ChunkKey, PolicyDebugBox, PolicyDebugBoxKind, TileKey};

pub(crate) fn nearest_chunk_center(local_voxels: Vec3) -> IVec3 {
	(local_voxels / CHUNK_SIZE as f32).round().as_ivec3()
}

pub(crate) fn add_near_chunks(out: &mut HashSet<ChunkKey>, grid: GridId, center: IVec3, camera_voxel_loader: &CameraVoxelLoader, streaming: &GridStreaming) {
	add_near_chunks_for_settings(out, grid, center, &camera_voxel_loader.settings, streaming);
}

fn add_near_chunks_for_settings(out: &mut HashSet<ChunkKey>, grid: GridId, center: IVec3, settings: &CameraVoxelLoaderSettings, streaming: &GridStreaming) {
	let (min, max) = near_bounds(center, settings);
	for x in min.x..max.x {
		for y in min.y..max.y {
			for z in min.z..max.z {
				let chunk = IVec3::new(x, y, z);
				if streaming.presence().is_present(chunk) {
					out.insert(ChunkKey { grid, chunk });
				}
			}
		}
	}
}

pub(crate) fn add_lod_tiles(out: &mut HashSet<TileKey>, grid: GridId, center: IVec3, camera_voxel_loader: &CameraVoxelLoader, streaming: &GridStreaming) {
	add_lod_tiles_for_settings(out, grid, center, &camera_voxel_loader.settings, streaming);
}

fn add_lod_tiles_for_settings(out: &mut HashSet<TileKey>, grid: GridId, center: IVec3, settings: &CameraVoxelLoaderSettings, streaming: &GridStreaming) {
	if presence_bounds(streaming).is_none() { return; }

	for lod in 1..=settings.max_lod {
		let tile_size = 1i32 << lod;
		let (inner_min, inner_max) = lod_inner_bounds(center, settings, lod);
		let (outer_min, outer_max) = lod_outer_bounds(center, settings, lod);
		for x in (outer_min.x..outer_max.x).step_by(tile_size as usize) {
			for y in (outer_min.y..outer_max.y).step_by(tile_size as usize) {
				for z in (outer_min.z..outer_max.z).step_by(tile_size as usize) {
					let min = IVec3::new(x, y, z);
					let max = min + IVec3::splat(tile_size);
					let inside_inner = min.cmpge(inner_min).all() && max.cmple(inner_max).all();
					if !inside_inner && tile_has_present_chunk(streaming, min, tile_size) {
						out.insert(TileKey { grid, lod, min });
					}
				}
			}
		}
	}
}

fn presence_bounds(streaming: &GridStreaming) -> Option<(IVec3, IVec3)> {
	let mut min = IVec3::splat(i32::MAX);
	let mut max = IVec3::splat(i32::MIN);
	let mut any = false;
	for (origin, size) in streaming.presence().iter_present() {
		any = true;
		min = min.min(origin);
		max = max.max(origin + IVec3::splat(size as i32) - IVec3::ONE);
	}
	any.then_some((min, max))
}

fn tile_has_present_chunk(streaming: &GridStreaming, min: IVec3, tile_size: i32) -> bool {
	let max = min + IVec3::splat(tile_size) - IVec3::ONE;
	streaming.presence().any_present_in_region(min, max)
}

pub(crate) fn align_chunk_to_tile(chunk: IVec3, tile_size: i32) -> IVec3 {
	chunk.div_euclid(IVec3::splat(tile_size)) * tile_size
}

pub(crate) fn tile_key_covering_chunk(grid: GridId, chunk: IVec3, lod: u8) -> TileKey {
	let tile_size = 1i32 << lod;
	TileKey { grid, lod, min: align_chunk_to_tile(chunk, tile_size) }
}

pub(crate) fn is_near_chunk_wanted(settings: &CameraVoxelLoaderSettings, streaming: &GridStreaming, center: IVec3, chunk: IVec3) -> bool {
	let (min, max) = near_bounds(center, settings);
	chunk.cmpge(min).all() && chunk.cmplt(max).all() && streaming.presence().is_present(chunk)
}

pub(crate) fn update_near_chunks_delta(
	out: &mut HashSet<ChunkKey>,
	debug_boxes: &mut Vec<PolicyDebugBox>,
	grid: GridId,
	old_center: IVec3,
	new_center: IVec3,
	settings: &CameraVoxelLoaderSettings,
	streaming: &GridStreaming,
) {
	let (old_min, old_max) = near_bounds(old_center, settings);
	let (new_min, new_max) = near_bounds(new_center, settings);
	for_each_changed_slab(old_min, old_max - IVec3::ONE, new_min, new_max - IVec3::ONE, |min, max, entering| {
		push_policy_debug_box(debug_boxes, grid, min, max, entering, PolicyDebugBoxKind::NearChunks);
	});
	out.retain(|key| key.grid != grid);
	add_near_chunks_for_settings(out, grid, new_center, settings, streaming);
}

pub(crate) fn update_lod_tiles_delta(
	out: &mut HashSet<TileKey>,
	debug_boxes: &mut Vec<PolicyDebugBox>,
	grid: GridId,
	old_center: IVec3,
	new_center: IVec3,
	settings: &CameraVoxelLoaderSettings,
	streaming: &GridStreaming,
) {
	for lod in 1..=settings.max_lod {
		let tile_size = 1i32 << lod;
		let (old_inner_min, old_inner_max) = lod_inner_bounds(old_center, settings, lod);
		let (old_outer_min, old_outer_max) = lod_outer_bounds(old_center, settings, lod);
		let (new_inner_min, new_inner_max) = lod_inner_bounds(new_center, settings, lod);
		let (new_outer_min, new_outer_max) = lod_outer_bounds(new_center, settings, lod);
		add_changed_tile_slabs(debug_boxes, grid, lod, old_outer_min, old_outer_max - IVec3::ONE, new_outer_min, new_outer_max - IVec3::ONE, tile_size, PolicyDebugBoxKind::LodOuter(lod));
		add_changed_tile_slabs(debug_boxes, grid, lod, old_inner_min, old_inner_max - IVec3::ONE, new_inner_min, new_inner_max - IVec3::ONE, tile_size, PolicyDebugBoxKind::LodInner(lod));
	}
	out.retain(|key| key.grid != grid);
	add_lod_tiles_for_settings(out, grid, new_center, settings, streaming);
}

pub(crate) fn is_lod_tile_wanted(settings: &CameraVoxelLoaderSettings, streaming: &GridStreaming, center: IVec3, key: TileKey) -> bool {
	wants_lod_tile(settings, streaming, center, key)
}

fn wants_lod_tile(settings: &CameraVoxelLoaderSettings, streaming: &GridStreaming, center: IVec3, key: TileKey) -> bool {
	let tile_size = 1i32 << key.lod;
	let (inner_min, inner_max) = lod_inner_bounds(center, settings, key.lod);
	let (outer_min, outer_max) = lod_outer_bounds(center, settings, key.lod);
	let max = key.min + IVec3::splat(tile_size);
	key.min.cmpge(outer_min).all()
		&& max.cmple(outer_max).all()
		&& !(key.min.cmpge(inner_min).all() && max.cmple(inner_max).all())
		&& tile_has_present_chunk(streaming, key.min, tile_size)
}

fn near_bounds(center: IVec3, settings: &CameraVoxelLoaderSettings) -> (IVec3, IVec3) {
	let min = center - IVec3::splat(settings.near_radius_chunks);
	let max = center + IVec3::splat(settings.near_radius_chunks + 1);
	align_bounds_to_tile(min, max, 2)
}

fn lod_inner_bounds(center: IVec3, settings: &CameraVoxelLoaderSettings, lod: u8) -> (IVec3, IVec3) {
	if lod == 1 {
		near_bounds(center, settings)
	} else {
		lod_outer_bounds(center, settings, lod - 1)
	}
}

fn lod_outer_bounds(center: IVec3, settings: &CameraVoxelLoaderSettings, lod: u8) -> (IVec3, IVec3) {
	let bounds = expand_bounds_by_tile_rings(lod_inner_bounds(center, settings, lod), 1i32 << lod, settings.rings_per_lod);
	if lod < settings.max_lod {
		align_bounds_to_tile(bounds.0, bounds.1, 1i32 << (lod + 1))
	} else {
		bounds
	}
}

fn expand_bounds_by_tile_rings((min, max): (IVec3, IVec3), tile_size: i32, rings: i32) -> (IVec3, IVec3) {
	let expansion = IVec3::splat(tile_size * rings);
	(min - expansion, max + expansion)
}

fn align_bounds_to_tile(min: IVec3, max: IVec3, tile_size: i32) -> (IVec3, IVec3) {
	let tile = IVec3::splat(tile_size);
	(min.div_euclid(tile) * tile, (max + tile - IVec3::ONE).div_euclid(tile) * tile)
}

fn add_changed_tile_slabs(
	debug_boxes: &mut Vec<PolicyDebugBox>,
	grid: GridId,
	_lod: u8,
	old_min: IVec3,
	old_max: IVec3,
	new_min: IVec3,
	new_max: IVec3,
	_tile_size: i32,
	kind: PolicyDebugBoxKind,
) {
	for_each_changed_slab(old_min, old_max, new_min, new_max, |min, max, entering| {
		push_policy_debug_box(debug_boxes, grid, min, max, entering, kind);
	});
}

fn push_policy_debug_box(debug_boxes: &mut Vec<PolicyDebugBox>, grid: GridId, min: IVec3, max: IVec3, entering: bool, kind: PolicyDebugBoxKind) {
	debug_boxes.push(PolicyDebugBox { grid, min, max: max + IVec3::ONE, entering, kind });
}

fn for_each_changed_slab(old_min: IVec3, old_max: IVec3, new_min: IVec3, new_max: IVec3, mut f: impl FnMut(IVec3, IVec3, bool)) {
	for axis in 0..3 {
		if new_min[axis] < old_min[axis] {
			let min = new_min;
			let mut max = new_max;
			max[axis] = old_min[axis] - 1;
			f(min, max, true);
		} else if new_min[axis] > old_min[axis] {
			let min = old_min;
			let mut max = old_max;
			max[axis] = new_min[axis] - 1;
			f(min, max, false);
		}

		if new_max[axis] > old_max[axis] {
			let mut min = new_min;
			let max = new_max;
			min[axis] = old_max[axis] + 1;
			f(min, max, true);
		} else if new_max[axis] < old_max[axis] {
			let mut min = old_min;
			let max = old_max;
			min[axis] = new_max[axis] + 1;
			f(min, max, false);
		}
	}
}

#[cfg(test)]
fn lod_outer_radius_chunks(camera_voxel_loader: &CameraVoxelLoader) -> i32 {
	let mut inner = camera_voxel_loader.settings.near_radius_chunks + 1;
	let mut outer = inner;
	for lod in 1..=camera_voxel_loader.settings.max_lod {
		outer = inner + camera_voxel_loader.settings.rings_per_lod * (1i32 << lod);
		inner = outer + 1;
	}
	outer
}
