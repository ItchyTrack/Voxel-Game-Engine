use std::collections::HashSet;

use bevy::prelude::*;
use voxel_data::grid::GridId;
use voxel_streaming::{GridStreaming, CHUNK_SIZE};

use crate::camera_voxel_loader::{CameraVoxelLoader, CameraVoxelLoaderSettings};
use crate::types::{ChunkKey, TileKey};

pub(crate) struct SetDelta<T> {
	pub(crate) added: Vec<T>,
	pub(crate) removed: Vec<T>,
}

pub(crate) struct DesiredSourceDelta {
	pub(crate) chunks: SetDelta<ChunkKey>,
	pub(crate) tiles: SetDelta<TileKey>,
}

impl DesiredSourceDelta {
	fn empty() -> Self {
		Self {
			chunks: SetDelta { added: Vec::new(), removed: Vec::new() },
			tiles: SetDelta { added: Vec::new(), removed: Vec::new() },
		}
	}
}

pub(crate) fn nearest_chunk_center(local_voxels: Vec3) -> IVec3 {
	(local_voxels / CHUNK_SIZE as f32).round().as_ivec3()
}

#[allow(dead_code)]
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

#[allow(dead_code)]
pub(crate) fn add_lod_tiles(out: &mut HashSet<TileKey>, grid: GridId, center: IVec3, camera_voxel_loader: &CameraVoxelLoader, streaming: &GridStreaming) {
	add_lod_tiles_for_settings(out, grid, center, &camera_voxel_loader.settings, streaming);
}

fn add_lod_tiles_for_settings(out: &mut HashSet<TileKey>, grid: GridId, center: IVec3, settings: &CameraVoxelLoaderSettings, streaming: &GridStreaming) {
	if streaming.presence().len() == 0 { return; }

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

pub(crate) fn update_desired_sources_delta(
	desired_chunks: &mut HashSet<ChunkKey>,
	desired_tiles: &mut HashSet<TileKey>,
	grid: GridId,
	previous_center: Option<IVec3>,
	new_center: IVec3,
	settings_changed: bool,
	settings: &CameraVoxelLoaderSettings,
	streaming: &GridStreaming,
) -> DesiredSourceDelta {
	let mut delta = if settings_changed || previous_center.map_or(true, |old| should_rebuild_policy(old, new_center)) {
		rebuild_desired_sources_delta(desired_chunks, desired_tiles, grid, new_center, settings, streaming)
	} else if let Some(old_center) = previous_center.filter(|old| *old != new_center) {
		DesiredSourceDelta {
			chunks: update_near_chunks_delta(desired_chunks, grid, old_center, new_center, settings, streaming),
			tiles: update_lod_tiles_delta(desired_tiles, grid, old_center, new_center, settings, streaming),
		}
	} else {
		DesiredSourceDelta::empty()
	};

	let removed_absent: Vec<_> = desired_chunks
		.iter()
		.copied()
		.filter(|key| key.grid == grid && !streaming.presence().is_present(key.chunk))
		.collect();
	for key in removed_absent {
		if desired_chunks.remove(&key) {
			delta.chunks.removed.push(key);
		}
	}
	delta
}

fn rebuild_desired_sources_delta(
	desired_chunks: &mut HashSet<ChunkKey>,
	desired_tiles: &mut HashSet<TileKey>,
	grid: GridId,
	center: IVec3,
	settings: &CameraVoxelLoaderSettings,
	streaming: &GridStreaming,
) -> DesiredSourceDelta {
	let old_chunks: HashSet<_> = desired_chunks.iter().copied().filter(|key| key.grid == grid).collect();
	let mut new_chunks = HashSet::new();
	add_near_chunks_for_settings(&mut new_chunks, grid, center, settings, streaming);
	let old_tiles: HashSet<_> = desired_tiles.iter().copied().filter(|key| key.grid == grid).collect();
	let mut new_tiles = HashSet::new();
	add_lod_tiles_for_settings(&mut new_tiles, grid, center, settings, streaming);

	let mut delta = DesiredSourceDelta::empty();
	for key in old_chunks.difference(&new_chunks).copied() {
		desired_chunks.remove(&key);
		delta.chunks.removed.push(key);
	}
	for key in new_chunks.difference(&old_chunks).copied() {
		desired_chunks.insert(key);
		delta.chunks.added.push(key);
	}
	for key in old_tiles.difference(&new_tiles).copied() {
		desired_tiles.remove(&key);
		delta.tiles.removed.push(key);
	}
	for key in new_tiles.difference(&old_tiles).copied() {
		desired_tiles.insert(key);
		delta.tiles.added.push(key);
	}
	delta
}

pub(crate) fn update_near_chunks_delta(
	out: &mut HashSet<ChunkKey>,
	grid: GridId,
	old_center: IVec3,
	new_center: IVec3,
	settings: &CameraVoxelLoaderSettings,
	streaming: &GridStreaming,
) -> SetDelta<ChunkKey> {
	let mut delta = SetDelta { added: Vec::new(), removed: Vec::new() };
	let (old_min, old_max) = near_bounds(old_center, settings);
	let (new_min, new_max) = near_bounds(new_center, settings);
	for_each_changed_slab(old_min, old_max - IVec3::ONE, new_min, new_max - IVec3::ONE, |min, max, entering| {
		for x in min.x..=max.x {
			for y in min.y..=max.y {
				for z in min.z..=max.z {
					let chunk = IVec3::new(x, y, z);
					let key = ChunkKey { grid, chunk };
					if entering {
						if streaming.presence().is_present(chunk) && out.insert(key) {
							delta.added.push(key);
						}
					} else if out.remove(&key) {
						delta.removed.push(key);
					}
				}
			}
		}
	});
	delta
}

pub(crate) fn update_lod_tiles_delta(
	out: &mut HashSet<TileKey>,
	grid: GridId,
	old_center: IVec3,
	new_center: IVec3,
	settings: &CameraVoxelLoaderSettings,
	streaming: &GridStreaming,
) -> SetDelta<TileKey> {
	let mut delta = SetDelta { added: Vec::new(), removed: Vec::new() };
	for lod in 1..=settings.max_lod {
		let tile_size = 1i32 << lod;
		let mut candidates = HashSet::new();
		let (old_inner_min, old_inner_max) = lod_inner_bounds(old_center, settings, lod);
		let (old_outer_min, old_outer_max) = lod_outer_bounds(old_center, settings, lod);
		let (new_inner_min, new_inner_max) = lod_inner_bounds(new_center, settings, lod);
		let (new_outer_min, new_outer_max) = lod_outer_bounds(new_center, settings, lod);
		add_changed_tile_slabs(&mut candidates, grid, lod, old_outer_min, old_outer_max - IVec3::ONE, new_outer_min, new_outer_max - IVec3::ONE, tile_size);
		add_changed_tile_slabs(&mut candidates, grid, lod, old_inner_min, old_inner_max - IVec3::ONE, new_inner_min, new_inner_max - IVec3::ONE, tile_size);

		for key in candidates {
			let old_wanted = wants_lod_tile(settings, streaming, old_center, key);
			let new_wanted = wants_lod_tile(settings, streaming, new_center, key);
			match (old_wanted, new_wanted) {
				(false, true) => {
					if out.insert(key) {
						delta.added.push(key);
					}
				}
				(true, false) => {
					if out.remove(&key) {
						delta.removed.push(key);
					}
				}
				_ => {}
			}
		}
	}
	delta
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
	out: &mut HashSet<TileKey>,
	grid: GridId,
	lod: u8,
	old_min: IVec3,
	old_max: IVec3,
	new_min: IVec3,
	new_max: IVec3,
	tile_size: i32,
) {
	for_each_changed_slab(old_min, old_max, new_min, new_max, |min, max, _entering| {
		add_tiles_intersecting_box(out, grid, lod, min, max, tile_size);
	});
}

fn add_tiles_intersecting_box(out: &mut HashSet<TileKey>, grid: GridId, lod: u8, min: IVec3, max: IVec3, tile_size: i32) {
	let min_tile = align_chunk_to_tile(min - IVec3::splat(tile_size - 1), tile_size);
	let max_tile = align_chunk_to_tile(max, tile_size);
	for x in (min_tile.x..=max_tile.x).step_by(tile_size as usize) {
		for y in (min_tile.y..=max_tile.y).step_by(tile_size as usize) {
			for z in (min_tile.z..=max_tile.z).step_by(tile_size as usize) {
				out.insert(TileKey { grid, lod, min: IVec3::new(x, y, z) });
			}
		}
	}
}

fn should_rebuild_policy(old_center: IVec3, new_center: IVec3) -> bool {
	const MAX_INCREMENTAL_DELTA_CHUNKS: i32 = 8;
	(new_center - old_center).abs().max_element() > MAX_INCREMENTAL_DELTA_CHUNKS
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
