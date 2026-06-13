use std::collections::HashSet;

use bevy::prelude::*;
use voxel_data::grid::GridId;
use voxel_streaming::{GridStreaming, CHUNK_SIZE};

use crate::camera_voxel_loader::{CameraVoxelLoader, CameraVoxelLoaderSettings};
use crate::types::{ChunkKey, PolicyDebugBox, PolicyDebugBoxKind, TileKey};

pub(crate) fn nearest_chunk_center(local_voxels: Vec3) -> IVec3 {
	(local_voxels / CHUNK_SIZE as f32).round().as_ivec3()
}

pub(crate) fn add_near_chunks(out: &mut HashSet<ChunkKey>, grid: GridId, center: IVec3, camera_voxel_loader: &CameraVoxelLoader) {
	let r = camera_voxel_loader.settings.near_radius_chunks;
	for x in -r..=r {
		for y in -r..=r {
			for z in -r..=r {
				out.insert(ChunkKey { grid, chunk: center + IVec3::new(x, y, z) });
			}
		}
	}
}

pub(crate) fn add_lod_tiles(out: &mut HashSet<TileKey>, grid: GridId, center: IVec3, camera_voxel_loader: &CameraVoxelLoader, streaming: &GridStreaming) {
	if presence_bounds(streaming).is_none() { return; }

	let mut inner = camera_voxel_loader.settings.near_radius_chunks + 1;
	for lod in 1..=camera_voxel_loader.settings.max_lod {
		let tile_size = 1i32 << lod;
		let outer = inner + camera_voxel_loader.settings.rings_per_lod * tile_size;
		let lod_min = center - IVec3::splat(outer);
		let lod_max = center + IVec3::splat(outer);
		let min_tile = align_chunk_to_tile(lod_min, tile_size);
		let max_tile = align_chunk_to_tile(lod_max, tile_size);
		let band_inner = if lod == 1 { 0 } else { inner };
		for x in (min_tile.x..=max_tile.x).step_by(tile_size as usize) {
			for y in (min_tile.y..=max_tile.y).step_by(tile_size as usize) {
				for z in (min_tile.z..=max_tile.z).step_by(tile_size as usize) {
					let min = IVec3::new(x, y, z);
					if tile_intersects_ring(center, min, tile_size, band_inner, outer)
						&& !tile_inside_near_box(center, min, tile_size, camera_voxel_loader.settings.near_radius_chunks)
						&& tile_has_present_chunk(streaming, min, tile_size)
					{
						out.insert(TileKey { grid, lod, min });
					}
				}
			}
		}
		inner = outer + 1;
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

pub(crate) fn update_near_chunks_delta(
	out: &mut HashSet<ChunkKey>,
	debug_boxes: &mut Vec<PolicyDebugBox>,
	grid: GridId,
	old_center: IVec3,
	new_center: IVec3,
	settings: &CameraVoxelLoaderSettings,
) {
	let r = settings.near_radius_chunks;
	let old_min = old_center - IVec3::splat(r);
	let old_max = old_center + IVec3::splat(r);
	let new_min = new_center - IVec3::splat(r);
	let new_max = new_center + IVec3::splat(r);

	for_each_changed_slab(old_min, old_max, new_min, new_max, |min, max, entering| {
		push_policy_debug_box(debug_boxes, grid, min, max, entering, PolicyDebugBoxKind::NearChunks);
		for x in min.x..=max.x {
			for y in min.y..=max.y {
				for z in min.z..=max.z {
					let key = ChunkKey { grid, chunk: IVec3::new(x, y, z) };
					if entering {
						out.insert(key);
					} else {
						out.remove(&key);
					}
				}
			}
		}
	});
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
		let mut candidates = HashSet::new();
		let (old_inner, old_outer, old_band_inner) = lod_bounds(settings, lod, old_center);
		let (new_inner, new_outer, new_band_inner) = lod_bounds(settings, lod, new_center);

		add_changed_tile_slabs(
			&mut candidates,
			debug_boxes,
			grid,
			lod,
			old_outer.0,
			old_outer.1,
			new_outer.0,
			new_outer.1,
			tile_size,
			PolicyDebugBoxKind::LodOuter(lod),
		);
		add_changed_tile_slabs(
			&mut candidates,
			debug_boxes,
			grid,
			lod,
			old_inner.0,
			old_inner.1,
			new_inner.0,
			new_inner.1,
			tile_size,
			PolicyDebugBoxKind::LodInner(lod),
		);
		add_tile_boundary_surfaces(
			&mut candidates,
			debug_boxes,
			grid,
			lod,
			old_center,
			old_outer.2,
			old_outer.2,
			tile_size,
			false,
			PolicyDebugBoxKind::LodOuter(lod),
		);
		add_tile_boundary_surfaces(
			&mut candidates,
			debug_boxes,
			grid,
			lod,
			new_center,
			new_outer.2,
			new_outer.2,
			tile_size,
			true,
			PolicyDebugBoxKind::LodOuter(lod),
		);
		if old_band_inner > 0 {
			add_tile_boundary_surfaces(
				&mut candidates,
				debug_boxes,
				grid,
				lod,
				old_center,
				old_band_inner,
				old_outer.2,
				tile_size,
				false,
				PolicyDebugBoxKind::LodInner(lod),
			);
		}
		if new_band_inner > 0 {
			add_tile_boundary_surfaces(
				&mut candidates,
				debug_boxes,
				grid,
				lod,
				new_center,
				new_band_inner,
				new_outer.2,
				tile_size,
				true,
				PolicyDebugBoxKind::LodInner(lod),
			);
		}

		let near = settings.near_radius_chunks;
		add_changed_tile_slabs(
			&mut candidates,
			debug_boxes,
			grid,
			lod,
			old_center - IVec3::splat(near),
			old_center + IVec3::splat(near),
			new_center - IVec3::splat(near),
			new_center + IVec3::splat(near),
			tile_size,
			PolicyDebugBoxKind::LodNearExclusion(lod),
		);
		add_tile_boundary_surfaces(
			&mut candidates,
			debug_boxes,
			grid,
			lod,
			old_center,
			near,
			near,
			tile_size,
			false,
			PolicyDebugBoxKind::LodNearExclusion(lod),
		);
		add_tile_boundary_surfaces(
			&mut candidates,
			debug_boxes,
			grid,
			lod,
			new_center,
			near,
			near,
			tile_size,
			true,
			PolicyDebugBoxKind::LodNearExclusion(lod),
		);

		for key in candidates {
			let old_wanted = wants_lod_tile(settings, streaming, old_center, key, old_band_inner, old_outer.2);
			let new_wanted = wants_lod_tile(settings, streaming, new_center, key, new_band_inner, new_outer.2);
			match (old_wanted, new_wanted) {
				(false, true) => {
					out.insert(key);
				}
				(true, false) => {
					out.remove(&key);
				}
				_ => {}
			}
		}
	}
}

fn wants_lod_tile(settings: &CameraVoxelLoaderSettings, streaming: &GridStreaming, center: IVec3, key: TileKey, band_inner: i32, outer: i32) -> bool {
	let tile_size = 1i32 << key.lod;
	tile_intersects_ring(center, key.min, tile_size, band_inner, outer)
		&& !tile_inside_near_box(center, key.min, tile_size, settings.near_radius_chunks)
		&& tile_has_present_chunk(streaming, key.min, tile_size)
}

fn lod_bounds(settings: &CameraVoxelLoaderSettings, lod: u8, center: IVec3) -> ((IVec3, IVec3, i32), (IVec3, IVec3, i32), i32) {
	let mut inner = settings.near_radius_chunks + 1;
	for current_lod in 1..=lod {
		let outer = inner + settings.rings_per_lod * (1i32 << current_lod);
		let band_inner = if current_lod == 1 { 0 } else { inner };
		if current_lod == lod {
			return (
				(center - IVec3::splat(band_inner), center + IVec3::splat(band_inner), band_inner),
				(center - IVec3::splat(outer), center + IVec3::splat(outer), outer),
				band_inner,
			);
		}
		inner = outer + 1;
	}
	unreachable!()
}

fn add_changed_tile_slabs(
	out: &mut HashSet<TileKey>,
	debug_boxes: &mut Vec<PolicyDebugBox>,
	grid: GridId,
	lod: u8,
	old_min: IVec3,
	old_max: IVec3,
	new_min: IVec3,
	new_max: IVec3,
	tile_size: i32,
	kind: PolicyDebugBoxKind,
) {
	for_each_changed_slab(old_min, old_max, new_min, new_max, |min, max, entering| {
		push_policy_debug_box(debug_boxes, grid, min, max, entering, kind);
		add_tiles_intersecting_box(out, grid, lod, min, max, tile_size);
	});
}

fn push_policy_debug_box(debug_boxes: &mut Vec<PolicyDebugBox>, grid: GridId, min: IVec3, max: IVec3, entering: bool, kind: PolicyDebugBoxKind) {
	debug_boxes.push(PolicyDebugBox { grid, min, max: max + IVec3::ONE, entering, kind });
}

fn add_tile_boundary_surfaces(
	out: &mut HashSet<TileKey>,
	debug_boxes: &mut Vec<PolicyDebugBox>,
	grid: GridId,
	lod: u8,
	center: IVec3,
	radius: i32,
	extent: i32,
	tile_size: i32,
	entering: bool,
	kind: PolicyDebugBoxKind,
) {
	for axis in 0..3 {
		for side in [-1, 1] {
			let mut min = center - IVec3::splat(extent);
			let mut max = center + IVec3::splat(extent);
			min[axis] = center[axis] + side * radius;
			max[axis] = min[axis];
			push_policy_debug_box(debug_boxes, grid, min, max, entering, kind);
			add_tiles_intersecting_box(out, grid, lod, min, max, tile_size);
		}
	}
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

fn axis_min_abs_delta(center: i32, lo: i32, hi: i32) -> i32 {
	if center < lo {
		lo - center
	} else if center > hi {
		center - hi
	} else {
		0
	}
}

fn tile_intersects_ring(center: IVec3, min: IVec3, tile_size: i32, inner: i32, outer: i32) -> bool {
	let max = min + IVec3::splat(tile_size - 1);
	let min_dist = axis_min_abs_delta(center.x, min.x, max.x)
		.max(axis_min_abs_delta(center.y, min.y, max.y))
		.max(axis_min_abs_delta(center.z, min.z, max.z));
	let max_dist = (min.x - center.x)
		.abs()
		.max((max.x - center.x).abs())
		.max((min.y - center.y).abs())
		.max((max.y - center.y).abs())
		.max((min.z - center.z).abs())
		.max((max.z - center.z).abs());
	max_dist >= inner && min_dist <= outer
}

fn tile_inside_near_box(center: IVec3, min: IVec3, tile_size: i32, near_radius: i32) -> bool {
	let max = min + IVec3::splat(tile_size - 1);
	(min.x - center.x).abs().max((max.x - center.x).abs()) <= near_radius
		&& (min.y - center.y).abs().max((max.y - center.y).abs()) <= near_radius
		&& (min.z - center.z).abs().max((max.z - center.z).abs()) <= near_radius
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
