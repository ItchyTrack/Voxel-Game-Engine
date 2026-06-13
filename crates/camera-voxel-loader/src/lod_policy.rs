use std::collections::HashSet;

use bevy::prelude::*;
use voxel_data::grid::GridId;
use voxel_streaming::{GridStreaming, CHUNK_SIZE};

use crate::camera_voxel_loader::CameraVoxelLoader;
use crate::types::{ChunkKey, TileKey};

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
	let mut any = false;
	streaming.presence().for_each_in_region(min, max, |_| {
		any = true;
	});
	any
}

pub(crate) fn align_chunk_to_tile(chunk: IVec3, tile_size: i32) -> IVec3 {
	chunk.div_euclid(IVec3::splat(tile_size)) * tile_size
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

