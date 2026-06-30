use bevy::prelude::*;
use tracy_client::span;
use voxel_data::grid::GridId;
use voxel_streaming::{GridStreaming, CHUNK_SIZE};

use crate::camera_voxel_loader::{CameraVoxelLoader, CameraVoxelLoaderSettings};
use crate::lod_bands::{run_over_diff, BandBounds, LodBand};
use crate::types::TileKey;

pub(crate) struct SetDelta<T> {
	pub(crate) added: Vec<T>,
	pub(crate) removed: Vec<T>,
}

pub(crate) struct DesiredSourceDelta {
	pub(crate) tiles: SetDelta<TileKey>,
}

pub(crate) fn nearest_chunk_center(local_voxels: Vec3) -> IVec3 {
	(local_voxels / CHUNK_SIZE as f32).round().as_ivec3()
}

pub(crate) fn update_desired_sources_delta(
	loader: &mut CameraVoxelLoader,
	grid: GridId,
	center: IVec3,
	settings: &CameraVoxelLoaderSettings,
	streaming: &GridStreaming,
) -> DesiredSourceDelta {
	let _span = span!();
	let new_bands = desired_lod_bands(center, settings);
	let old_bands = std::mem::take(loader.bands.entry(grid).or_default());

	let mut added = Vec::new();
	let mut removed = Vec::new();
	let desired_tiles = &mut loader.desired_tiles;
	run_over_diff(&old_bands, &new_bands, streaming, |lod, min, is_added| {
		let key = TileKey { grid, lod, min };
		if is_added {
			if desired_tiles.insert(key) {
				added.push(key);
			}
		} else {
			if desired_tiles.remove(&key) {
				removed.push(key);
			}
		}
	});

	loader.bands.insert(grid, new_bands);
	DesiredSourceDelta { tiles: SetDelta { added, removed } }
}

pub(crate) fn tile_has_present_source(streaming: &GridStreaming, key: TileKey) -> bool {
	if key.lod == 0 {
		streaming.presence().is_present(key.min)
	} else {
		let size = 1i32 << key.lod;
		streaming.presence().any_present_in_region(key.min, key.min + IVec3::splat(size) - IVec3::ONE)
	}
}

fn desired_lod_bands(center: IVec3, settings: &CameraVoxelLoaderSettings) -> Vec<LodBand> {
	let mut bands = Vec::with_capacity(settings.max_lod as usize + 1);

	let (near_min, near_max) = near_bounds(center, settings);
	bands.push(LodBand { lod: 0, outer: BandBounds { min: near_min, max: near_max }, inner: None });

	for lod in 1..=settings.max_lod {
		let (inner_min, inner_max) = lod_inner_bounds(center, settings, lod);
		let (outer_min, outer_max) = lod_outer_bounds(center, settings, lod);
		bands.push(LodBand {
			lod,
			outer: BandBounds { min: outer_min, max: outer_max },
			inner: Some(BandBounds { min: inner_min, max: inner_max }),
		});
	}

	bands
}

fn near_bounds(center: IVec3, settings: &CameraVoxelLoaderSettings) -> (IVec3, IVec3) {
	align_bounds_to_tile(center - IVec3::splat(settings.near_radius_chunks), center + IVec3::splat(settings.near_radius_chunks + 1), 2)
}

fn lod_inner_bounds(center: IVec3, settings: &CameraVoxelLoaderSettings, lod: u8) -> (IVec3, IVec3) {
	if lod == 1 { near_bounds(center, settings) } else { lod_outer_bounds(center, settings, lod - 1) }
}

fn lod_outer_bounds(center: IVec3, settings: &CameraVoxelLoaderSettings, lod: u8) -> (IVec3, IVec3) {
	let size = 1i32 << lod;
	let (min, max) = lod_inner_bounds(center, settings, lod);
	let bounds = (min - IVec3::splat(size * settings.rings_per_lod), max + IVec3::splat(size * settings.rings_per_lod));
	if lod < settings.max_lod { align_bounds_to_tile(bounds.0, bounds.1, 1i32 << (lod + 1)) } else { bounds }
}

fn align_bounds_to_tile(min: IVec3, max: IVec3, size: i32) -> (IVec3, IVec3) {
	(align_down_pow2(min, size), align_up_pow2(max, size))
}

fn align_down_pow2(v: IVec3, size: i32) -> IVec3 {
	let mask = !(size - 1);
	IVec3::new(v.x & mask, v.y & mask, v.z & mask)
}

fn align_up_pow2(v: IVec3, size: i32) -> IVec3 {
	let mask = !(size - 1);
	let add = size - 1;
	IVec3::new((v.x + add) & mask, (v.y + add) & mask, (v.z + add) & mask)
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn nearest_chunk_center_does_not_advance_to_next_chunk_at_half_chunk() {
		assert_eq!(nearest_chunk_center(Vec3::new(0.0, 0.0, 0.0)), IVec3::ZERO);
		assert_eq!(nearest_chunk_center(Vec3::new((CHUNK_SIZE / 2 - 1) as f32, 0.0, 0.0)), IVec3::ZERO);
		assert_eq!(nearest_chunk_center(Vec3::new((CHUNK_SIZE / 2) as f32, 0.0, 0.0)), IVec3::ZERO);
		assert_eq!(nearest_chunk_center(Vec3::new((CHUNK_SIZE - 1) as f32, 0.0, 0.0)), IVec3::ZERO);
	}

	#[test]
	fn nearest_chunk_center_uses_flooring_for_negative_positions() {
		assert_eq!(nearest_chunk_center(Vec3::new(-0.1, 0.0, 0.0)), IVec3::new(-1, 0, 0));
		assert_eq!(nearest_chunk_center(Vec3::new(-(CHUNK_SIZE as f32) * 0.5, 0.0, 0.0)), IVec3::new(-1, 0, 0));
		assert_eq!(nearest_chunk_center(Vec3::new(-(CHUNK_SIZE as f32), 0.0, 0.0)), IVec3::new(-1, 0, 0));
	}
}
