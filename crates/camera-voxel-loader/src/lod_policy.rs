use bevy::prelude::*;
use tracy_client::span;
use voxel_data::grid::GridId;
use tile_data::{CHUNK_SIZE, ChunkRegion, TileClassId};
use voxel_streaming::GridStreaming;

use crate::camera_voxel_loader::{CameraVoxelLoader, CameraVoxelLoaderSettings};
use crate::lod_bands::{run_over_diff, LodBand};
use crate::types::GridTileKey;

pub(crate) struct DesiredSourceDelta {
	pub(crate) added: Vec<GridTileKey>,
	pub(crate) removed: Vec<GridTileKey>,
}

pub(crate) fn nearest_chunk_center(local_voxels: Vec3) -> IVec3 {
	(local_voxels / CHUNK_SIZE as f32).round().as_ivec3()
}

pub(crate) fn update_desired_sources_delta(
	loader: &mut CameraVoxelLoader,
	grid: GridId,
	class: TileClassId,
	center: IVec3,
	settings: &CameraVoxelLoaderSettings,
	streaming: &GridStreaming,
) -> DesiredSourceDelta {
	let _span = span!();
	let new_bands = desired_lod_bands(center, settings);
	let old_bands = std::mem::take(loader.bands.entry(grid).or_default());
	let old_class = loader.classes.insert(grid, class);

	let mut added = Vec::new();
	let mut removed = Vec::new();
	if old_class.is_none_or(|old| old == class) {
		run_over_diff(&old_bands, &new_bands, streaming, |lod, min, is_added| {
			let key = GridTileKey::new(grid, class, lod, min);
			if is_added { added.push(key); } else { removed.push(key); }
		});
	} else {
		let old_class = old_class.unwrap();
		run_over_diff(&old_bands, &[], streaming, |lod, min, _| removed.push(GridTileKey::new(grid, old_class, lod, min)));
		run_over_diff(&[], &new_bands, streaming, |lod, min, _| added.push(GridTileKey::new(grid, class, lod, min)));
	}

	loader.bands.insert(grid, new_bands);
	DesiredSourceDelta { added, removed }
}

pub(crate) fn tile_has_present_source(streaming: &GridStreaming, key: GridTileKey) -> bool {
	streaming.presence().any_present_in_region(key.region)
}

fn desired_lod_bands(center: IVec3, settings: &CameraVoxelLoaderSettings) -> Vec<LodBand> {
	let mut bands = Vec::with_capacity(settings.max_lod as usize + 1);

	let (near_min, near_max) = near_bounds(center, settings);
	bands.push(LodBand { lod: 0, outer: ChunkRegion::from_min_end(near_min, near_max).unwrap(), inner: None });

	let mut prev_outer = (near_min, near_max);
	for lod in 1..=settings.max_lod {
		let size = 1i32 << lod;
		let inner = prev_outer;
		let expand = IVec3::splat(size * settings.rings_per_lod as i32);
		let mut outer = (inner.0 - expand, inner.1 + expand);
		if lod < settings.max_lod {
			outer = align_bounds_to_tile(outer.0, outer.1, 1i32 << (lod + 1));
		}
		bands.push(LodBand {
			lod,
			outer: ChunkRegion::from_min_end(outer.0, outer.1).unwrap(),
			inner: Some(ChunkRegion::from_min_end(inner.0, inner.1).unwrap()),
		});
		prev_outer = outer;
	}

	bands
}

fn near_bounds(center: IVec3, settings: &CameraVoxelLoaderSettings) -> (IVec3, IVec3) {
	let radius = settings.near_radius_chunks as i32;
	align_bounds_to_tile(center - IVec3::splat(radius), center + IVec3::splat(radius + 1), 2)
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
	use tile_data::NonZeroChunkRegion;

	use super::*;

	// The center is only the reference point the LOD bands are built around, so it snaps to the
	// nearest chunk: the detail box recenters when the camera crosses a chunk's midpoint, keeping it
	// centered on the camera rather than biased toward the origin-side chunk.
	#[test]
	fn nearest_chunk_center_snaps_to_the_nearest_chunk_at_the_half_chunk_point() {
		assert_eq!(nearest_chunk_center(Vec3::new(0.0, 0.0, 0.0)), IVec3::ZERO);
		assert_eq!(nearest_chunk_center(Vec3::new((CHUNK_SIZE / 2 - 1) as f32, 0.0, 0.0)), IVec3::ZERO);
		assert_eq!(nearest_chunk_center(Vec3::new((CHUNK_SIZE / 2) as f32, 0.0, 0.0)), IVec3::new(1, 0, 0));
		assert_eq!(nearest_chunk_center(Vec3::new((CHUNK_SIZE - 1) as f32, 0.0, 0.0)), IVec3::new(1, 0, 0));
	}

	#[test]
	fn nearest_chunk_center_rounds_symmetrically_for_negative_positions() {
		assert_eq!(nearest_chunk_center(Vec3::new(-0.1, 0.0, 0.0)), IVec3::ZERO);
		assert_eq!(nearest_chunk_center(Vec3::new(-(CHUNK_SIZE as f32) * 0.5, 0.0, 0.0)), IVec3::new(-1, 0, 0));
		assert_eq!(nearest_chunk_center(Vec3::new(-(CHUNK_SIZE as f32), 0.0, 0.0)), IVec3::new(-1, 0, 0));
	}

	fn policy_settings() -> CameraVoxelLoaderSettings {
		CameraVoxelLoaderSettings { max_lod: 3, near_radius_chunks: 4, rings_per_lod: 3 }
	}

	// The desired set computed from scratch at `center`: a fresh loader has empty bands, so a
	// single delta emits the whole set.
	fn desired_at(settings: &CameraVoxelLoaderSettings, grid: GridId, streaming: &GridStreaming, center: IVec3) -> std::collections::HashSet<GridTileKey> {
		let mut loader = CameraVoxelLoader::default();
		loader.settings = settings.clone();
		let delta = update_desired_sources_delta(&mut loader, grid, TileClassId(0), center, settings, streaming);
		loader.tiles.apply_delta(&delta.added, &delta.removed, &mut Vec::new(), &mut Vec::new());
		loader.tiles.desired_set().clone()
	}

	fn flight_path() -> Vec<IVec3> {
		let steps = [
			IVec3::new(1, 0, 0), IVec3::new(1, 0, 1), IVec3::new(0, 1, 1), IVec3::new(-1, 1, 0),
			IVec3::new(-1, 0, -1), IVec3::new(0, -1, -1), IVec3::new(2, 0, 1), IVec3::new(-2, 1, 0),
		];
		let mut centers = vec![IVec3::ZERO];
		let mut p = IVec3::ZERO;
		for i in 0..40 {
			p += steps[i % steps.len()];
			centers.push(p);
		}
		centers
	}

	// The incremental delta (diffing new bands against the loader's stored old bands) must leave the
	// desired set identical to recomputing it from scratch at each center. This is the core
	// correctness property of `run_over_diff`.
	#[test]
	fn incremental_desired_delta_converges_to_fresh_rebuild_while_flying() {
		let grid: GridId = Entity::PLACEHOLDER;
		let settings = policy_settings();
		let mut streaming = GridStreaming::default();
		streaming.mark_present_area(NonZeroChunkRegion::from_min_size(IVec3::splat(-80), UVec3::splat(161)).unwrap());

		let mut flying = CameraVoxelLoader::default();
		flying.settings = settings.clone();
		for center in flight_path() {
			let delta = update_desired_sources_delta(&mut flying, grid, TileClassId(0), center, &settings, &streaming);
			flying.tiles.apply_delta(&delta.added, &delta.removed, &mut Vec::new(), &mut Vec::new());
			let fresh = desired_at(&settings, grid, &streaming, center);
			assert_eq!(flying.tiles.desired_set(), &fresh, "incremental desired set diverged from fresh rebuild at {center:?}");
		}
	}

	// Presence-gated: the policy only desires tiles that actually cover loaded source data.
	#[test]
	fn every_desired_tile_covers_at_least_one_present_chunk() {
		let grid: GridId = Entity::PLACEHOLDER;
		let settings = policy_settings();
		let mut streaming = GridStreaming::default();
		streaming.mark_present_area(NonZeroChunkRegion::from_min_size(IVec3::splat(-40), UVec3::splat(81)).unwrap());

		let desired = desired_at(&settings, grid, &streaming, IVec3::new(3, -2, 5));
		assert!(!desired.is_empty(), "control setup produced no desired tiles");
		for tile in &desired {
			let min = tile.tile_key.region.min();
			let max = tile.tile_key.region.end();
			let mut covers_present = false;
			'scan: for x in min.x..max.x {
				for y in min.y..max.y {
					for z in min.z..max.z {
						if streaming.presence().is_present(IVec3::new(x, y, z)) {
							covers_present = true;
							break 'scan;
						}
					}
				}
			}
			assert!(covers_present, "desired tile {tile:?} covers no present chunk");
		}
	}

	// Concentric, tile-aligned LOD bands must tile space without overlap: no present chunk may be
	// owned by more than one desired tile, or coverage would double up / retire ambiguously.
	#[test]
	fn no_present_chunk_has_more_than_one_desired_owner() {
		let grid: GridId = Entity::PLACEHOLDER;
		let settings = policy_settings();
		let mut streaming = GridStreaming::default();
		streaming.mark_present_area(NonZeroChunkRegion::from_min_size(IVec3::splat(-60), UVec3::splat(121)).unwrap());

		for center in [IVec3::ZERO, IVec3::new(1, 0, 0), IVec3::new(12, 0, -4), IVec3::new(-17, 3, 9)] {
			let desired = desired_at(&settings, grid, &streaming, center);
			let mut owners: std::collections::HashMap<IVec3, u32> = std::collections::HashMap::new();
			for tile in &desired {
				let min = tile.tile_key.region.min();
				let max = tile.tile_key.region.end();
				for x in min.x..max.x {
					for y in min.y..max.y {
						for z in min.z..max.z {
							let chunk = IVec3::new(x, y, z);
							if streaming.presence().is_present(chunk) {
								*owners.entry(chunk).or_default() += 1;
							}
						}
					}
				}
			}
			if let Some((chunk, count)) = owners.iter().find(|&(_, &count)| count > 1) {
				panic!("center {center:?}: present chunk {chunk:?} owned by {count} desired tiles");
			}
		}
	}
}
