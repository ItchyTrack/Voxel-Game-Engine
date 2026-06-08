use std::collections::HashMap;

use bevy::math::{I16Vec3, IVec3};

use voxel_data::voxels::{Voxel, Voxels};
use voxel_streaming::CHUNK_SIZE;

/// Volume-weighted accumulator for one coarse voxel.
#[derive(Default, Clone, Copy)]
struct CoarseAccum {
	color: [f64; 4],
	weight: f64,
}

/// Downsample the chunk area `min..min+size` by factor `2^floor(lod)` into a coarse
/// [`Voxels`], reading each chunk through `fetch`. Coordinates collapse directly to the
/// compressed space (each coarse voxel spans `factor` base voxels per axis), so the
/// full-resolution region is never materialized.
///
/// Each coarse voxel's colour is the volume-weighted average of the base voxels it
/// covers, snapped to the nearest colour present in the source. Snapping keeps the
/// coarse palette within the source palette, which the GPU tree caps at 254 entries.
pub fn downsample_region(
	min: IVec3,
	size: IVec3,
	lod: f32,
	fetch: impl Fn(IVec3) -> Option<Voxels>,
) -> Voxels {
	let step = 1i32 << lod.max(0.0).floor() as u32;
	let step_v = IVec3::splat(step);

	let mut coarse: HashMap<IVec3, CoarseAccum> = HashMap::new();
	// Distinct source voxels, used as the palette the averaged colours snap back to.
	let mut source_voxels: Vec<Voxel> = Vec::new();
	let mut seen_colors: HashMap<[u8; 4], usize> = HashMap::new();

	for chunk_z in 0..size.z {
		for chunk_y in 0..size.y {
			for chunk_x in 0..size.x {
				let local = IVec3::new(chunk_x, chunk_y, chunk_z);
				let Some(src) = fetch(min + local) else { continue };
				let chunk_origin = local * CHUNK_SIZE;
				for (pos, run, id) in src.grid_tree().iter() {
					let Some(voxel) = src.palette().voxel(id).copied() else { continue };
					seen_colors.entry(voxel.color).or_insert_with(|| {
						source_voxels.push(voxel);
						source_voxels.len() - 1
					});
					// Source cell occupies the half-open cube `[cell_min, cell_max)`.
					let cell_min = chunk_origin + IVec3::new(pos.x as i32, pos.y as i32, pos.z as i32);
					let cell_max = cell_min + IVec3::splat(run as i32);
					let lo = cell_min.div_euclid(step_v);
					let hi = (cell_max - IVec3::ONE).div_euclid(step_v);
					for z in lo.z..=hi.z {
						for y in lo.y..=hi.y {
							for x in lo.x..=hi.x {
								let coarse_pos = IVec3::new(x, y, z);
								let coarse_lo = coarse_pos * step;
								let coarse_hi = coarse_lo + step_v;
								let overlap = (cell_max.min(coarse_hi) - cell_min.max(coarse_lo)).max(IVec3::ZERO);
								let weight = overlap.x as f64 * overlap.y as f64 * overlap.z as f64;
								if weight <= 0.0 { continue; }
								let accum = coarse.entry(coarse_pos).or_default();
								for channel in 0..4 {
									accum.color[channel] += voxel.color[channel] as f64 * weight;
								}
								accum.weight += weight;
							}
						}
					}
				}
			}
		}
	}

	let mut out = Voxels::new();
	for (coarse_pos, accum) in coarse {
		if accum.weight <= 0.0 { continue; }
		let average: [f64; 4] = std::array::from_fn(|channel| accum.color[channel] / accum.weight);
		let voxel = nearest_source_voxel(&source_voxels, average);
		out.add_voxel(
			I16Vec3::new(coarse_pos.x as i16, coarse_pos.y as i16, coarse_pos.z as i16),
			voxel,
		);
	}
	out
}

/// The source voxel whose colour is closest to `average` (squared RGBA distance).
fn nearest_source_voxel(source_voxels: &[Voxel], average: [f64; 4]) -> Voxel {
	source_voxels
		.iter()
		.copied()
		.min_by(|a, b| color_distance_sq(a.color, average).total_cmp(&color_distance_sq(b.color, average)))
		.unwrap_or(Voxel { color: [0, 0, 0, 0], mass: 0 })
}

fn color_distance_sq(color: [u8; 4], target: [f64; 4]) -> f64 {
	(0..4)
		.map(|channel| {
			let delta = color[channel] as f64 - target[channel];
			delta * delta
		})
		.sum()
}
