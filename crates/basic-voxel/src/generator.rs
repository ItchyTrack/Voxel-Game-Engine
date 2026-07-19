use std::collections::HashMap;

use bevy::math::{IVec3, U16Vec3};
use voxel_data::voxels::{VoxelRef, VoxelType, VoxelTypeId, Voxels};
use voxel_sources::VoxelLodGenerator;
use voxel_streaming::CHUNK_SIZE;

use crate::BasicVoxel;

#[derive(Clone, Copy, Debug, Default)]
pub struct BasicVoxelLodGenerator;

impl VoxelLodGenerator for BasicVoxelLodGenerator {
	fn input_type_id(&self) -> VoxelTypeId {
		BasicVoxel::TYPE_INFO.id
	}

	fn generate(
		&self,
		min: IVec3,
		size: IVec3,
		lod: f32,
		fetch: &dyn Fn(IVec3) -> Option<Voxels>,
	) -> Option<Voxels> {
		downsample_region(min, size, lod, fetch)
	}
}

/// Downsample the chunk area `min..min+size` by factor `2^floor(lod)` into a coarse
/// [`Voxels`], reading each chunk through `fetch`. Coordinates collapse directly to the
/// compressed space (each coarse voxel spans `factor` base voxels per axis), so the
/// full-resolution region is never materialized.
pub fn downsample_region(
	min: IVec3,
	size: IVec3,
	lod: f32,
	fetch: impl Fn(IVec3) -> Option<Voxels>,
) -> Option<Voxels> {
	let step = 1i32 << lod.max(0.0).floor() as u32;
	let step_v = IVec3::splat(step);
	let mut coarse: HashMap<IVec3, CoarseAccum> = HashMap::new();

	for chunk_z in 0..size.z {
		for chunk_y in 0..size.y {
			for chunk_x in 0..size.x {
				let local = IVec3::new(chunk_x, chunk_y, chunk_z);
				let Some(src) = fetch(min + local) else { continue };
				let chunk_origin = local * CHUNK_SIZE;
				accumulate_voxels(&src, chunk_origin, step_v, &mut coarse);
			}
		}
	}

	coarse_to_voxels(coarse)
}

#[derive(Default, Clone, Copy)]
struct CoarseAccum {
	color: [f64; 4],
	weight: f64,
}

fn accumulate_voxels(src: &Voxels, origin: IVec3, step_v: IVec3, coarse: &mut HashMap<IVec3, CoarseAccum>) {
	for (pos, run, id) in src.grid_tree().iter() {
		let Some(voxel) = src.voxel_for_palette_id(id).map(|voxel| BasicVoxel::from_voxel_ref(&voxel)) else { continue };
		let cell_min = origin + IVec3::new(pos.x as i32, pos.y as i32, pos.z as i32);
		let cell_max = cell_min + IVec3::splat(run as i32);
		let lo = cell_min.div_euclid(step_v);
		let hi = (cell_max - IVec3::ONE).div_euclid(step_v);
		for z in lo.z..=hi.z {
			for y in lo.y..=hi.y {
				for x in lo.x..=hi.x {
					let coarse_pos = IVec3::new(x, y, z);
					let coarse_lo = coarse_pos * step_v;
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

fn coarse_to_voxels(coarse: HashMap<IVec3, CoarseAccum>) -> Option<Voxels> {
	let mut out = Voxels::new::<BasicVoxel>();
	for (coarse_pos, accum) in coarse {
		if accum.weight <= 0.0 { continue; }
		let average: [f64; 4] = std::array::from_fn(|channel| accum.color[channel] / accum.weight);
		out.add_voxel(
			U16Vec3::new(coarse_pos.x as u16, coarse_pos.y as u16, coarse_pos.z as u16),
			VoxelRef::new(BasicVoxel::TYPE_INFO.id, &quantized_lod_voxel(average)),
		);
	}
	if out.is_empty() { None } else { Some(out) }
}

fn quantized_lod_voxel(average: [f64; 4]) -> [u8; BasicVoxel::TYPE_INFO.size_bytes as usize] {
	let mut bytes = [0u8; BasicVoxel::TYPE_INFO.size_bytes as usize];
	BasicVoxel {
		color: [
			quantize_channel(average[0], 6),
			quantize_channel(average[1], 6),
			quantize_channel(average[2], 6),
			255,
		],
		mass: 0,
	}.into_bytes(&mut bytes);
	bytes
}

fn quantize_channel(value: f64, levels: u8) -> u8 {
	let max_level = (levels - 1) as f64;
	let level = ((value.clamp(0.0, 255.0) / 255.0) * max_level).round();
	((level / max_level) * 255.0).round() as u8
}
