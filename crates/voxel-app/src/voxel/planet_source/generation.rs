use bevy::math::U16Vec3;
use bevy::prelude::*;
use tracy_client::span;
use tile_data::{CHUNK_SIZE, NonZeroChunkRegion, chunk_origin};
use voxel_data::voxels::{VoxelType, Voxels};
use voxel_tasks::CancellationToken;

use basic_voxel::{BasicVoxel, LodVoxel};

use super::config::{TILE_INWARD_DEPTH, TILE_OUTWARD_HEIGHT, TILE_SHAPE_EPSILON};
use super::tiles::{PlanetTile, planet_tiles};

pub(super) fn planet_voxel_unchecked(_pos: IVec3) -> BasicVoxel {
	BasicVoxel { color: [200, 100, 30, 255], mass: 100 }
}

pub(super) fn planet_lod_voxel_unchecked(_pos: IVec3) -> LodVoxel {
	LodVoxel::solid([200, 100, 30, 255])
}

pub(super) fn build_planet_region<V: VoxelType>(
	tile_index: usize,
	region: NonZeroChunkRegion,
	chunks: &[IVec3],
	lod: u8,
	cancellation: &CancellationToken,
	sample: impl Fn(IVec3) -> V,
) -> Option<Voxels> {
	let _zone = span!("planet build region");
	let tile = planet_tiles().get(tile_index)?;
	let step = 1i32 << lod as u32;
	let sample_offset = step / 2;
	let chunk_extent = CHUNK_SIZE as i32 / step;
	let step_f = step as f32;

	let mut areas = Vec::new();

	for &chunk in chunks {
		if cancellation.is_cancelled() { return None; }
		let origin = chunk_origin(chunk);
		let chunk_offset = (chunk - region.min()) * chunk_extent;
		let sample_base_z = origin.z as f32 + sample_offset as f32 + 0.5;

		for y in 0..chunk_extent {
			let sample_y = (origin.y + y * step + sample_offset) as f32 + 0.5;
			for x in 0..chunk_extent {
				let sample_x = (origin.x + x * step + sample_offset) as f32 + 0.5;
				let Some((z0, z1)) =
					column_shape_z_range(tile, sample_x, sample_y, sample_base_z, chunk_extent, step_f)
				else {
					continue;
				};

				let pos = chunk_offset + IVec3::new(x, y, z0);
				let size = UVec3::new(1, 1, (z1 - z0) as u32);
				areas.push((pos.as_u16vec3(), size, sample(IVec3::new(x, y, z0))));
			}
		}
	}

	if areas.is_empty() {
		return None;
	}
	let area_refs: Vec<_> = areas.iter().map(|(pos, size, voxel)| (*pos, *size, voxel.get_ref())).collect();
	let mut voxels = Voxels::new::<V>();
	voxels.add_areas(&area_refs);
	Some(voxels)
}

fn column_shape_z_range(
	tile: &PlanetTile,
	sample_x: f32,
	sample_y: f32,
	sample_base_z: f32,
	extent_z: i32,
	step: f32,
) -> Option<(i32, i32)> {
	let mut min_sample_z = -TILE_INWARD_DEPTH as f32;
	let mut max_sample_z = TILE_OUTWARD_HEIGHT as f32;

	for h in &tile.halfspaces {
		let base = h.normal.x * sample_x + h.normal.y * sample_y + h.offset;
		if h.normal.z > 1e-6 {
			min_sample_z = min_sample_z.max((-TILE_SHAPE_EPSILON - base) / h.normal.z);
		} else if h.normal.z < -1e-6 {
			max_sample_z = max_sample_z.min((-TILE_SHAPE_EPSILON - base) / h.normal.z);
		} else if base < -TILE_SHAPE_EPSILON {
			return None;
		}
	}

	if min_sample_z >= max_sample_z {
		return None;
	}

	let z0 = (((min_sample_z - sample_base_z) / step).ceil() as i32).clamp(0, extent_z);
	let z1 = (((max_sample_z - sample_base_z) / step).ceil() as i32).clamp(0, extent_z);
	(z0 < z1).then_some((z0, z1))
}
