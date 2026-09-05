use bevy::{math::{DMat3, DVec3}, prelude::*};
use tracy_client::span;
use tile_data::{CHUNK_SIZE, NonZeroChunkRegion, chunk_origin};
use voxel_data::voxels::{VoxelType, Voxels};
use voxel_mass::{CenterOfMass, InertiaTensor, Mass, MassProperties, RotationalInertia};
use voxel_tasks::CancellationToken;

use basic_voxel::{BasicVoxel, LodVoxel};

use super::config::{TILE_INWARD_DEPTH, TILE_OUTWARD_HEIGHT, TILE_SHAPE_EPSILON};
use super::tiles::{PlanetTile, planet_tiles};

const PLANET_VOXEL_MASS: u32 = 100;

pub(super) fn planet_voxel_unchecked(_pos: IVec3) -> BasicVoxel {
	BasicVoxel { color: [200, 100, 30, 255], mass: PLANET_VOXEL_MASS }
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
				areas.push((pos.as_uvec3(), size, sample(IVec3::new(x, y, z0))));
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

pub(super) fn planet_mass_properties(tile: &PlanetTile) -> MassProperties {
	let mut total = MassProperties::ZERO;
	for &chunk in &tile.present_chunks {
		total = total.checked_add(planet_chunk_mass_properties(tile, chunk));
	}
	assert!(total.rotational_inertia.0.mat.is_finite(), "planet mass estimate has non-finite inertia");
	total
}

fn planet_chunk_mass_properties(tile: &PlanetTile, chunk: IVec3) -> MassProperties {
	let origin = chunk_origin(chunk);
	let extent = i32::try_from(CHUNK_SIZE).expect("chunk size exceeds i32");
	let sample_base_z = origin.z as f32 + 0.5;
	let mut properties = MassProperties::ZERO;

	for y in 0..extent {
		let voxel_y = origin.y.checked_add(y).expect("planet voxel coordinate overflow");
		let sample_y = voxel_y as f32 + 0.5;
		for x in 0..extent {
			let voxel_x = origin.x.checked_add(x).expect("planet voxel coordinate overflow");
			let sample_x = voxel_x as f32 + 0.5;
			let Some((z0, z1)) =
				column_shape_z_range(tile, sample_x, sample_y, sample_base_z, extent, 1.0)
			else {
				continue;
			};

			let voxel_z = origin.z.checked_add(z0).expect("planet voxel coordinate overflow");
			let length = u64::try_from(z1.checked_sub(z0).expect("planet column length underflow"))
				.expect("planet column length is negative");
			properties = properties.checked_add(vertical_run_mass_properties(
				DVec3::new(f64::from(voxel_x), f64::from(voxel_y), f64::from(voxel_z)), length,
			));
		}
	}

	properties
}

fn vertical_run_mass_properties(origin: DVec3, count: u64) -> MassProperties {
	let mass = u64::from(PLANET_VOXEL_MASS).checked_mul(count).expect("planet mass overflow");
	let length = count as f64;
	MassProperties {
		mass: Mass(mass),
		center_of_mass: CenterOfMass(origin + DVec3::new(0.5, 0.5, length * 0.5)),
		rotational_inertia: RotationalInertia(InertiaTensor::from_mat3(DMat3::from_diagonal(
			DVec3::new(1.0 + length * length, 1.0 + length * length, 2.0) * (mass as f64 / 12.0),
		))),
	}
}

pub(super) fn column_shape_z_range(
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
