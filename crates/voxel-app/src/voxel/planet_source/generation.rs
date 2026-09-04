use bevy::{math::DMat3, prelude::*};
use tracy_client::span;
use tile_data::{CHUNK_SIZE, NonZeroChunkRegion, chunk_origin};
use voxel_data::voxels::{VoxelType, Voxels};
use voxel_mass::MassProperties;
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
		total = checked_sum_mass_properties(total, planet_chunk_mass_properties(tile, chunk));
	}
	assert!(total.inertia_at_origin().is_finite(), "planet mass estimate has non-finite inertia");
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
			properties = checked_sum_mass_properties(
				properties,
				vertical_run_mass_properties([i64::from(voxel_x), i64::from(voxel_y), i64::from(voxel_z)], length),
			);
		}
	}

	properties
}

fn vertical_run_mass_properties(origin: [i64; 3], count: u64) -> MassProperties {
	let voxel_mass = u64::from(PLANET_VOXEL_MASS);
	let mass = voxel_mass.checked_mul(count).expect("planet mass overflow");
	let count_i128 = i128::from(count);
	let index_sum = count
		.checked_mul(count.checked_sub(1).expect("planet column is empty"))
		.expect("planet coordinate sum overflow") / 2;
	let coordinate_sums = [
		i128::from(origin[0]).checked_mul(count_i128).expect("planet coordinate sum overflow"),
		i128::from(origin[1]).checked_mul(count_i128).expect("planet coordinate sum overflow"),
		i128::from(origin[2])
			.checked_mul(count_i128)
			.and_then(|sum| sum.checked_add(i128::from(index_sum)))
			.expect("planet coordinate sum overflow"),
	];
	let first_moment = coordinate_sums.map(|sum| {
		let weighted = sum.checked_mul(i128::from(PLANET_VOXEL_MASS)).expect("planet first moment overflow");
		i64::try_from(weighted).expect("planet first moment overflow")
	});

	MassProperties::new(mass, first_moment, vertical_run_inertia(origin, count))
}

fn vertical_run_inertia(origin: [i64; 3], count: u64) -> DMat3 {
	let n = count as f64;
	let x = origin[0] as f64 + 0.5;
	let y = origin[1] as f64 + 0.5;
	let z = origin[2] as f64 + 0.5;
	let index_sum = n * (n - 1.0) * 0.5;
	let index_square_sum = n * (n - 1.0) * (2.0 * n - 1.0) / 6.0;
	let z_sum = n * z + index_sum;
	let z_square_sum = n * z * z + 2.0 * z * index_sum + index_square_sum;
	let cube_center_inertia = n / 6.0;
	let voxel_mass = f64::from(PLANET_VOXEL_MASS);
	let xx = voxel_mass * (n * y * y + z_square_sum + cube_center_inertia);
	let yy = voxel_mass * (n * x * x + z_square_sum + cube_center_inertia);
	let zz = voxel_mass * (n * x * x + n * y * y + cube_center_inertia);

	DMat3::from_cols_array(&[
		xx, -voxel_mass * n * x * y, -voxel_mass * x * z_sum,
		-voxel_mass * n * x * y, yy, -voxel_mass * y * z_sum,
		-voxel_mass * x * z_sum, -voxel_mass * y * z_sum, zz,
	])
}

fn checked_sum_mass_properties(a: MassProperties, b: MassProperties) -> MassProperties {
	let mass = a.mass().checked_add(b.mass()).expect("planet mass overflow");
	let a_first_moment = a.first_moment();
	let b_first_moment = b.first_moment();
	let first_moment = std::array::from_fn(|axis| {
		a_first_moment[axis].checked_add(b_first_moment[axis]).expect("planet first moment overflow")
	});
	let inertia_at_origin = a.inertia_at_origin() + b.inertia_at_origin();
	assert!(inertia_at_origin.is_finite(), "planet mass estimate has non-finite inertia");
	MassProperties::new(mass, first_moment, inertia_at_origin)
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
