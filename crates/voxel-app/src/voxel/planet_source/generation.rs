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

pub(super) fn planet_voxel(_pos: IVec3) -> BasicVoxel {
	BasicVoxel { color: [200, 100, 30, 255], mass: PLANET_VOXEL_MASS }
}

pub(super) fn planet_lod_voxel(_pos: IVec3) -> LodVoxel {
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
	let chunk_extent = CHUNK_SIZE as i32 / step;

	let mut areas = Vec::new();

	for &chunk in chunks {
		if cancellation.is_cancelled() { return None; }
		let origin = chunk_origin(chunk);
		let chunk_offset = (chunk - region.min()) * chunk_extent;
		for_each_shape_column(tile, origin, step, |x, y, z0, z1| {
			let pos = chunk_offset + IVec3::new(x, y, z0);
			let size = UVec3::new(1, 1, (z1 - z0) as u32);
			areas.push((pos.as_uvec3(), size, sample(IVec3::new(x, y, z0))));
		});
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
		total = total.add(planet_chunk_mass_properties(tile, chunk));
	}
	assert!(total.rotational_inertia.0.mat.is_finite(), "planet mass estimate has non-finite inertia");
	total
}

fn planet_chunk_mass_properties(tile: &PlanetTile, chunk: IVec3) -> MassProperties {
	let origin = chunk_origin(chunk);
	let mut properties = MassProperties::ZERO;

	for_each_shape_column(tile, origin, 1, |x, y, z0, z1| {
		let voxel_x = origin.x + x;
		let voxel_y = origin.y + y;
		let voxel_z = origin.z + z0;
		let length = u64::try_from(z1.checked_sub(z0).expect("planet column length underflow"))
			.expect("planet column length is negative");
		properties = properties.add(vertical_run_mass_properties(
			DVec3::new(f64::from(voxel_x), f64::from(voxel_y), f64::from(voxel_z)), length,
		));
	});

	properties
}

fn vertical_run_mass_properties(origin: DVec3, count: u64) -> MassProperties {
	let mass = u64::from(PLANET_VOXEL_MASS) * count;
	let length = count as f64;
	MassProperties {
		mass: Mass(mass),
		center_of_mass: CenterOfMass(origin + DVec3::new(0.5, 0.5, length * 0.5)),
		rotational_inertia: RotationalInertia(InertiaTensor::from_mat3(DMat3::from_diagonal(
			DVec3::new(1.0 + length * length, 1.0 + length * length, 2.0) * (mass as f64 / 12.0),
		))),
	}
}

#[inline(always)]
fn for_each_shape_column(
	tile: &PlanetTile,
	origin: IVec3,
	step: i32,
	mut visit: impl FnMut(i32, i32, i32, i32),
) {
	let extent = CHUNK_SIZE as i32 / step;
	// Coarse LODs have too few columns to benefit from row setup.
	if extent <= 4 {
		let sample_offset = step / 2;
		let sample_base_z = origin.z as f32 + sample_offset as f32 + 0.5;
		if extent == 1 {
			let sample_x = (origin.x + sample_offset) as f32 + 0.5;
			let sample_y = (origin.y + sample_offset) as f32 + 0.5;
			if let Some((z0, z1)) = column_shape_z_range(tile, sample_x, sample_y, sample_base_z, 1, step as f32) {
				visit(0, 0, z0, z1);
			}
			return;
		}
		for y in 0..extent {
			let sample_y = (origin.y + y * step + sample_offset) as f32 + 0.5;
			for x in 0..extent {
				let sample_x = (origin.x + x * step + sample_offset) as f32 + 0.5;
				if let Some((z0, z1)) = column_shape_z_range(
					tile, sample_x, sample_y, sample_base_z, extent, step as f32,
				) {
					visit(x, y, z0, z1);
				}
			}
		}
	} else {
		for_each_shape_column_batched(tile, origin, step, visit);
	}
}

fn for_each_shape_column_batched(
	tile: &PlanetTile,
	origin: IVec3,
	step: i32,
	mut visit: impl FnMut(i32, i32, i32, i32),
) {
	let extent = CHUNK_SIZE as usize / step as usize;
	let sample_offset = step / 2;
	let sample_base_z = origin.z as f32 + sample_offset as f32 + 0.5;
	let z_index = |z: f32| {
		(((z - sample_base_z) / step as f32).ceil() as i32).clamp(0, extent as i32)
	};
	let slab_z0 = z_index(-TILE_INWARD_DEPTH as f32);
	let slab_z1 = z_index(TILE_OUTWARD_HEIGHT as f32);
	if slab_z0 >= slab_z1 { return; }

	let mut sample_x = [0.0; CHUNK_SIZE as usize];
	for (x, sample) in sample_x[..extent].iter_mut().enumerate() {
		*sample = (origin.x + x as i32 * step + sample_offset) as f32 + 0.5;
	}

	// Reject empty chunks using the furthest sampled corner of each halfspace.
	let sample_y0 = (origin.y + sample_offset) as f32 + 0.5;
	let sample_y1 = (origin.y + (extent as i32 - 1) * step + sample_offset) as f32 + 0.5;
	for h in &tile.halfspaces {
		let x = if h.normal.x >= 0.0 { sample_x[extent - 1] } else { sample_x[0] };
		let y = if h.normal.y >= 0.0 { sample_y1 } else { sample_y0 };
		let base = h.normal.x * x + h.normal.y * y + h.offset;
		if h.normal.z > 1e-6 {
			if z_index((-TILE_SHAPE_EPSILON - base) / h.normal.z) >= slab_z1 { return; }
		} else if h.normal.z < -1e-6 {
			if z_index((-TILE_SHAPE_EPSILON - base) / h.normal.z) <= slab_z0 { return; }
		} else if base < -TILE_SHAPE_EPSILON {
			return;
		}
	}

	'rows: for y in 0..extent {
		let sample_y = (origin.y + y as i32 * step + sample_offset) as f32 + 0.5;
		let mut min_z = [-TILE_INWARD_DEPTH as f32; CHUNK_SIZE as usize];
		let mut max_z = [TILE_OUTWARD_HEIGHT as f32; CHUNK_SIZE as usize];

		// Row endpoints bound every column, so many planes need no per-column work.
		for h in &tile.halfspaces {
			let y_term = h.normal.y * sample_y;
			// Keep the scalar arithmetic order: rounding here affects voxel ownership.
			let base0 = h.normal.x * sample_x[0] + y_term + h.offset;
			let base1 = h.normal.x * sample_x[extent - 1] + y_term + h.offset;
			if h.normal.z > 1e-6 {
				let bound0 = (-TILE_SHAPE_EPSILON - base0) / h.normal.z;
				let bound1 = (-TILE_SHAPE_EPSILON - base1) / h.normal.z;
				if z_index(bound0.min(bound1)) >= slab_z1 { continue 'rows; }
				if z_index(bound0.max(bound1)) <= slab_z0 { continue; }
				for (min_z, &x) in min_z[..extent].iter_mut().zip(&sample_x[..extent]) {
					let base = h.normal.x * x + y_term + h.offset;
					*min_z = min_z.max((-TILE_SHAPE_EPSILON - base) / h.normal.z);
				}
			} else if h.normal.z < -1e-6 {
				let bound0 = (-TILE_SHAPE_EPSILON - base0) / h.normal.z;
				let bound1 = (-TILE_SHAPE_EPSILON - base1) / h.normal.z;
				if z_index(bound0.max(bound1)) <= slab_z0 { continue 'rows; }
				if z_index(bound0.min(bound1)) >= slab_z1 { continue; }
				for (max_z, &x) in max_z[..extent].iter_mut().zip(&sample_x[..extent]) {
					let base = h.normal.x * x + y_term + h.offset;
					*max_z = max_z.min((-TILE_SHAPE_EPSILON - base) / h.normal.z);
				}
			} else {
				if base0.max(base1) < -TILE_SHAPE_EPSILON { continue 'rows; }
				if base0.min(base1) >= -TILE_SHAPE_EPSILON { continue; }
				for (max_z, &x) in max_z[..extent].iter_mut().zip(&sample_x[..extent]) {
					let base = h.normal.x * x + y_term + h.offset;
					if base < -TILE_SHAPE_EPSILON { *max_z = f32::NEG_INFINITY; }
				}
			}
		}

		for x in 0..extent {
			if min_z[x] >= max_z[x] { continue; }
			let z0 = z_index(min_z[x]);
			let z1 = z_index(max_z[x]);
			if z0 < z1 { visit(x as i32, y as i32, z0, z1); }
		}
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
		if min_sample_z >= max_sample_z { return None; }
	}

	let z0 = (((min_sample_z - sample_base_z) / step).ceil() as i32).clamp(0, extent_z);
	let z1 = (((max_sample_z - sample_base_z) / step).ceil() as i32).clamp(0, extent_z);
	(z0 < z1).then_some((z0, z1))
}
