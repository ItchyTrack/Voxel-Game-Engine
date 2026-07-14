use bevy::math::U16Vec3;
use bevy::prelude::*;
use tracy_client::span;
use voxel_data::voxels::{Voxel, Voxels};
use voxel_sources::CancellationToken;
use voxel_streaming::{CHUNK_SIZE, chunk_origin};

use super::config::{PLANET_RADIUS, TILE_INWARD_DEPTH, TILE_OUTWARD_HEIGHT, TILE_SHAPE_EPSILON};
use super::terrain::{terrain_color, terrain_sample};
use super::tiles::{PlanetTile, planet_tiles};

pub(super) fn build_planet_chunk(tile_index: usize, chunk: IVec3, cancellation: &CancellationToken) -> Option<Voxels> {
	let _zone = span!("planet build chunk");
	let tile = planet_tiles().get(tile_index)?;
	let origin = chunk_origin(chunk);
	let mut points = Vec::new();
	append_planet_samples(
		tile,
		origin,
		IVec3::splat(CHUNK_SIZE),
		1,
		0,
		true,
		Some(cancellation),
		&mut points,
	);
	if cancellation.is_cancelled() { return None; }
	tracy_client::plot!("planet chunk emitted voxels", points.len() as f64);
	points_to_voxels(points)
}

pub(super) fn build_planet_lod_region(
	tile_index: usize,
	min_chunk: IVec3,
	size_chunks: IVec3,
	lod: f32,
	cancellation: &CancellationToken,
) -> Option<Voxels> {
	let _zone = span!("planet build lod region");
	let tile = planet_tiles().get(tile_index)?;
	let step = 1i32 << lod.max(0.0).floor() as u32;
	let sample_offset = step / 2;
	let extent = (size_chunks * CHUNK_SIZE) / step;
	let origin = chunk_origin(min_chunk);
	let mut points = Vec::new();
	tracy_client::plot!("planet lod step", step as f64);
	tracy_client::plot!(
		"planet lod source chunks",
		(size_chunks.x * size_chunks.y * size_chunks.z) as f64
	);
	append_planet_samples(
		tile,
		origin,
		extent,
		step,
		sample_offset,
		false,
		Some(cancellation),
		&mut points,
	);
	if cancellation.is_cancelled() { return None; }
	tracy_client::plot!("planet lod emitted voxels", points.len() as f64);
	points_to_voxels(points)
}

fn points_to_voxels(points: Vec<(U16Vec3, Voxel)>) -> Option<Voxels> {
	let _zone = span!("planet points to voxels");
	tracy_client::plot!("planet points to voxels input", points.len() as f64);
	if points.is_empty() {
		None
	} else {
		let mut voxels = Voxels::new();
		voxels.add_voxels(&points);
		Some(voxels)
	}
}

fn append_planet_samples(
	tile: &PlanetTile,
	origin: IVec3,
	extent: IVec3,
	step: i32,
	sample_offset: i32,
	full_mass: bool,
	cancellation: Option<&CancellationToken>,
	points: &mut Vec<(U16Vec3, Voxel)>,
) {
	let _zone = span!("planet append samples");
	let mass = if full_mass { 100 } else { 0 };
	let step_f = step as f32;
	let sample_base_z = origin.z as f32 + sample_offset as f32 + 0.5;

	let mut columns_tested = 0usize;
	let mut columns_in_shape = 0usize;
	let mut z_candidates = 0usize;
	let mut terrain_samples = 0usize;
	let start_points = points.len();

	tracy_client::plot!("planet sample extent x", extent.x as f64);
	tracy_client::plot!("planet sample extent y", extent.y as f64);
	tracy_client::plot!("planet sample extent z", extent.z as f64);

	for y in 0..extent.y {
		if cancellation.is_some_and(CancellationToken::is_cancelled) { return; }
		let sample_y = (origin.y + y * step + sample_offset) as f32 + 0.5;
		for x in 0..extent.x {
			columns_tested += 1;
			let sample_x = (origin.x + x * step + sample_offset) as f32 + 0.5;
			let Some((z0, z1)) =
				column_shape_z_range(tile, sample_x, sample_y, sample_base_z, extent.z, step_f)
			else {
				continue;
			};
			columns_in_shape += 1;
			z_candidates += (z1 - z0) as usize;

			// let lateral_len_sq = sample_x * sample_x + sample_y * sample_y;

			for z in z0..z1 {
				// let sample_z = sample_base_z + z as f32 * step_f;
				// let radial = PLANET_RADIUS + sample_z;
				// let radius = (lateral_len_sq + radial * radial).sqrt();
				// if radius <= 1e-5 {
				//     continue;
				// }

				// let unit = local_unit_to_planet(tile, sample_x, sample_y, radial, radius);
				// terrain_samples += 1;
				// let terrain = terrain_sample(unit);
				// let altitude = radius - PLANET_RADIUS;
				// if altitude > terrain.height {
				//     continue;
				// }

				points.push((
					IVec3::new(x, y, z).as_u16vec3(),
					Voxel {
						color: [
							200,
							100,
							30,
							255
						],
						mass,
					},
				));
			}
		}
	}

	tracy_client::plot!("planet columns tested", columns_tested as f64);
	tracy_client::plot!("planet columns in shape", columns_in_shape as f64);
	tracy_client::plot!("planet z candidates", z_candidates as f64);
	tracy_client::plot!("planet terrain samples", terrain_samples as f64);
	tracy_client::plot!(
		"planet append emitted voxels",
		(points.len() - start_points) as f64
	);
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

fn local_unit_to_planet(tile: &PlanetTile, x: f32, y: f32, radial: f32, radius: f32) -> Vec3 {
	(tile.axis_x * x + tile.axis_y * y + tile.normal * radial) / radius
}
