use std::sync::OnceLock;

use bevy::prelude::*;
use voxel_math::{Fixed, FixedVec3};
use crate::voxel::fixed_math::{FixedVec2, PI, ShapeMath};
use tracy_client::span;
use tile_data::{chunk_of, chunk_origin};
use tile_data::CHUNK_SIZE;

use super::config::{
	PLANET_RADIUS, PLANET_TILE_COUNT, TILE_BOUND_PADDING, TILE_INWARD_DEPTH, TILE_OUTWARD_HEIGHT,
	TILE_SHAPE_EPSILON, VORONOI_NEIGHBORS,
};

#[derive(Debug, Clone)]
pub(super) struct Halfspace {
	// Local tile coordinates are inside when normal.dot(local) + offset >= 0.
	pub(super) normal: FixedVec3,
	pub(super) offset: Fixed,
}

#[derive(Debug, Clone)]
pub(super) struct PlanetTile {
	pub(super) index: usize,
	pub(super) normal: FixedVec3,
	pub(super) origin: FixedVec3,
	pub(super) axis_x: FixedVec3,
	pub(super) axis_y: FixedVec3,
	pub(super) halfspaces: Vec<Halfspace>,
	pub(super) present_chunks: Vec<IVec3>,
}

pub(super) fn planet_tiles() -> &'static [PlanetTile] {
	static TILES: OnceLock<Vec<PlanetTile>> = OnceLock::new();
	TILES.get_or_init(build_planet_tiles).as_slice()
}

fn build_planet_tiles() -> Vec<PlanetTile> {
	let _zone = span!("planet build tile cache");
	let normals: Vec<FixedVec3> = (0..PLANET_TILE_COUNT)
		.map(|index| fibonacci_sphere_point(index, PLANET_TILE_COUNT))
		.collect();

	let mut tiles = Vec::with_capacity(PLANET_TILE_COUNT);
	for (index, &normal) in normals.iter().enumerate() {
		let axis_x = if normal.x.abs() < Fixed::from_num(1e-6) && normal.z.abs() < Fixed::from_num(1e-6) {
			FixedVec3::X
		} else {
			FixedVec3::new(-normal.z, Fixed::ZERO, normal.x).normalize()
		};
		let axis_y = normal.cross(axis_x).normalize();

		let mut neighbor_dots: Vec<(usize, Fixed)> = normals
			.iter()
			.enumerate()
			.filter(|&(other, _)| other != index)
			.map(|(other, &other_normal)| (other, normal.dot(other_normal)))
			.collect();
		if neighbor_dots.len() > VORONOI_NEIGHBORS {
			neighbor_dots.select_nth_unstable_by(VORONOI_NEIGHBORS, |a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
			neighbor_dots.truncate(VORONOI_NEIGHBORS);
		}
		neighbor_dots.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));

		// A spherical Voronoi edge between tile A and tile B is the plane where
		// dot(A, point_dir) == dot(B, point_dir). In a tile's local tangent
		// coordinates this is just a linear halfspace, so we can cache the real
		// convex cell once and use it for spawn presence and voxel ownership. The first ~32 neighbors are plenty for a Fibonacci sphere;
		// farther sites cannot cut this local cell.
		let halfspaces: Vec<_> = neighbor_dots
			.iter()
			.take(VORONOI_NEIGHBORS)
			.map(|&(other, _)| voronoi_halfspace(normal, normals[other], axis_x, axis_y))
			.collect();
		let present_chunks = build_present_chunks(&halfspaces);
		let active_halfspaces = remove_redundant_halfspaces(&halfspaces);

		tiles.push(PlanetTile {
			index,
			normal,
			origin: normal * PLANET_RADIUS,
			axis_x,
			axis_y,
			halfspaces: active_halfspaces,
			present_chunks,
		});
	}

	tiles
}

fn fibonacci_sphere_point(index: usize, count: usize) -> FixedVec3 {
	let i = Fixed::from_num(index) + Fixed::from_num(0.5);
	let n = Fixed::from_num(count);
	let y = Fixed::ONE - Fixed::from_num(2) * i / n;
	let h = PI * (Fixed::ONE + Fixed::from_num(5).sqrt()) * i;
	let radius = (Fixed::ONE - y * y).max(Fixed::ZERO).sqrt();
	FixedVec3::new(h.cos() * radius, y, h.sin() * radius).normalize()
}

fn voronoi_halfspace(
	tile_normal: FixedVec3,
	neighbor_normal: FixedVec3,
	axis_x: FixedVec3,
	axis_y: FixedVec3,
) -> Halfspace {
	let diff = tile_normal - neighbor_normal;
	Halfspace {
		normal: FixedVec3::new(diff.dot(axis_x), diff.dot(axis_y), diff.dot(tile_normal)),
		offset: diff.dot(tile_normal * PLANET_RADIUS),
	}
}

fn remove_redundant_halfspaces(halfspaces: &[Halfspace]) -> Vec<Halfspace> {
	let active: Vec<_> = halfspaces
		.iter()
		.enumerate()
		.filter(|&(candidate, _)| halfspace_is_active(candidate, halfspaces))
		.map(|(_, halfspace)| halfspace.clone())
		.collect();

	// A valid Voronoi tile always has active boundaries. Retaining the original
	// set is a defensive fallback for malformed or numerically unstable input.
	if active.is_empty() { halfspaces.to_vec() } else { active }
}

fn halfspace_is_active(candidate: usize, halfspaces: &[Halfspace]) -> bool {
	let candidate_plane = halfspace_z_plane(&halfspaces[candidate]);
	let extent = PLANET_RADIUS;
	let mut polygon = vec![
		FixedVec2::new(-extent, -extent),
		FixedVec2::new(extent, -extent),
		FixedVec2::new(extent, extent),
		FixedVec2::new(-extent, extent),
	];

	// Ignore boundaries which can only be exposed outside the generated Z slab.
	clip_affine_polygon(
		&mut polygon,
		candidate_plane.0,
		candidate_plane.1 + Fixed::from_num(TILE_INWARD_DEPTH),
	);
	clip_affine_polygon(
		&mut polygon,
		-candidate_plane.0,
		Fixed::from_num(TILE_OUTWARD_HEIGHT) - candidate_plane.1,
	);

	// The candidate is active wherever its lower Z bound is at least every
	// other bound. If that region is empty, it can never affect a column.
	for (other_index, other) in halfspaces.iter().enumerate() {
		if other_index == candidate || polygon.is_empty() {
			continue;
		}
		let other_plane = halfspace_z_plane(other);
		clip_affine_polygon(
			&mut polygon,
			candidate_plane.0 - other_plane.0,
			candidate_plane.1 - other_plane.1,
		);
	}

	!polygon.is_empty()
}

fn halfspace_z_plane(halfspace: &Halfspace) -> (FixedVec2, Fixed) {
	debug_assert!(halfspace.normal.z > Fixed::from_num(1e-6));
	let inverse_z = halfspace.normal.z.recip();
	(
		FixedVec2::new(
			-halfspace.normal.x * inverse_z,
			-halfspace.normal.y * inverse_z,
		),
		(-TILE_SHAPE_EPSILON - halfspace.offset) * inverse_z,
	)
}

fn clip_affine_polygon(polygon: &mut Vec<FixedVec2>, normal: FixedVec2, offset: Fixed) {
	if polygon.is_empty() {
		return;
	}

	let mut clipped = Vec::with_capacity(polygon.len() + 1);
	for index in 0..polygon.len() {
		let a = polygon[index];
		let b = polygon[(index + 1) % polygon.len()];
		let a_value = normal.dot(a) + offset;
		let b_value = normal.dot(b) + offset;
		let a_inside = a_value >= Fixed::ZERO;
		let b_inside = b_value >= Fixed::ZERO;

		if a_inside && b_inside {
			clipped.push(b);
		} else if a_inside != b_inside {
			let t = a_value / (a_value - b_value);
			clipped.push(a.lerp(b, t));
			if b_inside {
				clipped.push(b);
			}
		}
	}
	*polygon = clipped;
}

fn build_present_chunks(halfspaces: &[Halfspace]) -> Vec<IVec3> {
	let (min_xy, max_xy) = voronoi_xy_bounds(halfspaces);
	let min_voxel = IVec3::new(
		min_xy.x.floor().to_num::<i32>() - TILE_BOUND_PADDING,
		min_xy.y.floor().to_num::<i32>() - TILE_BOUND_PADDING,
		-TILE_INWARD_DEPTH,
	);
	let max_voxel_exclusive = IVec3::new(
		max_xy.x.ceil().to_num::<i32>() + TILE_BOUND_PADDING,
		max_xy.y.ceil().to_num::<i32>() + TILE_BOUND_PADDING,
		TILE_OUTWARD_HEIGHT,
	);

	let min_chunk = chunk_of(min_voxel);
	let max_chunk = chunk_of(max_voxel_exclusive - IVec3::ONE);
	let mut chunks = Vec::new();
	for x in min_chunk.x..=max_chunk.x {
		for y in min_chunk.y..=max_chunk.y {
			for z in min_chunk.z..=max_chunk.z {
				let chunk = IVec3::new(x, y, z);
				if chunk_intersects_tile_shape(halfspaces, chunk) {
					chunks.push(chunk);
				}
			}
		}
	}
	chunks.sort_by_key(|c| (c.x, c.y, c.z));
	chunks.dedup();
	chunks
}

fn voronoi_xy_bounds(halfspaces: &[Halfspace]) -> (FixedVec2, FixedVec2) {
	let mut min = FixedVec2::splat(Fixed::MAX);
	let mut max = FixedVec2::splat(Fixed::MIN);
	for z in [Fixed::from_num(-TILE_INWARD_DEPTH), Fixed::from_num(TILE_OUTWARD_HEIGHT)] {
		let polygon = clipped_voronoi_polygon(halfspaces, z);
		for p in polygon {
			min = min.min(p);
			max = max.max(p);
		}
	}

	if min == FixedVec2::splat(Fixed::MAX) || max == FixedVec2::splat(Fixed::MIN) {
		// Extremely defensive fallback; this should never happen unless the
		// neighbor list is broken.
		(FixedVec2::splat(Fixed::from_num(-512)), FixedVec2::splat(Fixed::from_num(512)))
	} else {
		(min, max)
	}
}

fn clipped_voronoi_polygon(halfspaces: &[Halfspace], z: Fixed) -> Vec<FixedVec2> {
	let extent = PLANET_RADIUS * Fixed::from_num(0.25);
	let mut polygon = vec![
		FixedVec2::new(-extent, -extent),
		FixedVec2::new(extent, -extent),
		FixedVec2::new(extent, extent),
		FixedVec2::new(-extent, extent),
	];

	for halfspace in halfspaces {
		if polygon.is_empty() {
			break;
		}
		let mut clipped = Vec::new();
		let z_offset = halfspace.normal.z * z + halfspace.offset;
		for i in 0..polygon.len() {
			let a = polygon[i];
			let b = polygon[(i + 1) % polygon.len()];
			let va = halfspace.normal.x * a.x + halfspace.normal.y * a.y + z_offset;
			let vb = halfspace.normal.x * b.x + halfspace.normal.y * b.y + z_offset;
			let a_inside = va >= -TILE_SHAPE_EPSILON;
			let b_inside = vb >= -TILE_SHAPE_EPSILON;

			if a_inside && b_inside {
				clipped.push(b);
			} else if a_inside != b_inside {
				let t = (va / (va - vb)).clamp(Fixed::ZERO, Fixed::ONE);
				clipped.push(a.lerp(b, t));
				if b_inside {
					clipped.push(b);
				}
			}
		}
		polygon = clipped;
	}
	polygon
}

pub(super) fn tile_has_chunk(tile: &PlanetTile, chunk: IVec3) -> bool {
	tile.present_chunks
		.binary_search_by_key(&(chunk.x, chunk.y, chunk.z), |c| (c.x, c.y, c.z))
		.is_ok()
}

fn chunk_intersects_tile_shape(halfspaces: &[Halfspace], chunk: IVec3) -> bool {
	let min = FixedVec3::from(chunk_origin(chunk));
	let max = FixedVec3::from(chunk_origin(chunk) + IVec3::splat(CHUNK_SIZE as i32));
	if max.z <= Fixed::from_num(-TILE_INWARD_DEPTH) || min.z >= Fixed::from_num(TILE_OUTWARD_HEIGHT) {
		return false;
	}

	halfspaces.iter().all(|h| {
		// If the furthest AABB vertex in a halfspace's direction is still
		// outside, the whole chunk is outside. This is conservative, so it may
		// keep a few edge chunks but will never crop a valid Voronoi cell.
		let p = FixedVec3::new(
			if h.normal.x >= Fixed::ZERO { max.x } else { min.x },
			if h.normal.y >= Fixed::ZERO { max.y } else { min.y },
			if h.normal.z >= Fixed::ZERO { max.z } else { min.z },
		);
		h.normal.dot(p) + h.offset >= -Fixed::from_num(CHUNK_SIZE)
	})
}
