use bevy::math::{IVec2, IVec3};
use voxel_math::{Fixed, FixedVec3};

use crate::grid_tree::NonZeroVoxelRegion;

pub trait Sdf: Send + Sync {
	fn sample(&self, pos: FixedVec3) -> Fixed;
}

impl<F> Sdf for F
where
	F: Fn(FixedVec3) -> Fixed + Send + Sync,
{
	fn sample(&self, pos: FixedVec3) -> Fixed { self(pos) }
}

pub fn voxel_center(pos: IVec3) -> FixedVec3 {
	FixedVec3::from(pos) + FixedVec3::splat(Fixed::from_num(0.5))
}

pub fn voxel_region_from_bounds(min: FixedVec3, max: FixedVec3) -> Option<NonZeroVoxelRegion> {
	if !min.cmplt(max).all() { return None; }
	let voxel_min = min.floor().as_ivec3();
	let voxel_end = max.ceil().as_ivec3();
	NonZeroVoxelRegion::from_min_end(voxel_min, voxel_end)
}

pub fn shrink_aabb_with_sdf(
	mut min: FixedVec3,
	mut max: FixedVec3,
	sdf: &(impl Sdf + ?Sized),
	face_resolution: IVec2,
	iterations: usize,
) -> (FixedVec3, FixedVec3) {
	if !min.cmplt(max).all() { return (min, max); }
	let face_resolution = face_resolution.max(IVec2::ONE);
	for _ in 0..iterations {
		let next_min = FixedVec3::new(
			contract_face(min, max, sdf, 0, false, face_resolution),
			contract_face(min, max, sdf, 1, false, face_resolution),
			contract_face(min, max, sdf, 2, false, face_resolution),
		);
		let next_max = FixedVec3::new(
			contract_face(min, max, sdf, 0, true, face_resolution),
			contract_face(min, max, sdf, 1, true, face_resolution),
			contract_face(min, max, sdf, 2, true, face_resolution),
		);
		if !next_min.cmplt(next_max).all() { break; }
		if next_min.abs_diff_eq(min, Fixed::from_num(0.0001)) && next_max.abs_diff_eq(max, Fixed::from_num(0.0001)) { break; }
		min = next_min;
		max = next_max;
	}
	(min, max)
}

fn contract_face(min: FixedVec3, max: FixedVec3, sdf: &(impl Sdf + ?Sized), axis: usize, is_max_face: bool, resolution: IVec2) -> Fixed {
	let face_coord = if is_max_face { max[axis] } else { min[axis] };
	let mut min_positive = Fixed::MAX;
	for iy in 0..resolution.y {
		for ix in 0..resolution.x {
			let mut sample = FixedVec3::ZERO;
			sample[axis] = face_coord;
			let (a0, a1) = other_axes(axis);
			sample[a0] = lerp_axis(min[a0], max[a0], ix, resolution.x);
			sample[a1] = lerp_axis(min[a1], max[a1], iy, resolution.y);
			let d = sdf.sample(sample);
			if d <= Fixed::ZERO { return face_coord; }
			min_positive = min_positive.min(d);
		}
	}
	if is_max_face {
		face_coord - min_positive.min(face_coord - min[axis])
	} else {
		face_coord + min_positive.min(max[axis] - face_coord)
	}
}

fn other_axes(axis: usize) -> (usize, usize) {
	match axis {
		0 => (1, 2),
		1 => (0, 2),
		2 => (0, 1),
		_ => unreachable!(),
	}
}

fn lerp_axis(min: Fixed, max: Fixed, index: i32, count: i32) -> Fixed {
	if count <= 1 { return min + (max - min) * Fixed::from_num(0.5); }
	let t = Fixed::from_num(index) / Fixed::from_num(count - 1);
	min + (max - min) * t
}

#[cfg(test)]
mod tests {
	use super::*;

	fn sphere_sdf(center: FixedVec3, radius: Fixed) -> impl Fn(FixedVec3) -> Fixed {
		move |p: FixedVec3| (p - center).length() - radius
	}

	#[test]
	fn shrink_aabb_with_sdf_reduces_large_box_around_small_sdf() {
		let sdf = sphere_sdf(FixedVec3::splat(Fixed::from_num(8)), Fixed::from_num(2));
		let initial_min = FixedVec3::splat(Fixed::from_num(-100));
		let initial_max = FixedVec3::splat(Fixed::from_num(100));
		let (min, max) = shrink_aabb_with_sdf(initial_min, initial_max, &sdf, IVec2::splat(9), 8);
		assert!(min.cmpgt(FixedVec3::splat(Fixed::from_num(-50))).all(), "min was {min:?}");
		assert!(max.cmplt(FixedVec3::splat(Fixed::from_num(50))).all(), "max was {max:?}");
	}

	#[test]
	fn shrink_aabb_with_sdf_keeps_exact_bounds_unchanged() {
		let sdf = sphere_sdf(FixedVec3::splat(Fixed::from_num(8)), Fixed::from_num(5));
		let initial_min = FixedVec3::splat(Fixed::from_num(3));
		let initial_max = FixedVec3::splat(Fixed::from_num(13));
		let (min, max) = shrink_aabb_with_sdf(initial_min, initial_max, &sdf, IVec2::splat(9), 6);
		assert!(min.abs_diff_eq(initial_min, Fixed::from_num(0.001)), "min was {min:?}");
		assert!(max.abs_diff_eq(initial_max, Fixed::from_num(0.001)), "max was {max:?}");
	}

	#[test]
	fn shrink_aabb_with_sdf_keeps_too_small_box_unchanged() {
		let sdf = sphere_sdf(FixedVec3::splat(Fixed::from_num(8)), Fixed::from_num(5));
		let initial_min = FixedVec3::splat(Fixed::from_num(6));
		let initial_max = FixedVec3::splat(Fixed::from_num(10));
		let (min, max) = shrink_aabb_with_sdf(initial_min, initial_max, &sdf, IVec2::splat(9), 6);
		assert!(min.abs_diff_eq(initial_min, Fixed::from_num(0.001)), "min was {min:?}");
		assert!(max.abs_diff_eq(initial_max, Fixed::from_num(0.001)), "max was {max:?}");
	}

	#[test]
	fn shrink_aabb_with_sdf_still_shrinks_non_touching_faces_when_shape_hits_one_side() {
		let sdf = sphere_sdf(FixedVec3::new(Fixed::ZERO, Fixed::from_num(8), Fixed::from_num(8)), Fixed::from_num(5));
		let initial_min = FixedVec3::ZERO;
		let initial_max = FixedVec3::splat(Fixed::from_num(32));
		let (min, max) = shrink_aabb_with_sdf(initial_min, initial_max, &sdf, IVec2::splat(9), 8);
		assert!(min.x.abs() < Fixed::from_num(0.001), "min was {min:?}");
		assert!(min.y > Fixed::ZERO && min.z > Fixed::ZERO, "min was {min:?}");
		assert!(max.cmplt(initial_max).all(), "max was {max:?}");
	}
}
