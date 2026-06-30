use bevy::math::{IVec2, IVec3, Vec3};

use crate::grid_tree::GridRegion;

pub trait Sdf: Send + Sync {
	fn sample(&self, pos: Vec3) -> f32;
}

impl<F> Sdf for F
where
	F: Fn(Vec3) -> f32 + Send + Sync,
{
	fn sample(&self, pos: Vec3) -> f32 {
		self(pos)
	}
}

pub fn voxel_center(pos: IVec3) -> Vec3 {
	pos.as_vec3() + Vec3::splat(0.5)
}

pub fn voxel_region_from_bounds(min: Vec3, max: Vec3) -> Option<GridRegion> {
	if !min.cmplt(max).all() {
		return None;
	}
	let voxel_min = min.floor().as_ivec3();
	let voxel_end = max.ceil().as_ivec3();
	GridRegion::new(voxel_min, voxel_end)
}

pub fn shrink_aabb_with_sdf(
	mut min: Vec3,
	mut max: Vec3,
	sdf: &(impl Sdf + ?Sized),
	face_resolution: IVec2,
	iterations: usize,
) -> (Vec3, Vec3) {
	if !(min.cmplt(max).all()) {
		return (min, max);
	}
	let face_resolution = face_resolution.max(IVec2::ONE);
	for _ in 0..iterations {
		let next_min = Vec3::new(
			contract_face(min, max, sdf, 0, false, face_resolution),
			contract_face(min, max, sdf, 1, false, face_resolution),
			contract_face(min, max, sdf, 2, false, face_resolution),
		);
		let next_max = Vec3::new(
			contract_face(min, max, sdf, 0, true, face_resolution),
			contract_face(min, max, sdf, 1, true, face_resolution),
			contract_face(min, max, sdf, 2, true, face_resolution),
		);
		if !next_min.cmplt(next_max).all() {
			break;
		}
		if next_min.abs_diff_eq(min, 0.0001) && next_max.abs_diff_eq(max, 0.0001) {
			break;
		}
		min = next_min;
		max = next_max;
	}
	(min, max)
}

fn contract_face(min: Vec3, max: Vec3, sdf: &(impl Sdf + ?Sized), axis: usize, is_max_face: bool, resolution: IVec2) -> f32 {
	let face_coord = if is_max_face { max[axis] } else { min[axis] };
	let mut min_positive = f32::INFINITY;
	let mut hit_inside = false;
	for iy in 0..resolution.y {
		for ix in 0..resolution.x {
			let mut sample = Vec3::ZERO;
			sample[axis] = face_coord;
			let (a0, a1) = other_axes(axis);
			sample[a0] = lerp_axis(min[a0], max[a0], ix, resolution.x);
			sample[a1] = lerp_axis(min[a1], max[a1], iy, resolution.y);
			let d = sdf.sample(sample);
			if d <= 0.0 {
				hit_inside = true;
				min_positive = 0.0;
				break;
			}
			min_positive = min_positive.min(d);
		}
		if hit_inside {
			break;
		}
	}
	if !min_positive.is_finite() {
		return face_coord;
	}
	if is_max_face {
		(face_coord - min_positive).max(min[axis])
	} else {
		(face_coord + min_positive).min(max[axis])
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

fn lerp_axis(min: f32, max: f32, index: i32, count: i32) -> f32 {
	if count <= 1 {
		return (min + max) * 0.5;
	}
	let t = index as f32 / (count - 1) as f32;
	min + (max - min) * t
}

#[cfg(test)]
mod tests {
	use super::*;

	fn sphere_sdf(center: Vec3, radius: f32) -> impl Fn(Vec3) -> f32 {
		move |p: Vec3| (p - center).length() - radius
	}

	#[test]
	fn shrink_aabb_with_sdf_reduces_large_box_around_small_sdf() {
		let sdf = sphere_sdf(Vec3::splat(8.0), 2.0);
		let initial_min = Vec3::splat(-100.0);
		let initial_max = Vec3::splat(100.0);
		let (min, max) = shrink_aabb_with_sdf(initial_min, initial_max, &sdf, IVec2::splat(9), 8);
		assert!(min.x > -50.0 && min.y > -50.0 && min.z > -50.0, "min was {min:?}");
		assert!(max.x < 50.0 && max.y < 50.0 && max.z < 50.0, "max was {max:?}");
	}

	#[test]
	fn shrink_aabb_with_sdf_keeps_exact_bounds_unchanged() {
		let sdf = sphere_sdf(Vec3::splat(8.0), 5.0);
		let initial_min = Vec3::splat(3.0);
		let initial_max = Vec3::splat(13.0);
		let (min, max) = shrink_aabb_with_sdf(initial_min, initial_max, &sdf, IVec2::splat(9), 6);
		assert!(min.abs_diff_eq(initial_min, 0.001), "min was {min:?}");
		assert!(max.abs_diff_eq(initial_max, 0.001), "max was {max:?}");
	}

	#[test]
	fn shrink_aabb_with_sdf_keeps_too_small_box_unchanged() {
		let sdf = sphere_sdf(Vec3::splat(8.0), 5.0);
		let initial_min = Vec3::splat(6.0);
		let initial_max = Vec3::splat(10.0);
		let (min, max) = shrink_aabb_with_sdf(initial_min, initial_max, &sdf, IVec2::splat(9), 6);
		assert!(min.abs_diff_eq(initial_min, 0.001), "min was {min:?}");
		assert!(max.abs_diff_eq(initial_max, 0.001), "max was {max:?}");
	}

	#[test]
	fn shrink_aabb_with_sdf_still_shrinks_non_touching_faces_when_shape_hits_one_side() {
		let sdf = sphere_sdf(Vec3::new(0.0, 8.0, 8.0), 5.0);
		let initial_min = Vec3::ZERO;
		let initial_max = Vec3::splat(32.0);
		let (min, max) = shrink_aabb_with_sdf(initial_min, initial_max, &sdf, IVec2::splat(9), 8);
		assert!(min.x.abs() < 0.001, "min was {min:?}");
		assert!(min.y > 0.0 && min.z > 0.0, "min was {min:?}");
		assert!(max.x < initial_max.x && max.y < initial_max.y && max.z < initial_max.z, "max was {max:?}");
	}
}
