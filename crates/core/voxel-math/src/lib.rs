mod fixed;
mod vector;
#[cfg(test)]
mod tests;

pub use fixed::Fixed;
pub use vector::FixedVec3;
use bevy::math::Quat;

/// A spatial transform with fixed-point position and uniform scale.
pub trait Transform {
	fn translation(&self) -> FixedVec3;
	fn rotation(&self) -> Quat;
	fn scale(&self) -> Fixed { Fixed::ONE }
	fn direction(&self) -> FixedVec3 { self.rotation() * FixedVec3::Z }
	fn transform_point(&self, point: FixedVec3) -> FixedVec3 { self.translation() + self.rotation() * (point * self.scale()) }
	fn transform_vector(&self, vector: FixedVec3) -> FixedVec3 { self.rotation() * (vector * self.scale()) }
	fn inverse_transform_point(&self, point: FixedVec3) -> FixedVec3 { (self.rotation().inverse() * (point - self.translation())) / self.scale() }
	fn inverse_transform_vector(&self, vector: FixedVec3) -> FixedVec3 { (self.rotation().inverse() * vector) / self.scale() }
}

/// Ray distances are parameters along `direction`; use a unit direction for world distances.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct Ray {
	pub origin: FixedVec3,
	pub direction: FixedVec3,
}

impl Transform for Ray {
	fn translation(&self) -> FixedVec3 { self.origin }
	fn rotation(&self) -> Quat { Quat::IDENTITY }
	fn direction(&self) -> FixedVec3 { self.direction }
}

/// Intersect a forward ray with a closed box. Parallel axes never divide by zero.
pub fn ray_aabb_intersection(start: &FixedVec3, direction: &FixedVec3, aabb: &(FixedVec3, FixedVec3)) -> Option<Fixed> {
	let (min, max) = *aabb;
	if min.cmpgt(max).any() { return None; }
	let mut entry = 0i128;
	let mut exit = i64::MAX as i128;
	for axis in 0..3 {
		if direction[axis].is_zero() {
			if start[axis] < min[axis] || start[axis] > max[axis] { return None; }
			continue;
		}
		let plane_time = |plane: Fixed| {
			fixed::rounded_div((plane.to_bits() as i128 - start[axis].to_bits() as i128) * fixed::SCALE, direction[axis].to_bits() as i128)
		};
		let a = plane_time(min[axis]);
		let b = plane_time(max[axis]);
		entry = entry.max(a.min(b));
		exit = exit.min(a.max(b));
		if entry > exit { return None; }
	}
	Some(Fixed::from_bits(i64::try_from(entry).ok()?))
}
