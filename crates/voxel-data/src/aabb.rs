use bevy::math::{Vec3, Mat3};
use bevy::transform::components::Transform;

pub fn aabb_corners(lo: Vec3, hi: Vec3) -> [Vec3; 8] {
	[
		Vec3::new(lo.x, lo.y, lo.z),
		Vec3::new(hi.x, lo.y, lo.z),
		Vec3::new(lo.x, hi.y, lo.z),
		Vec3::new(lo.x, lo.y, hi.z),
		Vec3::new(hi.x, hi.y, lo.z),
		Vec3::new(hi.x, lo.y, hi.z),
		Vec3::new(lo.x, hi.y, hi.z),
		Vec3::new(hi.x, hi.y, hi.z),
	]
}

pub fn aabb_of_transformed_aabb(tf: &Transform, lo: Vec3, hi: Vec3) -> (Vec3, Vec3) {
	let center = (lo + hi) * 0.5;
	let half = (hi - lo) * 0.5;

	let new_center = *tf * center;
	let r = Mat3::from_quat(tf.rotation);
	let scaled_half = half * tf.scale.abs();

	let new_half = r.x_axis.abs() * scaled_half.x + r.y_axis.abs() * scaled_half.y + r.z_axis.abs() * scaled_half.z;

	(new_center - new_half, new_center + new_half)
}
