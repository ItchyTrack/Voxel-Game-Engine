use bevy::math::Vec3;
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

pub fn transform_aabb(tf: &Transform, lo: Vec3, hi: Vec3) -> (Vec3, Vec3) {
	let lo_transformed = *tf * lo;
	let transform_vec = |v: Vec3| tf.rotation * (tf.scale * v);
	let vec_x_transformed = transform_vec(Vec3::new(hi.x - lo.x, 0.0, 0.0));
	let vec_y_transformed = transform_vec(Vec3::new(0.0, hi.y - lo.y, 0.0));
	let vec_z_transformed = transform_vec(Vec3::new(0.0, 0.0, hi.z - lo.z));
	let mut min = lo_transformed;
	let mut max = lo_transformed;

	for v in [
		lo_transformed + vec_x_transformed,
		lo_transformed + vec_y_transformed,
		lo_transformed + vec_z_transformed,
		lo_transformed + vec_x_transformed + vec_y_transformed,
		lo_transformed + vec_x_transformed + vec_z_transformed,
		lo_transformed + vec_y_transformed + vec_z_transformed,
		lo_transformed + vec_x_transformed + vec_y_transformed + vec_z_transformed,
	] {
		min = min.min(v);
		max = max.max(v);
	}
	(min, max)
}
