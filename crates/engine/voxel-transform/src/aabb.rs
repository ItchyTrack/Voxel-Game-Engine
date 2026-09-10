use voxel_math::{FixedVec3, Transform};

pub fn aabb_corners(lo: FixedVec3, hi: FixedVec3) -> [FixedVec3; 8] {
	[
		FixedVec3::new(lo.x, lo.y, lo.z),
		FixedVec3::new(hi.x, lo.y, lo.z),
		FixedVec3::new(lo.x, hi.y, lo.z),
		FixedVec3::new(lo.x, lo.y, hi.z),
		FixedVec3::new(hi.x, hi.y, lo.z),
		FixedVec3::new(hi.x, lo.y, hi.z),
		FixedVec3::new(lo.x, hi.y, hi.z),
		FixedVec3::new(hi.x, hi.y, hi.z),
	]
}

pub fn aabb_of_transformed_aabb(tf: &(impl Transform + ?Sized), lo: FixedVec3, hi: FixedVec3) -> (FixedVec3, FixedVec3) {
	aabb_corners(lo, hi).into_iter().map(|corner| tf.transform_point(corner))
		.fold((FixedVec3::MAX, FixedVec3::MIN), |(min, max), point| (min.min(point), max.max(point)))
}
