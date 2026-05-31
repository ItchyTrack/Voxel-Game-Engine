use bevy::transform::components::Transform;
use bevy::math::Vec3;

/// Rigid-body inverse for [`Transform`]: assumes uniform scale of 1.0.
pub trait TransformExt {
	fn inverse(&self) -> Transform;
}

impl TransformExt for Transform {
	fn inverse(&self) -> Transform {
		let rotation = self.rotation.inverse();
		Transform {
			translation: rotation * -self.translation,
			rotation,
			scale: Vec3::ONE,
		}
	}
}
