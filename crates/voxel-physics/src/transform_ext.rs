use bevy::transform::components::Transform;
use glam::Vec3;

/// Rigid-body inverse for [`Transform`]: assumes uniform scale of 1.0.
///
/// Bevy's [`Transform`] has no built-in inverse because the general case
/// (with non-uniform scale) doesn't round-trip. Physics never scales bodies,
/// so this trait gives us the clean SE(3) inverse the solver needs.
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
