use voxel_math::Fixed;
use bevy::prelude::*;
use voxel_transform::Transform;

#[derive(Component, Debug, Clone, Copy)]
pub struct BallJoint {
	pub body_1: Entity,
	pub body_2: Entity,
	pub body_1_attachment: Transform,
	pub body_2_attachment: Transform,
	/// Use `Fixed::MAX` for a hard constraint.
	pub stiffness_linear: Fixed,
	pub stiffness_angular: Fixed,
}

impl BallJoint {
	pub fn new(
		body_1: Entity,
		body_2: Entity,
		body_1_attachment: &Transform,
		body_2_attachment: &Transform,
		stiffness_linear: Fixed,
		stiffness_angular: Fixed,
	) -> Self {
		crate::math::require_unit_scale(body_1_attachment);
		crate::math::require_unit_scale(body_2_attachment);
		Self {
			body_1,
			body_2,
			body_1_attachment: *body_1_attachment,
			body_2_attachment: *body_2_attachment,
			stiffness_linear,
			stiffness_angular,
		}
	}
}
