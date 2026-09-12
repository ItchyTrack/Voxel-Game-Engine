use bevy::prelude::*;
use bevy::transform::components::Transform;

#[derive(Component, Debug, Clone, Copy)]
pub struct BallJoint {
	pub body_1: Entity,
	pub body_2: Entity,
	pub body_1_attachment: Transform,
	pub body_2_attachment: Transform,
	pub stiffness_linear: f32,
	pub stiffness_angular: f32,
}

impl BallJoint {
	pub fn new(
		body_1: Entity,
		body_2: Entity,
		body_1_attachment: &Transform,
		body_2_attachment: &Transform,
		stiffness_linear: f32,
		stiffness_angular: f32,
	) -> Self {
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
