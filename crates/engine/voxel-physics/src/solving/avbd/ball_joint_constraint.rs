use crate::math::{Mat3, Mat6, Vec6, Wide, WideVec3};
use voxel_math::{Fixed, FixedVec3};
use bevy::prelude::*;
use voxel_transform::Transform;

use crate::constraints::BallJoint;

use super::physics_constraint::{GAMMA, PhysicsConstraint};
use super::Solver;

#[cfg(test)]
#[path = "ball_joint_constraint_tests.rs"]
mod tests;

#[derive(Component)]
pub(crate) struct AvbdBallJointConstraint {
	c0_linear: WideVec3,
	c0_angular: WideVec3,
	penalty_linear: WideVec3,
	penalty_angular: WideVec3,
	lambda_linear: WideVec3,
	_lambda_angular: WideVec3,
	body_1_attachment_com: Transform,
	body_2_attachment_com: Transform,
	stiffness_linear: Fixed,
	stiffness_angular: Fixed,
}

impl AvbdBallJointConstraint {
	pub(crate) fn from_ball_joint(joint: &BallJoint) -> Self {
		Self {
			c0_linear: WideVec3::ZERO,
			c0_angular: WideVec3::ZERO,
			penalty_linear: WideVec3::ZERO,
			penalty_angular: WideVec3::ZERO,
			lambda_linear: WideVec3::ZERO,
			_lambda_angular: WideVec3::ZERO,
			body_1_attachment_com: Transform::IDENTITY,
			body_2_attachment_com: Transform::IDENTITY,
			stiffness_linear: joint.stiffness_linear,
			stiffness_angular: joint.stiffness_angular,
		}
	}

	pub(crate) fn update_attachment_com(&mut self, joint: &BallJoint, body_1_com: &FixedVec3, body_2_com: &FixedVec3) {
		crate::math::require_unit_scale(&joint.body_1_attachment);
		crate::math::require_unit_scale(&joint.body_2_attachment);
		self.stiffness_linear = joint.stiffness_linear;
		self.stiffness_angular = joint.stiffness_angular;
		self.body_1_attachment_com = Transform::from_translation(-*body_1_com) * joint.body_1_attachment;
		self.body_2_attachment_com = Transform::from_translation(-*body_2_com) * joint.body_2_attachment;
	}
}

fn skew(r: WideVec3) -> Mat3 {
	Mat3::from_cols_array_2d(&[
		[Wide::ZERO, -r.z, r.y],
		[r.z, Wide::ZERO, -r.x],
		[-r.y, r.x, Wide::ZERO],
	]).transpose()
}

fn clamp_stiffness(penalty: WideVec3, stiffness: Fixed) -> WideVec3 {
	if stiffness == Fixed::MAX { penalty } else { penalty.clamp_length_max(stiffness.into()) }
}

impl PhysicsConstraint for AvbdBallJointConstraint {
	fn init(&mut self, initial_state_1: &Transform, initial_state_2: &Transform) {
		self.c0_linear = (*initial_state_1 * self.body_1_attachment_com.translation - *initial_state_2 * self.body_2_attachment_com.translation).into();
		self.c0_angular = Solver::sub_quat(&initial_state_1.rotation, &initial_state_2.rotation).into();
		let max_penalty = WideVec3::splat(Wide::from_num(10_000_000_000u64));
		self.penalty_linear = clamp_stiffness((self.penalty_linear * GAMMA).clamp(WideVec3::ONE, max_penalty), self.stiffness_linear);
		self.penalty_angular = clamp_stiffness((self.penalty_angular * GAMMA).clamp(WideVec3::ONE, max_penalty), self.stiffness_angular);
	}

	fn get_updated(&self, state_1: &Transform, _initial_state_1: &Transform, state_2: &Transform, _initial_state_2: &Transform, alpha: Wide, calc_1: bool) -> Option<(Vec6, Mat6)> {
		if self.stiffness_linear > Fixed::ZERO {
			let penalty_mat = Mat3::from_diagonal(self.penalty_linear);
			let mut c = WideVec3::from(*state_1 * self.body_1_attachment_com.translation - *state_2 * self.body_2_attachment_com.translation);
			if self.stiffness_linear == Fixed::MAX {
				c -= self.c0_linear * alpha;
			}
			let force = penalty_mat * c + self.lambda_linear;
			let d_prime_linear = if calc_1 { Mat3::IDENTITY } else { -Mat3::IDENTITY };
			let d_prime_angular = if calc_1 { skew((-(state_1.rotation * self.body_1_attachment_com.translation)).into()) } else { skew((state_2.rotation * self.body_2_attachment_com.translation).into()) };
			let d_prime_linear_transpose_times_k = d_prime_linear.transpose() * penalty_mat;
			let d_prime_angular_transpose_times_k = d_prime_angular.transpose() * penalty_mat;
			return Some((
				Vec6::from_vec3(d_prime_linear.transpose() * force, d_prime_angular.transpose() * force),
				Mat6::from_mat3(
					d_prime_linear_transpose_times_k * d_prime_linear,
					d_prime_linear_transpose_times_k * d_prime_angular,
					d_prime_angular_transpose_times_k * d_prime_linear,
					d_prime_angular_transpose_times_k * d_prime_angular,
				)
			));
		}
		None
	}

	fn update_dual(&mut self, state_1: &Transform, _initial_state_1: &Transform, state_2: &Transform, _initial_state_2: &Transform, alpha: Wide) {
		let penalty_mat = Mat3::from_diagonal(self.penalty_linear);
		let mut c = WideVec3::from(*state_1 * self.body_1_attachment_com.translation - *state_2 * self.body_2_attachment_com.translation);
		if self.stiffness_linear == Fixed::MAX {
			c -= self.c0_linear * alpha;
			self.lambda_linear = penalty_mat * c + self.lambda_linear;
		}
		let beta = Wide::from_num(5_000_000);
		self.penalty_linear = (self.penalty_linear + beta * c.abs()).clamp_length_max(Wide::from(self.stiffness_linear).min(Wide::from_num(10_000_000_000u64)));
	}
}
