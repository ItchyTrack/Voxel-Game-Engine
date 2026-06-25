use bevy::math::{Mat3, Vec3};
use bevy::prelude::*;
use bevy::transform::components::Transform;

use crate::constraints::BallJoint;
use crate::math::{Mat6, Vec6};

use super::physics_constraint::{GAMMA, PhysicsConstraint};
use super::Solver;

#[derive(Component)]
pub(crate) struct AvbdBallJointConstraint {
	c0_linear: Vec3,
	c0_angular: Vec3,
	penalty_linear: Vec3,
	penalty_angular: Vec3,
	lambda_linear: Vec3,
	_lambda_angular: Vec3,
	body_1_attachment_com: Transform,
	body_2_attachment_com: Transform,
	stiffness_linear: f32,
	stiffness_angular: f32,
}

impl AvbdBallJointConstraint {
	pub(crate) fn from_ball_joint(joint: &BallJoint) -> Self {
		Self {
			c0_linear: Vec3::ZERO,
			c0_angular: Vec3::ZERO,
			penalty_linear: Vec3::ZERO,
			penalty_angular: Vec3::ZERO,
			lambda_linear: Vec3::ZERO,
			_lambda_angular: Vec3::ZERO,
			body_1_attachment_com: Transform::IDENTITY,
			body_2_attachment_com: Transform::IDENTITY,
			stiffness_linear: joint.stiffness_linear,
			stiffness_angular: joint.stiffness_angular,
		}
	}

	pub(crate) fn update_attachment_com(&mut self, joint: &BallJoint, body_1_com: &Vec3, body_2_com: &Vec3) {
		self.stiffness_linear = joint.stiffness_linear;
		self.stiffness_angular = joint.stiffness_angular;
		self.body_1_attachment_com = joint.body_1_attachment * Transform::from_translation(-*body_1_com);
		self.body_2_attachment_com = joint.body_2_attachment * Transform::from_translation(-*body_2_com);
	}
}

fn skew(r: &Vec3) -> Mat3 {
	Mat3::from_cols_array_2d(&[
		[0.0, -r.z, r.y],
		[r.z, 0.0, -r.x],
		[-r.y, r.x, 0.0],
	]).transpose()
}

impl PhysicsConstraint for AvbdBallJointConstraint {
	fn init(&mut self, initial_state_1: &Transform, initial_state_2: &Transform) {
		self.c0_linear = *initial_state_1 * self.body_1_attachment_com.translation - *initial_state_2 * self.body_2_attachment_com.translation;
		self.c0_angular = Solver::sub_quat(&initial_state_1.rotation, &initial_state_2.rotation);
		self.penalty_linear = (self.penalty_linear * GAMMA).clamp(Vec3::splat(1.0), Vec3::splat(10000000000.0)).clamp_length_max(self.stiffness_linear);
		self.penalty_angular = (self.penalty_angular * GAMMA).clamp(Vec3::splat(1.0), Vec3::splat(10000000000.0)).clamp_length_max(self.stiffness_angular);
	}

	fn get_updated(&self, state_1: &Transform, _initial_state_1: &Transform, state_2: &Transform, _initial_state_2: &Transform, alpha: f32, calc_1: bool) -> Option<(Vec6, Mat6)> {
		if self.stiffness_linear > 0.0 {
			let penalty_mat = Mat3::from_diagonal(self.penalty_linear);
			let mut c = *state_1 * self.body_1_attachment_com.translation - *state_2 * self.body_2_attachment_com.translation;
			if self.stiffness_linear.is_infinite() {
				c -= self.c0_linear * alpha;
			}
			let force: Vec3 = penalty_mat * c + self.lambda_linear;
			let d_prime_linear = if calc_1 { Mat3::IDENTITY } else { -Mat3::IDENTITY };
			let d_prime_angular = if calc_1 { skew(&-(state_1.rotation * self.body_1_attachment_com.translation)) } else { skew(&(state_2.rotation * self.body_2_attachment_com.translation)) };
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

	fn update_dual(&mut self, state_1: &Transform, _initial_state_1: &Transform, state_2: &Transform, _initial_state_2: &Transform, alpha: f32) {
		let penalty_mat = Mat3::from_diagonal(self.penalty_linear);
		let mut c = *state_1 * self.body_1_attachment_com.translation - *state_2 * self.body_2_attachment_com.translation;
		if self.stiffness_linear.is_infinite() {
			c -= self.c0_linear * alpha;
			let force: Vec3 = penalty_mat * c + self.lambda_linear;
			self.lambda_linear = force;
		}
		let beta = 5000000.0;
		self.penalty_linear = (self.penalty_linear + beta * c.abs()).clamp_length_max(self.stiffness_linear.min(10000000000.0));
	}
}
