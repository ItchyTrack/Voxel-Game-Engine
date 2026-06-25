use bevy::math::{Mat3, Vec3};
use bevy::prelude::*;
use bevy::transform::components::Transform;

use crate::math::{Mat6, Vec6};
use crate::solving::avbd::physics_constraint::{GAMMA, PhysicsConstraint};
use crate::solving::avbd::Solver;

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

// TODO: remove as this is just a subset of BallJoint
#[derive(Component, Clone)]
pub struct BallJointConstraint {
	body_1_attachment: Transform,
	body_2_attachment: Transform,
	stiffness_linear: f32,
	stiffness_angular: f32,
}

#[derive(Component)]
pub struct AvbdBallJointConstraint {
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


impl BallJointConstraint {
	pub fn new(body_1_attachment: &Transform, body_2_attachment: &Transform, stiffness_linear: f32, stiffness_angular: f32) -> Self {
		Self {
			body_1_attachment: *body_1_attachment,
			body_2_attachment: *body_2_attachment,
			stiffness_linear,
			stiffness_angular,
		}
	}

	pub fn body_1_attachment(&self) -> &Transform {
		&self.body_1_attachment
	}

	pub fn body_2_attachment(&self) -> &Transform {
		&self.body_2_attachment
	}
}

pub fn ordered_pair(a: Entity, b: Entity) -> (Entity, Entity) {
	if a < b { (a, b) } else { (b, a) }
}

pub fn sync_ball_joint_constraints(
	mut commands: Commands,
	joints: Query<(Entity, &BallJoint, Option<&BallJointConstraint>, Option<&AvbdBallJointConstraint>)>,
) {
	for (entity, joint, runtime, avbd) in joints.iter() {
		if runtime.is_none() {
			commands.entity(entity).insert(BallJointConstraint::new(
				&joint.body_1_attachment,
				&joint.body_2_attachment,
				joint.stiffness_linear,
				joint.stiffness_angular,
			));
		}
		if avbd.is_none() {
			let runtime = BallJointConstraint::new(
				&joint.body_1_attachment,
				&joint.body_2_attachment,
				joint.stiffness_linear,
				joint.stiffness_angular,
			);
			commands.entity(entity).insert(AvbdBallJointConstraint::from_constraint(&runtime));
		}
	}
}

// TODO: make AvbdBallJointConstraint private to solving/avbd
impl AvbdBallJointConstraint {
	pub fn from_constraint(constraint: &BallJointConstraint) -> Self {
		Self {
			c0_linear: Vec3::ZERO,
			c0_angular: Vec3::ZERO,
			penalty_linear: Vec3::ZERO,
			penalty_angular: Vec3::ZERO,
			lambda_linear: Vec3::ZERO,
			_lambda_angular: Vec3::ZERO,
			body_1_attachment_com: Transform::IDENTITY,
			body_2_attachment_com: Transform::IDENTITY,
			stiffness_linear: constraint.stiffness_linear,
			stiffness_angular: constraint.stiffness_angular,
		}
	}

	pub fn update_attachment_com(&mut self, constraint: &BallJointConstraint, body_1_com: &Vec3, body_2_com: &Vec3) {
		self.stiffness_linear = constraint.stiffness_linear;
		self.stiffness_angular = constraint.stiffness_angular;
		self.body_1_attachment_com = *constraint.body_1_attachment() * Transform::from_translation(-*body_1_com);
		self.body_2_attachment_com = *constraint.body_2_attachment() * Transform::from_translation(-*body_2_com);
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
