use glam::{Mat3, Vec3};

use crate::{math::{Mat6, Vec6}, pose::Pose};

use super::{physics_constraint::{PhysicsConstraint, GAMMA}, solver::Solver};

pub struct BallJointConstraint {
	c0_linear: Vec3,
	c0_angular: Vec3,
	penalty_linear: Vec3,
	penalty_angular: Vec3,
	lambda_linear: Vec3,
	lambda_angular: Vec3,
	body_1_attachment: Pose,
	body_2_attachment: Pose,
}

impl BallJointConstraint {
	pub fn new(body_1_attachment: &Pose, body_2_attachment: &Pose) -> Self {
		Self {
			c0_linear: Vec3::ZERO,
			c0_angular: Vec3::ZERO,
			penalty_linear: Vec3::ZERO,
			penalty_angular: Vec3::ZERO,
			lambda_linear: Vec3::ZERO,
			lambda_angular: Vec3::ZERO,
			body_1_attachment: *body_1_attachment,
			body_2_attachment: *body_2_attachment,
		}
	}
}


fn skew(r: &Vec3) -> Mat3 {
	Mat3::from_cols_array_2d(&[
		[0.0, -r.z, r.y],
		[r.z, 0.0, -r.x],
		[-r.y, r.x, 0.0],
	]).transpose()
}

impl PhysicsConstraint for BallJointConstraint {
	fn init(&mut self, initial_state_1: &Pose, initial_state_2: &Pose) {
		// let normal = (self.collision.collision2 - self.collision.collision1).normalize();
		// if normal.is_nan() { return; }

		// let basis_pair = normal.any_orthonormal_pair();
		// self.basis = Mat3::from_cols(
		// 	normal,
		// 	basis_pair.0,
		// 	basis_pair.1
		// ).transpose();

		// self.c0 = self.basis * (self.collision.collision1 - self.collision.collision2) + Vec3::new(0.01, 0.0, 0.0);


		self.c0_linear = initial_state_1 * self.body_1_attachment.translation - initial_state_2 * self.body_2_attachment.translation;
		self.c0_angular = Solver::sub_quat(&initial_state_1.rotation, &initial_state_2.rotation)/* * torqueArm*/; // TODO: what is torqueArm

		self.penalty_linear = (self.penalty_linear * GAMMA).clamp(Vec3::splat(1.0), Vec3::splat(10000000000.0));
		self.penalty_angular = (self.penalty_angular * GAMMA).clamp(Vec3::splat(1.0), Vec3::splat(10000000000.0));
	}

	fn get_updated(
		&self,
		state_1: &Pose,
		_initial_state_1: &Pose,
		state_2: &Pose,
		_initial_state_2: &Pose,
		alpha: f32,
		calc_1: bool
	) -> Option<(Vec6, Mat6)> {
		// linear
		let penalty_mat = Mat3::from_diagonal(self.penalty_linear);
		let c = state_1 * self.body_1_attachment.translation - state_2 * self.body_2_attachment.translation - self.c0_linear * alpha;
		let force: Vec3 = penalty_mat * c + self.lambda_linear;

		let d_prime_linear = if calc_1 { Mat3::IDENTITY } else { -Mat3::IDENTITY };
		let d_prime_angular = if calc_1 { skew(&-(state_1.rotation * self.body_1_attachment.translation)) } else { skew(&(state_2.rotation * self.body_2_attachment.translation)) };

		let d_prime_linear_transpose_times_k = d_prime_linear.transpose() * penalty_mat;
		let d_prime_angular_transpose_times_k = d_prime_angular.transpose() * penalty_mat;

		Some((
			Vec6::from_vec3(d_prime_linear.transpose() * force, d_prime_angular.transpose() * force),
			Mat6::from_mat3(
				d_prime_linear_transpose_times_k * d_prime_linear,
				d_prime_linear_transpose_times_k * d_prime_angular,
				d_prime_angular_transpose_times_k * d_prime_linear,
				d_prime_angular_transpose_times_k * d_prime_angular
			)
		))


        // Diagonal approximation for higher order terms
        // float3 r = body == bodyA ? rotate(bodyA->positionAng, rA) : -rotate(bodyB->positionAng, rB);
        // float3x3 H =
        //     geometricStiffnessBallSocket(0, r) * F[0] +
        //     geometricStiffnessBallSocket(1, r) * F[1] +
        //     geometricStiffnessBallSocket(2, r) * F[2];
        // lhsAng += diagonalize(H);

	}

	fn update_dual(
		&mut self,
		state_1: &Pose,
		_initial_state_1: &Pose,
		state_2: &Pose,
		_initial_state_2: &Pose,
		alpha: f32
	) {
		let penalty_mat = Mat3::from_diagonal(self.penalty_linear);
		let c = state_1 * self.body_1_attachment.translation - state_2 * self.body_2_attachment.translation - self.c0_linear * alpha;
		let force: Vec3 = penalty_mat * c + self.lambda_linear;

		self.lambda_linear = force;

		// penalty
		let beta = 5000000.0; // beta
		self.penalty_linear = (self.penalty_linear + beta * c.abs()).min(Vec3::splat(10000000000.0));
	}
}
