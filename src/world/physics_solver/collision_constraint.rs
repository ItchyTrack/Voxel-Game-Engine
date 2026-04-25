use glam::{Mat3, Vec2, Vec3};

use crate::pose::Pose;

use super::{math::{Mat6, Vec6}, physics_constraint::{PhysicsConstraint, GAMMA}, solver::Solver, collision};

pub struct CollisionConstraint {
	pub collision: collision::Collision,
	friction: f32,
	basis: Mat3,
	c0: Vec3,
	pub penalty: Vec3,
	pub lambda: Vec3,
}

impl CollisionConstraint {
	pub fn new(collision: collision::Collision, old_penalty: &Vec3, old_lambda: &Vec3) -> Self {
		Self {
			collision: collision,
			friction: 0.5,
			basis: Mat3::ZERO,
			c0: Vec3::ZERO,
			penalty: *old_penalty,
			lambda: *old_lambda,
		}
	}
}

impl PhysicsConstraint for CollisionConstraint {
	fn init(&mut self, _initial_state_1: &Pose, _initial_state_2: &Pose) {
		let normal = (self.collision.part2.collision - self.collision.part1.collision).normalize();
		if normal.is_nan() { return; }

		let basis_pair = normal.any_orthonormal_pair();
		self.basis = Mat3::from_cols(
			normal,
			basis_pair.0,
			basis_pair.1
		).transpose();

		self.c0 = self.basis * (self.collision.part1.collision - self.collision.part2.collision) + Vec3::new(0.01, 0.0, 0.0);
		self.penalty = (self.penalty * GAMMA).clamp(Vec3::splat(1.0), Vec3::splat(10000000000.0));
	}

	fn get_updated(
		&self,
		state_1: &Pose,
		initial_state_1: &Pose,
		state_2: &Pose,
		initial_state_2: &Pose,
		alpha: f32,
		calc_1: bool
	) -> Option<(Vec6, Mat6)> {
		let world_local_collision_1 = state_1.rotation * self.collision.part1.local_collision;
		let world_local_collision_2 = state_2.rotation * self.collision.part2.local_collision;

		let d_prime_linear_1 = self.basis;
		let d_prime_angular_1 = Mat3::from_cols(
			world_local_collision_1.cross(d_prime_linear_1.row(0)),
			world_local_collision_1.cross(d_prime_linear_1.row(1)),
			world_local_collision_1.cross(d_prime_linear_1.row(2))
		).transpose();
		let d_prime_linear_2 = -self.basis;
		let d_prime_angular_2 = Mat3::from_cols(
			world_local_collision_2.cross(d_prime_linear_2.row(0)),
			world_local_collision_2.cross(d_prime_linear_2.row(1)),
			world_local_collision_2.cross(d_prime_linear_2.row(2))
		).transpose();

		let diff_1 = Solver::sub_state(state_1, initial_state_1);
		let diff_2 = Solver::sub_state(state_2, initial_state_2);

		let c = self.c0 * (1.0 - alpha) + (
			d_prime_linear_1 * diff_1.upper_vec3() + d_prime_angular_1 * diff_1.lower_vec3() +
			d_prime_linear_2 * diff_2.upper_vec3() + d_prime_angular_2 * diff_2.lower_vec3()
		);

		let penalty_mat = Mat3::from_diagonal(self.penalty);

		let mut force: Vec3 = penalty_mat * c + self.lambda;
		force.x = force.x.min(0.0);

		let bounds = force.x.abs() * self.friction;
		let friction_scale = Vec2::new(force.y, force.z).length();
		if friction_scale > bounds && friction_scale > 0.0 {
			force.y *= bounds / friction_scale;
			force.z *= bounds / friction_scale;
		}

		let (d_prime_linear, d_prime_angular) = if calc_1 { (d_prime_linear_1, d_prime_angular_1) } else { (d_prime_linear_2, d_prime_angular_2) };
		// let (d_prime_linear, d_prime_angular) = (d_prime_linear_1, d_prime_angular_1);

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
	}

	fn update_dual(
		&mut self,
		state_1: &Pose,
		initial_state_1: &Pose,
		state_2: &Pose,
		initial_state_2: &Pose,
		alpha: f32
	) {
		let world_local_collision_1 = state_1.rotation * self.collision.part1.local_collision;
		let world_local_collision_2 = state_2.rotation * self.collision.part2.local_collision;

		let d_prime_linear_1 = self.basis;
		let d_prime_angular_1 = Mat3::from_cols(
			world_local_collision_1.cross(d_prime_linear_1.row(0)),
			world_local_collision_1.cross(d_prime_linear_1.row(1)),
			world_local_collision_1.cross(d_prime_linear_1.row(2))
		).transpose();
		let d_prime_linear_2 = -self.basis;
		let d_prime_angular_2 = Mat3::from_cols(
			world_local_collision_2.cross(d_prime_linear_2.row(0)),
			world_local_collision_2.cross(d_prime_linear_2.row(1)),
			world_local_collision_2.cross(d_prime_linear_2.row(2))
		).transpose();

		let diff = Solver::sub_state(state_1, initial_state_1);
		let diff_other = Solver::sub_state(state_2, initial_state_2);

		let c = self.c0 * (1.0 - alpha) + (
			d_prime_linear_1 * diff.upper_vec3() + d_prime_angular_1 * diff.lower_vec3() +
			d_prime_linear_2 * diff_other.upper_vec3() + d_prime_angular_2 * diff_other.lower_vec3()
		);

		let penalty_mat = Mat3::from_diagonal(self.penalty);

		let mut force: Vec3 = penalty_mat * c + self.lambda;
		force.x = force.x.min(0.0);

		let bounds = force.x.abs() * self.friction;
		let friction_scale = Vec2::new(force.y, force.z).length();
		if friction_scale > bounds && friction_scale > 0.0 {
			force.y *= bounds / friction_scale;
			force.z *= bounds / friction_scale;
		}

		self.lambda = force;

		// penalty
		let beta = 5000000.0; // beta
		if force.x < 0.0 {
			self.penalty.x = (self.penalty.x + beta * c.x.abs()).min(10000000000.0);
		}
		if friction_scale <= bounds {
			self.penalty.y = (self.penalty.y + beta * c.y.abs()).min(10000000000.0);
			self.penalty.z = (self.penalty.z + beta * c.z.abs()).min(10000000000.0);
		}
	}
}
