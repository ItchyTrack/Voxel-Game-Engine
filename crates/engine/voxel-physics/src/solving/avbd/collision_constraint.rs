use voxel_transform::Transform;
use crate::math::{FixedVec3Ext, Mat3, Mat6, Vec6, Wide, WideVec3};
use crate::collision;

use super::physics_constraint::{PhysicsConstraint, GAMMA};
use super::Solver;

#[cfg(test)]
#[path = "collision_constraint_tests.rs"]
mod tests;

pub struct CollisionConstraint {
	pub collision: collision::Collision,
	friction: Wide,
	basis: Mat3,
	c0: WideVec3,
	pub penalty: WideVec3,
	pub lambda: WideVec3,
}

impl CollisionConstraint {
	pub fn new(collision: collision::Collision, old_penalty: &WideVec3, old_lambda: &WideVec3) -> Self {
		Self {
			collision,
			friction: Wide::ONE / Wide::from_num(2),
			basis: Mat3::ZERO,
			c0: WideVec3::ZERO,
			penalty: *old_penalty,
			lambda: *old_lambda,
		}
	}
}

impl PhysicsConstraint for CollisionConstraint {
	fn init(&mut self, _initial_state_1: &Transform, _initial_state_2: &Transform) {
		let Some(normal) = (self.collision.part2.collision - self.collision.part1.collision).try_normalize() else { return };

		let basis_pair = normal.any_orthonormal_pair();
		self.basis = Mat3::from_cols(
			normal.into(),
			basis_pair.0.into(),
			basis_pair.1.into()
		).transpose();

		self.c0 = self.basis * WideVec3::from(self.collision.part1.collision - self.collision.part2.collision) + WideVec3::new(Wide::ONE / Wide::from_num(100), Wide::ZERO, Wide::ZERO);
		self.penalty = (self.penalty * GAMMA).clamp(WideVec3::ONE, WideVec3::splat(Wide::from_num(10_000_000_000u64)));
	}

	fn get_updated(
		&self,
		state_1: &Transform,
		initial_state_1: &Transform,
		state_2: &Transform,
		initial_state_2: &Transform,
		alpha: Wide,
		calc_1: bool
	) -> Option<(Vec6, Mat6)> {
		let world_local_collision_1 = WideVec3::from(state_1.rotation * self.collision.part1.local_collision);
		let world_local_collision_2 = WideVec3::from(state_2.rotation * self.collision.part2.local_collision);

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

		let c = self.c0 * (Wide::ONE - alpha) + (
			d_prime_linear_1 * diff_1.upper_vec3() + d_prime_angular_1 * diff_1.lower_vec3() +
			d_prime_linear_2 * diff_2.upper_vec3() + d_prime_angular_2 * diff_2.lower_vec3()
		);

		let penalty_mat = Mat3::from_diagonal(self.penalty);

		let mut force = penalty_mat * c + self.lambda;
		force.x = force.x.min(Wide::ZERO);

		let bounds = force.x.abs() * self.friction;
		let friction_scale = WideVec3::new(Wide::ZERO, force.y, force.z).length();
		if friction_scale > bounds && friction_scale > Wide::ZERO {
			force.y = (force.y / friction_scale) * bounds;
			force.z = (force.z / friction_scale) * bounds;
		}

		let (d_prime_linear, d_prime_angular) = if calc_1 { (d_prime_linear_1, d_prime_angular_1) } else { (d_prime_linear_2, d_prime_angular_2) };

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
		state_1: &Transform,
		initial_state_1: &Transform,
		state_2: &Transform,
		initial_state_2: &Transform,
		alpha: Wide
	) {
		let world_local_collision_1 = WideVec3::from(state_1.rotation * self.collision.part1.local_collision);
		let world_local_collision_2 = WideVec3::from(state_2.rotation * self.collision.part2.local_collision);

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

		let c = self.c0 * (Wide::ONE - alpha) + (
			d_prime_linear_1 * diff.upper_vec3() + d_prime_angular_1 * diff.lower_vec3() +
			d_prime_linear_2 * diff_other.upper_vec3() + d_prime_angular_2 * diff_other.lower_vec3()
		);

		let penalty_mat = Mat3::from_diagonal(self.penalty);

		let mut force = penalty_mat * c + self.lambda;
		force.x = force.x.min(Wide::ZERO);

		let bounds = force.x.abs() * self.friction;
		let friction_scale = WideVec3::new(Wide::ZERO, force.y, force.z).length();
		if friction_scale > bounds && friction_scale > Wide::ZERO {
			force.y = (force.y / friction_scale) * bounds;
			force.z = (force.z / friction_scale) * bounds;
		}

		self.lambda = force;

		let beta = Wide::from_num(5_000_000);
		if force.x < Wide::ZERO {
			self.penalty.x = (self.penalty.x + beta * c.x.abs()).min(Wide::from_num(10_000_000_000u64));
		}
		if friction_scale <= bounds {
			self.penalty.y = (self.penalty.y + beta * c.y.abs()).min(Wide::from_num(10_000_000_000u64));
			self.penalty.z = (self.penalty.z + beta * c.z.abs()).min(Wide::from_num(10_000_000_000u64));
		}
	}
}
