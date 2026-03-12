use std::{collections::HashMap};

use glam::{IVec3, Mat3, Quat, Vec2, Vec3};
use tracy_client::span;

use crate::{math::{Mat6, Vec6}, physics, pose::Pose};

use super::{physics_body};

type CollisionKlMapKey = (u32, u32, IVec3, physics::collision::CubeFeature, u32, u32, IVec3, physics::collision::CubeFeature);
pub struct Solver {
	collisions_kl_map: HashMap<CollisionKlMapKey, ((Vec3, Vec3), (Vec3, Vec3))>,
}

fn mat6_outer(a: Vec6, b: Vec6) -> Mat6 {
	Mat6::from_cols(a * b.get(0), a * b.get(1), a * b.get(2), a * b.get(3), a * b.get(4), a * b.get(5))
}

impl Solver {
	pub fn new() -> Self {
		Self { collisions_kl_map: HashMap::new(), }
	}

	pub fn add_vec_to_quat(q: &Quat, dx: &Vec3) -> Quat { (q + (Quat::from_xyzw(dx.x, dx.y, dx.z, 0.0) * 0.5) * q).normalize() }
	pub fn sub_quat(q1: &Quat, q2: &Quat) -> Vec3 { (q1 * q2.inverse()).xyz() * 2.0 }

	fn sub_state(state_a: &Pose, state_b: &Pose) -> Vec6 {
		Vec6::from_vec3(state_a.translation - state_b.translation, Self::sub_quat(&state_a.rotation, &state_b.rotation))
	}

	pub fn solve(&mut self, physics_bodies: &mut Vec<physics_body::PhysicsBody>, dt: f32) {
		let _zone = span!("Solve Collisions");
		let collisions: Vec<_> = physics::collision::get_collisions(&physics_bodies).iter().map(
			|c| {
				let body1 = &physics_bodies[c.body_index1 as usize];
				let body2 = &physics_bodies[c.body_index2 as usize];
				physics::collision::Collision {
					local_collision1: c.local_collision1 - body1.get_local_center_of_mass(),
					local_collision2: c.local_collision2 - body2.get_local_center_of_mass(),
					..*c
				}
			}
		).collect();
		let initial_all:Vec<Pose> = physics_bodies.iter().map(|physics_body| Pose::new(physics_body.get_center_of_mass(), Quat::IDENTITY) * physics_body.pose).collect();
		let y_all: Vec<Pose> = physics_bodies.iter().map(|physics_body| {
			if physics_body.is_static || physics_body.mass() < f32::EPSILON { return Pose::new(physics_body.get_center_of_mass(), Quat::IDENTITY) * physics_body.pose; }
			let gravity = -90.0;
			let pos = physics_body.pose.translation + physics_body.velocity * dt + Vec3::new(0.0, gravity, 0.0) * (0.5 * dt * dt) + physics_body.get_center_of_mass();
			let orientation = (Quat::from_scaled_axis(physics_body.angular_velocity * dt) * physics_body.pose.rotation).normalize();
			Pose::new(pos, orientation)
		}).collect();
		let gamma = 0.99;
		let mut collisions_kl: Vec<((Vec3, Vec3), (Vec3, Vec3))> = collisions.iter().map(|collision| {
			let result = self.collisions_kl_map.get(&if collision.body_index1 < collision.body_index2 {
				(
					collision.body_index1, collision.sub_grid_index1, collision.voxel_pos1, collision.feature1,
					collision.body_index2, collision.sub_grid_index2, collision.voxel_pos2, collision.feature2
				)
			} else {
				(
					collision.body_index2, collision.sub_grid_index2, collision.voxel_pos2, collision.feature2,
					collision.body_index1, collision.sub_grid_index1, collision.voxel_pos1, collision.feature1
				)
			});
			result.map_or(
				((Vec3::ONE, Vec3::ZERO), (Vec3::ONE, Vec3::ZERO)),
				|((k1, l1), (k2, l2))| (((k1 * gamma).max(Vec3::ONE), *l1), ((k2 * gamma).max(Vec3::ONE), *l2))
			)
		}).collect();
		self.collisions_kl_map.clear();
		let mut x_guess = initial_all.clone();
		let iterations = 20;
		let total_iterations = iterations + 1; // because post stabilize
		for iteration in 0..total_iterations {
			let _zone = span!("Solve Iteration");
			let alpha = (iteration < iterations) as i32 as f32 * 0.995;
			for index in 0..physics_bodies.len() {
				let physics_body = &physics_bodies[index];
				if physics_body.is_static { continue; }

				let m = Mat6::from_mat3(physics_body.mass() * Mat3::IDENTITY, Mat3::ZERO, Mat3::ZERO, physics_body.rotational_inertia());
				let mut h: Mat6 = m / (dt * dt);
				let mut f: Vec6 = h * Self::sub_state(&x_guess[index], &y_all[index]);

				let physics_body_collisions = collisions.iter().zip(collisions_kl.iter_mut()).filter(|collision| {
					collision.0.body_index1 == index as u32 || collision.0.body_index2 == index as u32
				});
				for physics_body_collision in physics_body_collisions {
					if physics_body_collision.0.body_index1 == index as u32 {
						let k = physics_body_collision.1.0.0;
						let l = physics_body_collision.1.0.1;
						let result = self.get_collision_f_h_and_new_kl(&x_guess[index], &initial_all[index], &x_guess[physics_body_collision.0.body_index2 as usize], &initial_all[physics_body_collision.0.body_index2 as usize], &physics_body_collision.0, k, l, alpha);
						if result.is_none() { continue; }
						let (c_f, c_h, _, _) = result.unwrap();
						f += c_f;
						h += c_h;
					} else {
						let swapped_collision = physics_body_collision.0.get_swaped();
						let k = physics_body_collision.1.1.0;
						let l = physics_body_collision.1.1.1;
						let result = self.get_collision_f_h_and_new_kl(&x_guess[index], &initial_all[index], &x_guess[swapped_collision.body_index2 as usize], &initial_all[swapped_collision.body_index2 as usize], &swapped_collision, k, l, alpha);
						if result.is_none() { continue; }
						let (c_f, c_h, _, _) = result.unwrap();
						f += c_f;
						h += c_h;
					}
				}
				let mats = h.to_mat3();
				let solved = solve(mats[0], mats[3], mats[1], -f.upper_vec3(), -f.lower_vec3());
				let x_change = Vec6::from_vec3(solved.0, solved.1);
				x_guess[index].translation += x_change.upper_vec3();
				x_guess[index].rotation = (
					x_guess[index].rotation +
					Quat::from_xyzw(x_change.get(3) * 0.5, x_change.get(4) * 0.5, x_change.get(5) * 0.5, 0.0) * x_guess[index].rotation
				).normalize();
			}
			if iteration < iterations {
				for index in 0..physics_bodies.len() {
					let physics_body = &physics_bodies[index];
					if physics_body.is_static { continue; }

					let physics_body_collisions = collisions.iter().zip(collisions_kl.iter_mut()).filter(|collision| {
						collision.0.body_index1 == index as u32 || collision.0.body_index2 == index as u32
					});
					for physics_body_collision in physics_body_collisions {
						if physics_body_collision.0.body_index1 == index as u32 {
							let k = physics_body_collision.1.0.0;
							let l = physics_body_collision.1.0.1;
							let result = self.get_collision_f_h_and_new_kl(&x_guess[index], &initial_all[index], &x_guess[physics_body_collision.0.body_index2 as usize], &initial_all[physics_body_collision.0.body_index2 as usize], &physics_body_collision.0, k, l, alpha);
							if result.is_none() { continue; }
							let (_, _, new_k, new_l) = result.unwrap();
							physics_body_collision.1.0.0 = new_k;
							physics_body_collision.1.0.1 = new_l;
						} else {
							let swapped_collision = physics_body_collision.0.get_swaped();
							let k = physics_body_collision.1.1.0;
							let l = physics_body_collision.1.1.1;
							let result = self.get_collision_f_h_and_new_kl(&x_guess[index], &initial_all[index], &x_guess[swapped_collision.body_index2 as usize], &initial_all[swapped_collision.body_index2 as usize], &swapped_collision, k, l, alpha);
							if result.is_none() { continue; }
							let (_, _, new_k, new_l) = result.unwrap();
							physics_body_collision.1.1.0 = new_k;
							physics_body_collision.1.1.1 = new_l;
						}
					}
				}
			}
			if iteration == iterations - 1 { // before post stabilize
				for index in 0..physics_bodies.len() {
					physics_bodies[index].velocity = (x_guess[index].translation - initial_all[index].translation) / dt;
					physics_bodies[index].angular_velocity = (x_guess[index].rotation * initial_all[index].rotation.inverse()).normalize().to_scaled_axis() / dt;
				}
			}
		}
		// after post stabilize
		for index in 0..physics_bodies.len() {
			physics_bodies[index].pose.rotation = x_guess[index].rotation;
			physics_bodies[index].pose.translation = x_guess[index].translation - physics_bodies[index].get_center_of_mass();
		}
		// save K and L
		collisions_kl.iter().zip(collisions).for_each(|(data, collision)| {
			self.collisions_kl_map.insert(if collision.body_index1 < collision.body_index2 {
				(
					collision.body_index1, collision.sub_grid_index1, collision.voxel_pos1, collision.feature1,
					collision.body_index2, collision.sub_grid_index2, collision.voxel_pos2, collision.feature2
				)
			} else {
				(
					collision.body_index2, collision.sub_grid_index2, collision.voxel_pos2, collision.feature2,
					collision.body_index1, collision.sub_grid_index1, collision.voxel_pos1, collision.feature1
				)
			}, *data);
		});
	}

	fn get_collision_f_h_and_new_kl(
		&self,
		this_state: &Pose,
		this_initial_state: &Pose,
		other_state: &Pose,
		other_initial_state: &Pose,
		collision: &physics::collision::Collision,
		k: Vec3, // penalty
		lambda: Vec3,
		alpha: f32
	) -> Option<(Vec6, Mat6, Vec3, Vec3)> {
		let normal = (collision.collision2 - collision.collision1).normalize();

		// debug_draw::line(this_initial_state.0, this_initial_state.0 + normal * 2.0, Vec4::W + Vec4::Y);

		if normal.is_nan() { return None }
		let orthonormal_basis = normal.any_orthonormal_pair();
		let basis = Mat3::from_cols(
			normal,
			orthonormal_basis.0,
			orthonormal_basis.1
		).transpose();

		let world_local_collision1 = this_state.rotation * collision.local_collision1;
		let world_local_collision2 = other_state.rotation * collision.local_collision2;

		let d_prime_linear = basis;
		let d_prime_angular = Mat3::from_cols(
			world_local_collision1.cross(d_prime_linear.row(0)),
			world_local_collision1.cross(d_prime_linear.row(1)),
			world_local_collision1.cross(d_prime_linear.row(2))
		).transpose();
		let d_prime_other_linear = -basis;
		let d_prime_other_angular = Mat3::from_cols(
			world_local_collision2.cross(d_prime_other_linear.row(0)),
			world_local_collision2.cross(d_prime_other_linear.row(1)),
			world_local_collision2.cross(d_prime_other_linear.row(2))
		).transpose();

		let diff = Self::sub_state(this_state, this_initial_state);
		let diff_other = Self::sub_state(other_state, other_initial_state);

		let c0 = basis * (collision.collision1 - collision.collision2) + Vec3::new(0.0005, 0.0, 0.0);
		let c = c0 * (1.0 - alpha) + (
			d_prime_linear * diff.upper_vec3() + d_prime_angular * diff.lower_vec3() +
			d_prime_other_linear * diff_other.upper_vec3() + d_prime_other_angular * diff_other.lower_vec3()
		);

		let k_mat = Mat3::from_diagonal(k);

		let mut f: Vec3 = k_mat * c + lambda;
		f.x = f.x.min(0.0);

		let friction = 0.1;
		let bounds = f.x.abs() * friction;
		let friction_scale = Vec2::new(f.y, f.z).length();
		if friction_scale > bounds && friction_scale > 0.0 {
            f.y *= bounds / friction_scale;
            f.z *= bounds / friction_scale;
        }

		let d_prime_linear_transpose_times_k = d_prime_linear.transpose() * k_mat;
		let d_prime_angular_transpose_times_k = d_prime_angular.transpose() * k_mat;

		// penalty
		let mut next_f = f;
		let beta = 10000.0; // beta
		if f.x < 0.0 {
			next_f.x = (k.x + beta * c.x.abs()).min(10000000000.0);
		}
        if friction_scale <= bounds {
            next_f.y = (next_f.y + beta * c.y.abs()).min(10000000000.0);
            next_f.z = (next_f.z + beta * c.z.abs()).min(10000000000.0);
            // contacts[i].stick = length(float2{C[1], C[2]}) < STICK_THRESH;
        }

		Some((
			Vec6::from_vec3(d_prime_linear.transpose() * f, d_prime_angular.transpose() * f),
			Mat6::from_mat3(
				d_prime_linear_transpose_times_k * d_prime_linear,
				d_prime_linear_transpose_times_k * d_prime_angular,
				d_prime_angular_transpose_times_k * d_prime_linear,
				d_prime_angular_transpose_times_k * d_prime_angular
			),
			next_f,
			f
		))
	}
}

// From https://github.com/savant117/avbd-demo3d
fn solve(a_lin: Mat3, a_ang: Mat3, a_cross: Mat3, b_lin: Vec3, b_ang: Vec3) -> (Vec3, Vec3) {
    // Extract elements from lower triangle storage
    let a11 = a_lin.col(0).to_array()[0];
    let a21 = a_lin.col(1).to_array()[0];
	let a22 = a_lin.col(1).to_array()[1];
    let a31 = a_lin.col(2).to_array()[0];
	let a32 = a_lin.col(2).to_array()[1];
	let a33 = a_lin.col(2).to_array()[2];
    let a41 = a_cross.col(0).to_array()[0];
	let a42 = a_cross.col(0).to_array()[1];
	let a43 = a_cross.col(0).to_array()[2];
	let a44 = a_ang.col(0).to_array()[0];
    let a51 = a_cross.col(1).to_array()[0];
	let a52 = a_cross.col(1).to_array()[1];
	let a53 = a_cross.col(1).to_array()[2];
	let a54 = a_ang.col(1).to_array()[0];
	let a55 = a_ang.col(1).to_array()[1];
    let a61 = a_cross.col(2).to_array()[0];
	let a62 = a_cross.col(2).to_array()[1];
	let a63 = a_cross.col(2).to_array()[2];
	let a64 = a_ang.col(2).to_array()[0];
	let a65 = a_ang.col(2).to_array()[1];
	let a66 = a_ang.col(2).to_array()[2];

    // Step 1: LDL^T decomposition
    let l21 = a21 / a11;
    let l31 = a31 / a11;
    let l41 = a41 / a11;
    let l51 = a51 / a11;
    let l61 = a61 / a11;

    let d1 = a11;

    let d2 = a22 - l21 * l21 * d1;

    let l32 = (a32 - l21 * l31 * d1) / d2;
    let l42 = (a42 - l21 * l41 * d1) / d2;
    let l52 = (a52 - l21 * l51 * d1) / d2;
    let l62 = (a62 - l21 * l61 * d1) / d2;

    let d3 = a33 - (l31 * l31 * d1 + l32 * l32 * d2);

    let l43 = (a43 - l31 * l41 * d1 - l32 * l42 * d2) / d3;
    let l53 = (a53 - l31 * l51 * d1 - l32 * l52 * d2) / d3;
    let l63 = (a63 - l31 * l61 * d1 - l32 * l62 * d2) / d3;

    let d4 = a44 - (l41 * l41 * d1 + l42 * l42 * d2 + l43 * l43 * d3);

    let l54 = (a54 - l41 * l51 * d1 - l42 * l52 * d2 - l43 * l53 * d3) / d4;
    let l64 = (a64 - l41 * l61 * d1 - l42 * l62 * d2 - l43 * l63 * d3) / d4;

    let d5 = a55 - (l51 * l51 * d1 + l52 * l52 * d2 + l53 * l53 * d3 + l54 * l54 * d4);

    let l65 = (a65 - l51 * l61 * d1 - l52 * l62 * d2 - l53 * l63 * d3 - l54 * l64 * d4) / d5;

    let d6 = a66 - (l61 * l61 * d1 + l62 * l62 * d2 + l63 * l63 * d3 + l64 * l64 * d4 + l65 * l65 * d5);

    // Step 2: Forward substitution: Solve Ly = b
    let y1 = b_lin[0];
    let y2 = b_lin[1] - l21 * y1;
    let y3 = b_lin[2] - l31 * y1 - l32 * y2;
    let y4 = b_ang[0] - l41 * y1 - l42 * y2 - l43 * y3;
    let y5 = b_ang[1] - l51 * y1 - l52 * y2 - l53 * y3 - l54 * y4;
    let y6 = b_ang[2] - l61 * y1 - l62 * y2 - l63 * y3 - l64 * y4 - l65 * y5;

    // Step 3: Diagonal solve: Solve Dz = y
    let z1 = y1 / d1;
    let z2 = y2 / d2;
    let z3 = y3 / d3;
    let z4 = y4 / d4;
    let z5 = y5 / d5;
    let z6 = y6 / d6;

    // Step 4: Backward substitution: Solve L^T x = z
	let mut x_ang = Vec3::ZERO;
    x_ang[2] = z6;
    x_ang[1] = z5 - l65 * x_ang[2];
    x_ang[0] = z4 - l54 * x_ang[1] - l64 * x_ang[2];
	let mut x_lin = Vec3::ZERO;
    x_lin[2] = z3 - l43 * x_ang[0] - l53 * x_ang[1] - l63 * x_ang[2];
    x_lin[1] = z2 - l32 * x_lin[2] - l42 * x_ang[0] - l52 * x_ang[1] - l62 * x_ang[2];
    x_lin[0] = z1 - l21 * x_lin[1] - l31 * x_lin[2] - l41 * x_ang[0] - l51 * x_ang[1] - l61 * x_ang[2];
	(x_lin, x_ang)
}