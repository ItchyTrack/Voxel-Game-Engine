use std::{collections::HashMap};

use glam::{IVec3, Mat3, Quat, Vec3};
use tracy_client::span;

use crate::{math::{Mat6, Vec6}, pose::Pose};

use super::{physics_body, collision_constraint::CollisionConstraint, physics_constraint::PhysicsConstraint, collision};

type CollisionKlMapKey = (u32, u32, IVec3, collision::CubeFeature, u32, u32, IVec3, collision::CubeFeature);
pub struct Solver {
	collisions_kl_map: HashMap<CollisionKlMapKey, (Vec3, Vec3)>,
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

	pub fn sub_state(state_a: &Pose, state_b: &Pose) -> Vec6 {
		Vec6::from_vec3(state_a.translation - state_b.translation, Self::sub_quat(&state_a.rotation, &state_b.rotation))
	}

	pub fn solve(&mut self, physics_bodies: &mut Vec<physics_body::PhysicsBody>, dt: f32) {
		let _zone = span!("Solve Collisions");
		let initial_all:Vec<Pose> = physics_bodies.iter().map(|physics_body| Pose::new(physics_body.get_global_center_of_mass(), Quat::IDENTITY) * physics_body.pose).collect();
		let mut collision_constraints: Vec<CollisionConstraint> = collision::get_collisions(&physics_bodies).iter().map(
			|c| {
				let body1 = &physics_bodies[c.body_index1 as usize];
				let body2 = &physics_bodies[c.body_index2 as usize];
				let collision = collision::Collision {
					local_collision1: c.local_collision1 - body1.get_local_center_of_mass(),
					local_collision2: c.local_collision2 - body2.get_local_center_of_mass(),
					..*c
				};
				let (old_penalty, old_lambda) = self.collisions_kl_map.get(&if collision.body_index1 < collision.body_index2 {
					(
						collision.body_index1, collision.sub_grid_index1, collision.voxel_pos1, collision.feature1,
						collision.body_index2, collision.sub_grid_index2, collision.voxel_pos2, collision.feature2
					)
				} else {
					(
						collision.body_index2, collision.sub_grid_index2, collision.voxel_pos2, collision.feature2,
						collision.body_index1, collision.sub_grid_index1, collision.voxel_pos1, collision.feature1
					)
				}).unwrap_or(&(Vec3::ZERO, Vec3::ZERO));
				let mut collision_constraint = CollisionConstraint::new(collision, old_penalty, old_lambda);
				collision_constraint.init(&initial_all[c.body_index1 as usize], &initial_all[c.body_index2 as usize]);
				collision_constraint
			}
		).collect();
		self.collisions_kl_map.clear();
		let y_all: Vec<Pose> = physics_bodies.iter().map(|physics_body| {
			if physics_body.is_static || physics_body.mass() < f32::EPSILON { return Pose::new(physics_body.get_global_center_of_mass(), Quat::IDENTITY) * physics_body.pose; }
			let gravity = -90.0;
			let pos = physics_body.pose.translation + physics_body.velocity * dt + Vec3::new(0.0, gravity, 0.0) * (0.5 * dt * dt) + physics_body.get_global_center_of_mass();
			let orientation = (Quat::from_scaled_axis(physics_body.angular_velocity * dt) * physics_body.pose.rotation).normalize();
			Pose::new(pos, orientation)
		}).collect();
		let mut x_guess = initial_all.clone();
		let iterations = 30;
		let total_iterations = iterations + 1; // because post stabilize
		for iteration in 0..total_iterations {
			let _zone = span!("Solve Iteration");
			let alpha = (iteration < iterations) as i32 as f32 * 0.995;
			for index in 0..physics_bodies.len() {
				let physics_body = &physics_bodies[index];
				if physics_body.is_static { continue; }

				let m = Mat6::from_mat3(physics_body.mass() * Mat3::IDENTITY, Mat3::ZERO, Mat3::ZERO, physics_body.rotational_inertia().mat.as_mat3());
				let mut h: Mat6 = m / (dt * dt);
				let mut f: Vec6 = h * Self::sub_state(&x_guess[index], &y_all[index]);

				let physics_body_collisions = collision_constraints.iter().filter(|collision| {
					collision.collision.body_index1 == index as u32 || collision.collision.body_index2 == index as u32
				});
				for physics_body_collision in physics_body_collisions {
					let result = physics_body_collision.get_updated(
						&x_guess[physics_body_collision.collision.body_index1 as usize], &initial_all[physics_body_collision.collision.body_index1 as usize],
						&x_guess[physics_body_collision.collision.body_index2 as usize], &initial_all[physics_body_collision.collision.body_index2 as usize],
						alpha,
						physics_body_collision.collision.body_index1 == index as u32
					);
					if result.is_none() { continue; }
					let (c_f, c_h) = result.unwrap();
					f += c_f;
					h += c_h;
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
				for collision_constraint in collision_constraints.iter_mut() {
					collision_constraint.update_dual(
						&x_guess[collision_constraint.collision.body_index1 as usize], &initial_all[collision_constraint.collision.body_index1 as usize],
						&x_guess[collision_constraint.collision.body_index2 as usize], &initial_all[collision_constraint.collision.body_index2 as usize],
						alpha
					);
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
			physics_bodies[index].pose.translation = x_guess[index].translation - physics_bodies[index].get_global_center_of_mass();
		}
		// save K and L
		for collision_constraint in collision_constraints {
			let collision = collision_constraint.collision;
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
			}, (collision_constraint.penalty, collision_constraint.lambda));
		}
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
