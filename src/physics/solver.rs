use std::{collections::HashMap};

use glam::{ Mat3, Quat, Vec3, Vec4 };

use crate::{debug_draw, entity, math::{Mat6, Vec6, add_vec_to_quat, sub_quat}, physics};
use super::integrator;

pub struct Solver {
	collisions_kl_map: HashMap<(u32, physics::collision::CubeFeature, u32, physics::collision::CubeFeature), ((f32, f32), (f32, f32))>,
}

fn mat6_outer(a: Vec6, b: Vec6) -> Mat6 {
	Mat6::from_cols(a * b.get(0), a * b.get(1), a * b.get(2), a * b.get(3), a * b.get(4), a * b.get(5))
}

impl Solver {
	pub fn new() -> Self {
		Self { collisions_kl_map: HashMap::new(), }
	}

	pub fn solve(&mut self, entities: &mut Vec<entity::Entity>, dt: f32) {
		let initial_all:Vec<(Vec3, Quat)> = entities.iter().map(|entity| (entity.position, entity.orientation)).collect();
		let collisions = physics::collision::get_collisions(&entities, &initial_all);
		let y_all: Vec<(Vec3, Quat)> = entities.iter().map(|entity| integrator::get_integrated_single(entity, dt)).collect();
		// println!("collisions: {0}", collisions.len());
		for collision in collisions.iter() {
			debug_draw::line(collision.collision1, collision.collision2, Vec4::new(1.0, 0.0, 0.0, 1.0));
		}
		let gamma = 0.99;
		let mut collisions_kl: Vec<((f32, f32), (f32, f32))> = collisions.iter().map(|collision| {
			let result = self.collisions_kl_map.get(&if collision.id1 < collision.id2 {
				(collision.id1, collision.feature1, collision.id2, collision.feature2)
			} else {
				(collision.id2, collision.feature2, collision.id1, collision.feature1)
			});
			result.map_or(
				((1.0, 0.0), (1.0, 0.0)),
				|((k1, l1), (k2, l2))| (((k1 * gamma).max(1.0), *l1), ((k2 * gamma).max(1.0), *l2))
			)
		}).collect();
		self.collisions_kl_map.clear();
		let mut x_guess = initial_all.clone();
		let iterations = 5;
		let total_iterations = iterations + 1; // because post stabilize
		for iteration in 0..total_iterations {
			// println!("------------------------------------------------------------------------");
			let alpha = (iteration < iterations) as i32 as f32;
			for index in 0..entities.len() {
				let entity = &entities[index];
				if entity.is_static { continue; }
				let M = Mat6::from_mat3(entity.mass() * Mat3::IDENTITY, Mat3::ZERO, Mat3::ZERO, entity.rotational_inertia());
				let mut f: Vec6 = M / (dt * dt) * Self::sub_state(&x_guess[index], &y_all[index]);
				let mut h: Mat6 = M / (dt * dt);
				let entity_collisions = collisions.iter().zip(collisions_kl.iter_mut()).filter(|collision| {
					collision.0.id1 == index as u32 || collision.0.id2 == index as u32
				});
				for entity_collision in entity_collisions {
					if entity_collision.0.id1 == index as u32 {
						let k = entity_collision.1.0.0;
						let l = entity_collision.1.0.1;
						let result = self.get_collision_f_h_and_new_kld(&x_guess[index], &initial_all[index], &x_guess[entity_collision.0.id2 as usize], &initial_all[entity_collision.0.id2 as usize], &entity_collision.0, k, l, alpha);
						if result.is_none() { continue; }
						let (c_f, c_h, _, _) = result.unwrap();
						f += c_f;
						h += c_h;
					} else {
						let swapped_collision = physics::collision::Collision {
							id1: entity_collision.0.id2,
							id2: entity_collision.0.id1,
							feature1: entity_collision.0.feature2,
							feature2: entity_collision.0.feature1,
							collision1: entity_collision.0.collision2,
							collision2: entity_collision.0.collision1,
							local_collision1: entity_collision.0.local_collision2,
							local_collision2: entity_collision.0.local_collision1,
						};
						let k = entity_collision.1.1.0;
						let l = entity_collision.1.1.1;
						let result = self.get_collision_f_h_and_new_kld(&x_guess[index], &initial_all[index], &x_guess[swapped_collision.id2 as usize], &initial_all[swapped_collision.id2 as usize], &swapped_collision, k, l, alpha);
						if result.is_none() { continue; }
						let (c_f, c_h, _, _) = result.unwrap();
						f += c_f;
						h += c_h;
					}
				}
				let mats = h.to_mat3();
				// if mats[1].transpose() != mats[2] {
				// 	println!("mats[1].transpose(): {}", mats[1].transpose());
				// 	println!("mats[2]: {}", mats[2]);
				// }
				let solved = solve(mats[0], mats[3], mats[1], f.upper_vec3(), f.lower_vec3());
				let x_change = Vec6::from_vec3(solved.0, solved.1);
				// println!("{}", x_change);
				x_guess[index].0 -= x_change.upper_vec3();
				// let q = Quat::IDENTITY;
				// let v = Vec3::X * 0.1;
				// let q_plus_v = add_vec_to_quat(&q, &v);
				// let v_hope = sub_quat(&q_plus_v, &q);
				// println!("Test: {}, {}, {}, {}", q, v, q_plus_v, v_hope);
				// println!("xg: {}, y: {}, xa: {}", x_guess[index].1, y_all[index].1, add_vec_to_quat(&x_guess[index].1, x_change.lower_vec3()));
				// println!("xg*v1: {}, y*v1: {}, xa*v1: {}", x_guess[index].1 * Vec3::ONE, y_all[index].1 * Vec3::ONE, Self::add_vec_to_quat(&x_guess[index].1, x_change.lower_vec3()) * Vec3::ONE);
				// println!("dif= {}", y_all[index].1 * Vec3::ONE - add_vec_to_quat(&x_guess[index].1, x_change.lower_vec3()) * Vec3::ONE);
				// x_guess[index].1 = add_vec_to_quat(&x_guess[index].1, &(x_change.lower_vec3() / 8.0)); // fix
			}
			if iteration < iterations {
				for index in 0..entities.len() {
					let entity = &entities[index];
					if entity.is_static { continue; }
					let entity_collisions = collisions.iter().zip(collisions_kl.iter_mut()).filter(|collision| {
						collision.0.id1 == index as u32 || collision.0.id2 == index as u32
					});
					for entity_collision in entity_collisions {
						if entity_collision.0.id1 == index as u32 {
							let k = entity_collision.1.0.0;
							let l = entity_collision.1.0.1;
							let result = self.get_collision_f_h_and_new_kld(&x_guess[index], &initial_all[index], &x_guess[entity_collision.0.id2 as usize], &initial_all[entity_collision.0.id2 as usize], &entity_collision.0, k, l, alpha);
							if result.is_none() { continue; }
							let (_, _, new_k, new_l) = result.unwrap();
							if iteration < iterations {
								entity_collision.1.0.0 = new_k;
								entity_collision.1.0.1 = new_l;
							}
						} else {
							let swapped_collision = physics::collision::Collision {
								id1: entity_collision.0.id2,
								id2: entity_collision.0.id1,
								feature1: entity_collision.0.feature2,
								feature2: entity_collision.0.feature1,
								collision1: entity_collision.0.collision2,
								collision2: entity_collision.0.collision1,
								local_collision1: entity_collision.0.local_collision2,
								local_collision2: entity_collision.0.local_collision1,
							};
							let k = entity_collision.1.1.0;
							let l = entity_collision.1.1.1;
							let result = self.get_collision_f_h_and_new_kld(&x_guess[index], &initial_all[index], &x_guess[swapped_collision.id2 as usize], &initial_all[swapped_collision.id2 as usize], &swapped_collision, k, l, alpha);
							if result.is_none() { continue; }
							let (_, _, new_k, new_l) = result.unwrap();
							if iteration < iterations {
								entity_collision.1.1.0 = new_k;
								entity_collision.1.1.1 = new_l;
							}
						}
					}
				}
			}
			if iteration == iterations - 1 { // before post stabilize
				for index in 0..entities.len() {
					entities[index].velocity = (x_guess[index].0 - initial_all[index].0) / dt;
					// println!("{} -> {}", entities[index].angular_velocity, ((x_guess[index].1 * initial_all[index].1.inverse())).to_scaled_axis()/dt);
					// entities[index].angular_velocity = sub_quat(&x_guess[index].1, &initial_all[index].1) / dt; // fix
				}
			}
		}
		// after post stabilize
		for index in 0..entities.len() {
			entities[index].position = x_guess[index].0;
			entities[index].orientation = x_guess[index].1;
		}
		// save K and L
		collisions_kl.iter().zip(collisions).for_each(|(data, collision)| {
			self.collisions_kl_map.insert(if collision.id1 < collision.id2 {
				(collision.id1, collision.feature1, collision.id2, collision.feature2)
			} else {
				(collision.id2, collision.feature2, collision.id1, collision.feature1)
			}, *data);
		});
	}

	fn get_collision_f_h_and_new_kld(
		&self,
		this_state: &(Vec3, Quat),
		this_initial_state: &(Vec3, Quat),
		other_state: &(Vec3, Quat),
		other_initial_state: &(Vec3, Quat),
		collision: &physics::collision::Collision,
		k: f32, // penalty
		lambda: f32,
		alpha: f32
	) -> Option<(Vec6, Mat6, f32, f32)> {
		let normal = -(collision.collision1 - collision.collision2).normalize();
		if normal.is_nan() { return None }
		let c0 = -(collision.collision1 - collision.collision2).length() + 0.0005;
		let d_prime = Vec6::from_vec3(normal, (this_initial_state.1 * collision.local_collision1).cross(normal));
		let d_prime_other = -Vec6::from_vec3(normal, -(other_initial_state.1 * collision.local_collision2).cross(normal));
		let c = c0 * (1.0 - alpha) + d_prime.dot(&Self::sub_state(this_state, this_initial_state)) + d_prime_other.dot(&Self::sub_state(other_state, other_initial_state));
		let b = 100000.0; // beta
		let f = (k * c + lambda).min(0.0);
		Some((d_prime * f, mat6_outer(d_prime, d_prime * k), k + b * c.abs(), k * c + lambda))
	}

	fn sub_state(state_a: &(Vec3, Quat), state_b: &(Vec3, Quat)) -> Vec6 {
		Vec6::from_vec3(state_a.0 - state_b.0, sub_quat(&state_a.1, &state_b.1))
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
