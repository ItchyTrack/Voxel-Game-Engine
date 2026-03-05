use std::{collections::HashMap};

use glam::{ Mat3, Quat, Vec3, Vec4 };

use crate::{debug_draw, entity, math::{Mat6, Vec6}, physics};
use super::integrator;

pub struct Solver {
	collisions_kl_map: HashMap<(u32, physics::collision::CubeFeature, u32, physics::collision::CubeFeature), ((f32, f32), (f32, f32))>,
}

fn mat3_skew_neg(v: Vec3) -> Mat3 {
    Mat3::from_cols(
        Vec3::new( 0.0, -v.z,  v.y),
        Vec3::new( v.z,  0.0, -v.x),
        Vec3::new(-v.y,  v.x,  0.0),
    )
}

fn mat6_outer(a: Vec6, b: Vec6) -> Mat6 {
	Mat6::from_cols(a * b.get(0), a * b.get(1), a * b.get(2), a * b.get(3), a * b.get(4), a * b.get(5))
}

impl Solver {
	pub fn new() -> Self {
		Self { collisions_kl_map: HashMap::new(), }
	}

	pub fn solve(&mut self, entities: &mut Vec<entity::Entity>, dt: f32) {
		let i_all:Vec<(Vec3, Quat)> = entities.iter().map(|entity| (entity.position, entity.orientation)).collect();
		let y_all = entities.iter().map(|entity| integrator::get_integrated_single(entity, dt)).collect();
		let collisions = physics::collision::get_collisions(&entities, &y_all);
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
				((0.0, 0.0), (0.0, 0.0)),
				|((k1, l1), (k2, l2))| (((k1 * gamma).max(1.0), *l1), ((k2 * gamma).max(1.0), *l2))
			)
		}).collect();
		self.collisions_kl_map.clear();
		let mut x_guess = i_all.clone();
		let iterations = 10;
		let total_iterations = iterations + 1; // because post_stabilize
		for iteration in 0..total_iterations {
			// println!("------------------------ {}", iteration);
			let alpha = (iteration < iterations) as i32 as f32;
			for index in 0..entities.len() {
				let entity = &entities[index];
				if entity.is_static { continue; }
				let entity_collisions = collisions.iter().zip(collisions_kl.iter_mut()).filter(|collision| {
					collision.0.id1 == index as u32 || collision.0.id2 == index as u32
				});
				let M = Mat6::from_mat3(entity.mass() * Mat3::IDENTITY, Mat3::ZERO, Mat3::ZERO, entity.rotational_inertia());
				let mut f: Vec6 = -(M * Self::sub_state(&x_guess[index], &y_all[index]) / (dt * dt));
				let mut h: Mat6 = M / (dt * dt);
				for entity_collision in entity_collisions {
					if entity_collision.0.id1 == index as u32 {
						let k = entity_collision.1.0.0;
						let l = entity_collision.1.0.1;
						let result = self.get_collision_f_h_and_new_kld(&x_guess[index], &i_all[index], &x_guess[entity_collision.0.id2 as usize], &i_all[entity_collision.0.id2 as usize], &entity_collision.0, k, l, alpha);
						if result.is_none() { continue; }
						let (c_f, c_h, _, _) = result.unwrap();
						// println!(">> {}, {}", c_f, c_h);
						f -= c_f;
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
						let result = self.get_collision_f_h_and_new_kld(&x_guess[index], &i_all[index], &x_guess[swapped_collision.id2 as usize], &i_all[swapped_collision.id2 as usize], &swapped_collision, k, l, alpha);
						if result.is_none() { continue; }
						let (c_f, c_h, _, _) = result.unwrap();
						// println!(">> {}, {}", c_f, c_h);
						f -= c_f;
						h += c_h;
					}
				}
				let x_change = h.inverse() * f;
				// println!("x_change: {},    h: {}, hInv: {}", x_change, h, h.inverse());
				x_guess[index].0 += x_change.upper_vec3();
				x_guess[index].1 = (
					x_guess[index].1 +
					Quat::from_xyzw(x_change.get(3) * 0.5, x_change.get(4) * 0.5, x_change.get(5) * 0.5, 0.0) * x_guess[index].1
				).normalize();
			}
			// println!("--");
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
							let result = self.get_collision_f_h_and_new_kld(&x_guess[index], &i_all[index], &x_guess[entity_collision.0.id2 as usize], &i_all[entity_collision.0.id2 as usize], &entity_collision.0, k, l, alpha);
							if result.is_none() { continue; }
							let (_, _, new_k, new_l) = result.unwrap();
							// println!("{}, {}", new_k, new_l);
							entity_collision.1.0.0 = new_k;
							entity_collision.1.0.1 = new_l;
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
							let result = self.get_collision_f_h_and_new_kld(&x_guess[index], &i_all[index], &x_guess[swapped_collision.id2 as usize], &i_all[swapped_collision.id2 as usize], &swapped_collision, k, l, alpha);
							if result.is_none() { continue; }
							let (_, _, new_k, new_l) = result.unwrap();
							// println!("{}, {}", new_k, new_l);
							entity_collision.1.1.0 = new_k;
							entity_collision.1.1.1 = new_l;
						}
					}
				}
			}
			if iteration == iterations - 1 {
				for index in 0..entities.len() {
					entities[index].velocity = (x_guess[index].0 - entities[index].position)/dt;
					entities[index].angular_velocity = ((x_guess[index].1 * entities[index].orientation.inverse())).to_scaled_axis()/dt;
					entities[index].position = x_guess[index].0;
					entities[index].orientation = x_guess[index].1;
				}
			}
		}
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
		let c0 = -(collision.collision1 - collision.collision2).length();
		let d_prime = Vec6::from_vec3(normal, (this_initial_state.1 * collision.local_collision1).cross(normal));
		let d_prime_other = Vec6::from_vec3(-normal, -(other_initial_state.1 * collision.local_collision2).cross(normal));
		let c = c0 * (1.0 - alpha) + d_prime.dot(&Self::sub_state(this_state, this_initial_state)) + d_prime_other.dot(&Self::sub_state(other_state, other_initial_state));
		// println!("--- {}*{} {} {}", c0, (1.0 - alpha), d_prime.dot(&Self::sub_state(this_state, this_initial_state)), d_prime_other.dot(&Self::sub_state(other_state, other_initial_state)));
		let b = 1000.0; // beta
		let f = (k * c + lambda).min(0.0);
		// println!("{} {} {} {}", f, k, c, lambda);
		// if (alpha == 1.0) {
		// 	println!("<{}, {}", d_prime, d_prime_other);
		// 	println!("<{}, {}, {}, {}", k, k + b * d_corrected.abs(), l, f);
		// 	println!("{}, {}, {}, {}", d_corrected, c0 * (1.0 - alpha), d_prime.dot(&Self::sub_state(this_state, this_initial_state)), d_prime_other.dot(&Self::sub_state(other_state, other_initial_state)));
		// 	println!("{}, {}, {}, {}", this_state.0, this_initial_state.0, other_state.0, other_initial_state.0);
		// }
		Some((f * d_prime, mat6_outer(d_prime, d_prime * k), k + b * c.abs(), k * c + lambda))
	}

	fn sub_state(state_a: &(Vec3, Quat), state_b: &(Vec3, Quat)) -> Vec6 {
		Vec6::from_vec3(state_a.0 - state_b.0, (state_a.1 * state_b.1.inverse()).xyz() * 2.0)
	}
}
