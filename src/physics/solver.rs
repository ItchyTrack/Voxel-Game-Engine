use std::vec;

use glam::{ Mat3, Quat, Vec3, Vec4 };

use crate::{debug_draw, entity, math::{Mat6, Vec6}, physics};
use super::integrator;

pub struct Solver {
	pub collision_stiffness: f32,
}

fn mat3_skew(v: Vec3) -> Mat3 {
	Mat3::from_cols(
		Vec3::new( 0.0,  v.z, -v.y),
		Vec3::new(-v.z,  0.0,  v.x),
		Vec3::new( v.y, -v.x,  0.0),
	)
}

fn mat3_skew_neg(v: Vec3) -> Mat3 {
    Mat3::from_cols(
        Vec3::new( 0.0, -v.z,  v.y),
        Vec3::new( v.z,  0.0, -v.x),
        Vec3::new(-v.y,  v.x,  0.0),
    )
}

fn mat3_outer(a: Vec3, b: Vec3) -> Mat3 {
	Mat3::from_cols(a * b.x, a * b.y, a * b.z)
}
fn mat6_outer(a: Vec6, b: Vec6) -> Mat6 {
	Mat6::from_cols(a * b.get(0), a * b.get(1), a * b.get(2), a * b.get(3), a * b.get(4), a * b.get(5))
}

impl Solver {
	pub fn solve(&mut self, entities: &mut Vec<entity::Entity>, dt: f32) {
		let y_all = entities.iter().map(|entity| integrator::get_integrated_single(entity, dt)).collect();
		let collisions = physics::collision::get_collisions(&entities, &y_all);
		for collision in collisions.iter() {
			debug_draw::line(collision.collision1, collision.collision2, Vec4::new(1.0, 0.0, 0.0, 1.0));
		}
		let mut collisions_kld: Vec<((f32, f32, f32), (f32, f32, f32))> = vec![((20.0, 0.0, 0.0), (20.0, 0.0, 0.0)); collisions.len()];
		let mut x_guess = y_all.clone();
		for _ in 0..30 {
			for index in 0..entities.len() {
				let entity = &entities[index];
				if entity.is_static { continue; }
				let entity_collisions = collisions.iter().zip(collisions_kld.iter_mut()).filter(|collision| {
					collision.0.id1 == index as u32 || collision.0.id2 == index as u32
				});
				let M = Mat6::from_mat3(entity.mass() * Mat3::IDENTITY, Mat3::ZERO, Mat3::ZERO, entity.rotational_inertia());
				let mut f: Vec6 = -(M * Self::sub_state(&x_guess[index], &y_all[index]) / (dt * dt));
				let mut h: Mat6 = M / (dt * dt);
				for entity_collision in entity_collisions {
					if entity_collision.0.id1 == index as u32 {
						let k = entity_collision.1.0.0;
						let l = entity_collision.1.0.1;
						let d = entity_collision.1.1.2;
						let result = self.get_collision_f_h_and_new_kld(&x_guess[index], &x_guess[entity_collision.0.id2 as usize], &entity_collision.0, k, l, d);
						if result.is_none() { continue; }
						let (c_f, c_h, new_k, new_l, new_d) = result.unwrap();
						f -= c_f;
						h += c_h;
						entity_collision.1.0.0 = new_k;
						entity_collision.1.0.1 = new_l;
						entity_collision.1.1.2 = new_d;
					} else {
						let swapped_collision = physics::collision::Collision {
							id1: entity_collision.0.id2,
							id2: entity_collision.0.id1,
							collision1: entity_collision.0.collision2,
							collision2: entity_collision.0.collision1,
							local_collision1: entity_collision.0.local_collision2,
							local_collision2: entity_collision.0.local_collision1,
						};
						let k = entity_collision.1.1.0;
						let l = entity_collision.1.1.1;
						let d = entity_collision.1.1.2;
						let result = self.get_collision_f_h_and_new_kld(&x_guess[index], &x_guess[swapped_collision.id2 as usize], &swapped_collision, k, l, d);
						if result.is_none() { continue; }
						let (c_f, c_h, new_k, new_l, new_d) = result.unwrap();
						f -= c_f;
						h += c_h;
						entity_collision.1.1.0 = new_k;
						entity_collision.1.1.1 = new_l;
						entity_collision.1.1.2 = new_d;
					}
				}
				let x_change = h.inverse() * f;
				x_guess[index].0 += x_change.upper_vec3();
				x_guess[index].1 = (
					x_guess[index].1 +
					Quat::from_xyzw(x_change.get(3) * 0.5, x_change.get(4) * 0.5, x_change.get(5) * 0.5, 0.0) * x_guess[index].1
				).normalize();
			}
		}
		for index in 0..entities.len() {
			entities[index].velocity = (x_guess[index].0 - entities[index].position)/dt;
			entities[index].angular_velocity = ((x_guess[index].1 * entities[index].orientation.inverse())).to_scaled_axis()/dt;
			entities[index].position = x_guess[index].0;
			entities[index].orientation = x_guess[index].1;
		}
	}

	/*
	 * E = 0
	 */
	fn get_collision_d(&self, this_state: &(Vec3, Quat), other_state: &(Vec3, Quat), collision: &physics::collision::Collision) -> Option<(f32, Vec6, Mat6)> {
		let r1_rotated = this_state.1 * collision.local_collision1;
		let r2_rotated = other_state.1 * collision.local_collision2;
		let p1 = this_state.0 + r1_rotated;
		let p2 = other_state.0 + r2_rotated;
		let diff = p1 - p2;
		let normal = (collision.collision1 - collision.collision2).normalize();
		if normal.is_nan() { return None }
		// calculation
		// d = normal dot (p1 - p2)
		// d = normal.x * (p1 - p2).x + normal.y * (p1 - p2).y + normal.z * (p1 - p2).z
		// d = normal.x * (p1.x - p2.x) + normal.y * (p1.y - p2.y) + normal.z * (p1.z - p2.z)
		// d = normal.x * p1.x - normal.x * p2.x + normal.y * p1.y - normal.y * p2.y + normal.z * p1.z - normal.z * p2.z
		// d_x = normal.x
		// d_y = normal.y
		// d_z = normal.z

		// d = max(normal dot (p1 - p2), 0)
		let d = normal.dot(diff);
		if d <= 0.0 { return None; }

		// d' = normal
		let d_prime = Vec6::from_vec3(normal, r1_rotated.cross(normal));

		// d'' = 0
		let d_prime_prime = Mat6::from_mat3(Mat3::ZERO, Mat3::ZERO, Mat3::ZERO, mat3_skew_neg(r1_rotated.cross(normal)));

		Some((d, d_prime, d_prime_prime))
	}

	fn get_collision_f_h_and_new_kld(
		&self,
		this_state: &(Vec3, Quat),
		other_state: &(Vec3, Quat),
		collision: &physics::collision::Collision,
		k: f32,
		l: f32,
		d_old: f32
	) -> Option<(Vec6, Mat6, f32, f32, f32)> {
		let (d, d_prime, d_prime_prime) = self.get_collision_d(this_state, other_state, collision)?;
		// VBD
		// e = 1/2*k*d^2
		// f = e' = k*d*d'
		// h = e'' = k*(d*d'' + d'*d')
		// Some((self.collision_stiffness * d * d_prime, self.collision_stiffness * (d * d_prime_prime + mat6_outer(d_prime, d_prime))))
		// AVBD
		// e = 1/2*k*d^2+l*d
		// f = e' = (k*d+l)*d'
		// h = e'' = k*(d*d'' + d'*d')
		let a = 0.50; // alpha
		let d_corrected = d - a * d_old;
		let b = 20.0; // beta
		Some(((k * d_corrected + l) * d_prime, k * (d_corrected * d_prime_prime + mat6_outer(d_prime, d_prime)), k + b * d_corrected, k * d_corrected + l, d))
	}

	fn sub_state(state_a: &(Vec3, Quat), state_b: &(Vec3, Quat)) -> Vec6 {
		Vec6::from_vec3(state_a.0 - state_b.0, (state_a.1 * state_b.1.inverse()).xyz() * 2.0)
	}
}
