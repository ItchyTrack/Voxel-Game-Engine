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

impl Solver {
	pub fn solve(&mut self, entities: &mut Vec<entity::Entity>, dt: f32) {
		let y_all = entities.iter().map(|entity| integrator::get_integrated_single(entity, dt)).collect();
		let collisions = physics::collision::get_collisions(&entities, &y_all);
		for c in collisions.iter() {
			debug_draw::line(c.collision1, c.collision2, Vec4::new(1.0, 0.0, 0.0, 1.0));
		}
		let mut x_guess = y_all.clone();
		for _ in 0..1 {
			for index in 0..entities.len() {
				let entity = &entities[index];
				let entity_collisions = collisions.iter().filter(|c| {
					c.id1 == index as u32 || c.id2 == index as u32
				});
				let M = Mat6::from_mat3(entity.mass() * Mat3::IDENTITY, Mat3::ZERO, Mat3::ZERO, entity.rotational_inertia());
				let mut f: Vec6 = -(M * Self::sub_state(&x_guess[index], &y_all[index]) / (dt * dt));
				let mut h: Mat6 = M / (dt * dt);
				for entity_collision in entity_collisions {
					if entity_collision.id1 == index as u32 {
						f -= self.get_f(&x_guess[index], &x_guess[entity_collision.id2 as usize], &entity_collision);
						h += self.get_h(&x_guess[index], &x_guess[entity_collision.id2 as usize], &entity_collision);
					} else {
						let swapped_collision = physics::collision::Collision {
							id1: entity_collision.id2,
							id2: entity_collision.id1,
							collision1: entity_collision.collision2,
							collision2: entity_collision.collision1,
							local_collision1: entity_collision.local_collision2,
							local_collision2: entity_collision.local_collision1,
						};
						f -= self.get_f(&x_guess[index], &x_guess[swapped_collision.id2 as usize], &swapped_collision);
						h += self.get_h(&x_guess[index], &x_guess[swapped_collision.id2 as usize], &swapped_collision);
					}
				}
				let x_change = h.inverse() * f;
				x_guess[index].0 += x_change.upper_vec3();
				x_guess[index].1 = (x_guess[index].1 + {
					Quat::from_xyzw(x_change.get(3), x_change.get(4), x_change.get(5), 0.0)
				} * 0.5 * x_guess[index].1).normalize();

			}
		}
		for index in 0..entities.len() {
			let com = entities[index].center_of_mass();
			entities[index].velocity = (x_guess[index].0 + x_guess[index].1 * com - entities[index].position - entities[index].orientation * com)/dt;
			entities[index].angular_velocity = ((x_guess[index].1 * entities[index].orientation.inverse())).to_scaled_axis()/dt;
			entities[index].position = x_guess[index].0;
			entities[index].orientation = x_guess[index].1;
		}
	}


	/*
	 * E = 0.5 * self.collision_stiffness * (
	 * 	this_state.0 + this_state.1 * collision.local_collision1 -
	 * 	other_state.0 + other_state.1 * collision.local_collision2
	 *	).length_squared();
	 */
	fn get_f(&self, this_state: &(Vec3, Quat), other_state: &(Vec3, Quat), collision: &physics::collision::Collision) -> Vec6 {
		let p1 = this_state.0 + this_state.1 * collision.local_collision1;
		let p2 = other_state.0 + other_state.1 * collision.local_collision2;
		Vec6::from_vec3(self.collision_stiffness * Vec3::new(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z), Vec3::ZERO)
	}

	fn get_h(&self, _this_state: &(Vec3, Quat), _other_state: &(Vec3, Quat), _collision: &physics::collision::Collision) -> Mat6 {
		Mat6::from_mat3(self.collision_stiffness * Mat3::IDENTITY, Mat3::ZERO, Mat3::ZERO, Mat3::ZERO)
	}

	// fn get_f(&self, this_state: &(Vec3, Quat), other_state: &(Vec3, Quat), collision: &physics::collision::Collision) -> Vec6 {
	// 	let r1_rotated = this_state.1 * collision.local_collision1;
	// 	let r2_rotated = other_state.1 * collision.local_collision2;
	// 	let p1 = this_state.0 + r1_rotated;
	// 	let p2 = other_state.0 + r2_rotated;
	// 	let diff = p1 - p2;

	// 	self.collision_stiffness * Vec6::from_vec3(diff, diff.cross(r1_rotated))
	// }

	// fn get_h(&self, this_state: &(Vec3, Quat), _other_state: &(Vec3, Quat), collision: &physics::collision::Collision) -> Mat6 {
	// 	let r = this_state.1 * collision.local_collision1;

	// 	let h_tt = Mat3::IDENTITY;
	// 	let h_tr = mat3_skew_neg(r);          // top-right
	// 	let h_rt = mat3_skew(r);              // bottom-left = h_tr.T
	// 	let h_rr = r.length_squared() * Mat3::IDENTITY - mat3_outer(r, r);

	// 	self.collision_stiffness * Mat6::from_mat3(h_tt, h_tr, h_rt, h_rr)
	// }

	fn sub_state(state_a: &(Vec3, Quat), state_b: &(Vec3, Quat)) -> Vec6 {
		Vec6::from_vec3(state_a.0 - state_b.0, (state_a.1 * state_b.1.inverse() * 2.0).xyz())
	}
}

