use glam::{ Quat, Vec3, Mat3 };

use crate::{entity, physics, math::{Vec6, Mat6}};
use super::integrator;

pub struct Solver {
	pub collision_stiffness: f32,
}

impl Solver {
	pub fn solve(&mut self, entities: &mut Vec<entity::Entity>, dt: f32) {
		let y_all = entities.iter().map(|entity| integrator::get_integrated_single(entity, dt)).collect();
		let collisions = physics::collision::get_collisions(&entities, &y_all);
		let mut x_guess = y_all.clone();
		for _ in 0..5 {
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
						f -= self.get_f(&entity_collision);
						h += self.get_h(&entity_collision);
					} else {
						let swapped_collision = physics::collision::Collision {
							id1: entity_collision.id2,
							id2: entity_collision.id1,
							collision1: entity_collision.collision2,
							collision2: entity_collision.collision1,
							normal: -entity_collision.normal,
							..*entity_collision
						};
						f -= self.get_f(&swapped_collision);
						h += self.get_h(&swapped_collision);
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
			entities[index].velocity = (x_guess[index].0 - entities[index].position)/dt;
			entities[index].angular_velocity = ((x_guess[index].1 - entities[index].orientation.inverse())).to_scaled_axis()/dt;
			entities[index].position = x_guess[index].0;
			entities[index].orientation = x_guess[index].1;
		}
	}

	// E = 0.5 * self.collision_stiffness * (collision.collision1 - collision.collision2).length_squared();
	fn get_f(&self, collision: &physics::collision::Collision, ) -> Vec6 {
		Vec6::from_vec3(
			self.collision_stiffness * Vec3::new(
				collision.collision1.x - collision.collision2.x,
				collision.collision1.y - collision.collision2.y,
				collision.collision1.z - collision.collision2.z,
			),
			Vec3::ZERO
		)
	}

	fn get_h(&self, collision: &physics::collision::Collision) -> Mat6 {
		Mat6::from_mat3(self.collision_stiffness * Mat3::IDENTITY, Mat3::ZERO, Mat3::ZERO, Mat3::ZERO)
	}

	fn sub_state(state_a: &(Vec3, Quat), state_b: &(Vec3, Quat)) -> Vec6 {
		Vec6::from_vec3(state_a.0 - state_b.0, (state_a.1 * state_b.1.inverse() * 2.0).xyz())
	}
}

