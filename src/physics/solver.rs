use glam::{ Quat, Vec3, Mat3 };

use crate::{entity, physics};
use super::integrator;

pub struct Solver {
	pub collision_stiffness: f32,
}

impl Solver {
	pub fn solve(&mut self, entities: &mut Vec<entity::Entity>, dt: f32) {
		let y_all = entities.iter().map(|entity| integrator::get_integrated_single(entity, dt)).collect();
		let collisions = physics::collision::get_collisions(&entities, &y_all);
		let mut x_guess = y_all.clone();
		for n in 0..4 {
			for index in 0..entities.len() {
				let entity = &entities[index];
				let entity_collisions = collisions.iter().filter(|c| {
					c.id1 == index as u32 || c.id2 == index as u32
				});
				let M = entity.mass() * Mat3::IDENTITY;
				let mut f: Vec3 = -(M * (x_guess[index].0 - y_all[index].0) / (dt * dt));
				let mut h: Mat3 = M / (dt * dt);
				for entity_collision in entity_collisions {
					if entity_collision.id1 == index as u32 {
						f -= self.get_f(&entity_collision).0;
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
						f -= self.get_f(&swapped_collision).0;
						h += self.get_h(&swapped_collision);
					}
				}
				x_guess[index].0 += h.inverse() * f;

			}
		}
		for index in 0..entities.len() {
			entities[index].momentum = entities[index].mass() * (x_guess[index].0 - entities[index].position)/dt;
			entities[index].position = x_guess[index].0;
			entities[index].orientation = x_guess[index].1;
		}
	}
	// E = 0.5 * self.collision_stiffness * (collision.collision1 - collision.collision2).length_squared();
	fn get_f(&self, collision: &physics::collision::Collision, ) -> (Vec3, Quat) {
		(
			self.collision_stiffness * Vec3::new(
				collision.collision1.x - collision.collision2.x,
				collision.collision1.y - collision.collision2.y,
				collision.collision1.z - collision.collision2.z,
			),
			Quat::IDENTITY
		)

	}
	fn get_h(&self, collision: &physics::collision::Collision) -> Mat3 {
		self.collision_stiffness * Mat3::IDENTITY
	}
}

