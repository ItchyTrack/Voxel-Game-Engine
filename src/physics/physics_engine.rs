use std::collections::{HashMap};

use super::{physics_body::{PhysicsBody}, solver::Solver};

pub struct PhysicsEngine {
	physics_bodies: Vec<PhysicsBody>,
	physics_body_id_to_index: HashMap<u32, u32>,
	next_body_id: u32,
	solver: Solver,
}

impl PhysicsEngine {
	pub fn new() -> Self {
		Self {
			physics_bodies: vec![],
			physics_body_id_to_index: HashMap::new(),
			next_body_id: 0,
			solver: Solver::new(),
		}
	}

	pub fn update(&mut self, dt: f32) {
		self.solver.solve(&mut self.physics_bodies, dt)
	}

	pub fn add_physics_body(&mut self) -> u32 {
		self.physics_body_id_to_index.insert(self.next_body_id, self.physics_bodies.len() as u32);
		self.physics_bodies.push(PhysicsBody::new(self.next_body_id));
		self.next_body_id += 1;
		self.next_body_id - 1
	}

	pub fn remove_physics_body(&mut self, physics_body_id: u32) {
		let index = match self.physics_body_id_to_index.remove(&physics_body_id) {
			Some(i) => i,
			None => return,
		};
		self.physics_bodies.swap_remove(index as usize);
		if index != self.physics_bodies.len() as u32 {
			let other_id = self.physics_bodies[index as usize].id();
			self.physics_body_id_to_index.insert(other_id, index);
		}
	}

	pub fn physics_body(&self, physics_body_id: u32) -> Option<&PhysicsBody> {
		let index = *self.physics_body_id_to_index.get(&physics_body_id)?;
		self.physics_bodies.get(index as usize)
	}

	pub fn physics_body_mut(&mut self, physics_body_id: u32) -> Option<&mut PhysicsBody> {
		let index = *self.physics_body_id_to_index.get(&physics_body_id)?;
		self.physics_bodies.get_mut(index as usize)
	}

	pub fn physics_bodies(&self) -> &[PhysicsBody] {
		&self.physics_bodies
	}
}
