use std::{cell::{Ref, RefCell}, collections::HashMap};

use glam::{I8Vec3, I16Vec3, Vec3};

use crate::{pose::Pose};

use super::{bvh::BVH, physics_body::{PhysicsBody}, solver::Solver};

pub struct PhysicsEngine {
	physics_bodies: Vec<PhysicsBody>,
	physics_body_id_to_index: HashMap<u32, u32>,
	next_body_id: u32,
	solver: Solver,
	bvh: RefCell<Option<BVH<(u32, u32)>>>,
}

macro_rules! get_bvh_macro {
	($self:expr) => {{
		if $self.bvh.borrow().is_none() {
			let mut bounds = vec![];
			for body_index in 0..$self.physics_bodies.len() {
				let physics_body = &$self.physics_bodies[body_index];
				for grid_index in 0..physics_body.grids().len() {
					if let Some(bound) = physics_body.grid_aabb(grid_index as u32) {
						bounds.push(((body_index as u32, grid_index as u32), bound));
					}
				}
			}
			*$self.bvh.borrow_mut() = Some(BVH::new(bounds));
		}
		Ref::map($self.bvh.borrow(), |bvh| bvh.as_ref().unwrap())
	}};
}

impl PhysicsEngine {
	pub fn new() -> Self {
		Self {
			physics_bodies: vec![],
			physics_body_id_to_index: HashMap::new(),
			next_body_id: 0,
			solver: Solver::new(),
			bvh: RefCell::new(None),
		}
	}

	pub fn update(&mut self, dt: f32) {
		{
			let bvh = &get_bvh_macro!(self);
			self.solver.solve(&mut self.physics_bodies, dt, bvh);
		}
		*self.bvh.borrow_mut() = None;
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

	pub fn physics_body_by_index(&self, physics_body_index: u32) -> Option<&PhysicsBody> {
		self.physics_bodies.get(physics_body_index as usize)
	}

	pub fn physics_body_mut(&mut self, physics_body_id: u32) -> Option<&mut PhysicsBody> {
		let index = *self.physics_body_id_to_index.get(&physics_body_id)?;
		self.physics_bodies.get_mut(index as usize)
	}

	pub fn physics_bodies(&self) -> &[PhysicsBody] {
		&self.physics_bodies
	}

	pub fn raycast(&self, pose: &Pose, max_length: Option<f32>) -> Option<(u32, u32, I16Vec3, I8Vec3, f32)> {
		let mut best_hit: Option<(u32, u32, I16Vec3, I8Vec3, f32)> = None;
		for ((body_index, grid_index), bvh_distance) in self.get_bvh().raycast(pose, max_length) {
			if best_hit.is_some() && bvh_distance > best_hit.unwrap().4 { break; }
			let physics_body = &self.physics_bodies[body_index as usize];
			let grid = physics_body.grid_by_index(grid_index).unwrap();
			if let Some((hit_pos, hit_normal, grid_distance)) = grid.get_voxels().get_voxels().raycast(
				&(grid.pose.inverse() * physics_body.pose.inverse() * Pose::new(pose.translation + pose.rotation * Vec3::Z * bvh_distance, pose.rotation)),
				max_length.map(|max_length| max_length - bvh_distance)
			) {
				if best_hit.is_none() || grid_distance + bvh_distance < best_hit.unwrap().4 {
					best_hit = Some((body_index, grid_index, hit_pos, hit_normal, grid_distance + bvh_distance));
				}
			}
		}
		best_hit
	}

	pub fn get_bvh(&'_ self) -> Ref<'_, BVH<(u32, u32)>> {
		if self.bvh.borrow().is_none() {
			let mut bounds = vec![];
			for body_index in 0..self.physics_bodies.len() {
				let physics_body = &self.physics_bodies[body_index];
				for grid_index in 0..physics_body.grids().len() {
					if let Some(bound) = physics_body.grid_aabb(grid_index as u32) {
						bounds.push(((body_index as u32, grid_index as u32), bound));
					}
				}
			}
			*self.bvh.borrow_mut() = Some(BVH::new(bounds));
		}
		Ref::map(self.bvh.borrow(), |bvh| { bvh.as_ref().unwrap() })
	}
}
