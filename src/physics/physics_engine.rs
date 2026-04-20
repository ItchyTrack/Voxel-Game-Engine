use std::{cell::{Ref, RefCell}, collections::HashMap};

use glam::{I8Vec3, Vec3, IVec3};
use tracy_client::span;

use crate::{physics::{physics_body::{PhysicsBodyGridId, PhysicsBodyId}, solver::Impulse}, pose::Pose};

use super::{bvh::BVH, physics_body::{PhysicsBody}, solver::Solver, ball_joint_constraint::BallJointConstraint, voxel_tracker::VoxelTracker};

pub struct PhysicsEngine {
	physics_bodies: Vec<PhysicsBody>,
	physics_body_id_to_index: HashMap<PhysicsBodyId, u32>,
	constraints: HashMap<(PhysicsBodyId, PhysicsBodyId), BallJointConstraint>,
	impulses: HashMap<PhysicsBodyId, Vec<Impulse>>,
	next_body_id: PhysicsBodyId,
	solver: Solver,
	bvh: RefCell<Option<BVH<(u32, u32, u32)>>>,
	voxel_tracker: VoxelTracker,
}

macro_rules! get_bvh_macro {
	($self:expr) => {{
		if $self.bvh.borrow().is_none() {
			let mut bounds = vec![];
			{
				let _zone = span!("Collect aabb");
				for body_index in 0..$self.physics_bodies.len() {
					let physics_body = &$self.physics_bodies[body_index];
					for grid_index in 0..physics_body.grids().len() as u32 {
						for (sub_grid_index, _sub_grid) in physics_body.grid_by_index(grid_index).unwrap().sub_grids().iter().enumerate() {
							if let Some(bound) = physics_body.sub_grid_aabb_by_index(grid_index, sub_grid_index as u32) {
								bounds.push(((body_index as u32, grid_index as u32, sub_grid_index as u32), bound));
							}
						}
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
			constraints: HashMap::new(),
			impulses: HashMap::new(),
			next_body_id: PhysicsBodyId(0),
			solver: Solver::new(),
			bvh: RefCell::new(None),
			voxel_tracker: VoxelTracker::new(),
		}
	}

	pub fn update(&mut self, dt: f32) {
		{
			let _zone = span!("Clean up");
			let mut index = 0u32;
			while index < self.physics_bodies.len() as u32 {
				if self.physics_bodies[index as usize].clean_up() {
					self.remove_physics_body_by_index(index);
				} else {
					index += 1;
				}
			}
			*self.bvh.borrow_mut() = None; // have to clear afer cleanup
		}
		{
			let bvh = &get_bvh_macro!(self);
			let _zone = span!("Collect constraints");
			let constraints = self.constraints.iter_mut().filter_map(|((id1, id2), constraint)| {
				Some(((*self.physics_body_id_to_index.get(id1)?, *self.physics_body_id_to_index.get(id2)?), constraint))
			}).collect();
			drop(_zone);
			self.solver.solve(&mut self.physics_bodies, constraints, &self.impulses, dt, bvh);
			self.impulses.clear();
		}
		*self.bvh.borrow_mut() = None;
	}

	pub fn bvh(&self) -> Ref<'_, BVH<(u32, u32, u32)>> {
		self.get_bvh()
	}

	pub fn add_physics_body(&mut self) -> PhysicsBodyId {
		self.physics_body_id_to_index.insert(self.next_body_id, self.physics_bodies.len() as u32);
		self.physics_bodies.push(PhysicsBody::new(self.next_body_id));
		self.next_body_id.0 += 1;
		PhysicsBodyId(self.next_body_id.0 - 1)
	}

	pub fn remove_physics_body(&mut self, physics_body_id: PhysicsBodyId) {
		let index = match self.physics_body_id_to_index.remove(&physics_body_id) {
			Some(i) => i,
			None => return,
		};
		self.physics_bodies.swap_remove(index as usize);
		if index != self.physics_bodies.len() as u32 {
			let other_id = self.physics_bodies[index as usize].id();
			self.physics_body_id_to_index.insert(other_id, index);
		}
		self.constraints.retain(|&(k1, k2), _| {
			k1 != physics_body_id && k2 != physics_body_id
		});
	}
	pub fn remove_physics_body_by_index(&mut self, index: u32) {
		if let Some(physics_body) = self.physics_bodies.get(index as usize) {
			self.remove_physics_body(physics_body.id());
		}
	}

	pub fn physics_body(&self, physics_body_id: PhysicsBodyId) -> Option<&PhysicsBody> {
		let index = *self.physics_body_id_to_index.get(&physics_body_id)?;
		self.physics_bodies.get(index as usize)
	}

	pub fn physics_body_by_index(&self, physics_body_index: u32) -> Option<&PhysicsBody> {
		self.physics_bodies.get(physics_body_index as usize)
	}

	pub fn physics_body_mut(&mut self, physics_body_id: PhysicsBodyId) -> Option<&mut PhysicsBody> {
		let index = *self.physics_body_id_to_index.get(&physics_body_id)?;
		self.physics_bodies.get_mut(index as usize)
	}

	pub fn physics_body_by_index_mut(&mut self, physics_body_index: u32) -> Option<&mut PhysicsBody> {
		self.physics_bodies.get_mut(physics_body_index as usize)
	}

	pub fn physics_bodies(&self) -> &[PhysicsBody] {
		&self.physics_bodies
	}

	pub fn apply_central_impulse(&mut self, physics_body_id: PhysicsBodyId, impluse: &Vec3) {
		self.impulses.entry(physics_body_id).or_default().push(Impulse::CentralImpulse { central_impluse: *impluse });
	}
	pub fn apply_rotational_impulse(&mut self, physics_body_id: PhysicsBodyId, rotational_impluse: &Vec3) {
		self.impulses.entry(physics_body_id).or_default().push(Impulse::RotationalImpulse { rotational_impluse: *rotational_impluse });
	}
	pub fn apply_impulse(&mut self, physics_body_id: PhysicsBodyId, impluse_pos: &Vec3, impluse: &Vec3) {
		self.impulses.entry(physics_body_id).or_default().push(Impulse::Impulse { impluse: *impluse, impluse_pos: *impluse_pos });
	}

	pub fn create_ball_joint_constraint(&mut self, physics_body_id_1: PhysicsBodyId, body_1_attachment: &Pose, physics_body_id_2: PhysicsBodyId, body_2_attachment: &Pose) {
		if self.physics_body_id_to_index.contains_key(&physics_body_id_1) && self.physics_body_id_to_index.contains_key(&physics_body_id_2) {
			self.constraints.insert(
				if physics_body_id_1.0 < physics_body_id_2.0 { (physics_body_id_1, physics_body_id_2) } else { (physics_body_id_2, physics_body_id_1) },
				BallJointConstraint::new(body_1_attachment, body_2_attachment, f32::INFINITY, 0.0)
			);
		}
	}

	pub fn create_ball_joint_spring_constraint(&mut self, physics_body_id_1: PhysicsBodyId, body_1_attachment: &Pose, physics_body_id_2: PhysicsBodyId, body_2_attachment: &Pose, stiffness : f32) {
		if self.physics_body_id_to_index.contains_key(&physics_body_id_1) && self.physics_body_id_to_index.contains_key(&physics_body_id_2) {
			self.constraints.insert(
				if physics_body_id_1.0 < physics_body_id_2.0 { (physics_body_id_1, physics_body_id_2) } else { (physics_body_id_2, physics_body_id_1) },
				BallJointConstraint::new(body_1_attachment, body_2_attachment, stiffness, 0.0)
			);
		}
	}

	pub fn raycast(&self, pose: &Pose, max_length: Option<f32>) -> Option<(u32, u32, IVec3, I8Vec3, f32)> {
		let mut best_hit: Option<(u32, u32, IVec3, I8Vec3, f32)> = None;
		for ((body_index, grid_index, sub_grid_index), bvh_distance) in self.get_bvh().raycast(pose, max_length) {
			if best_hit.is_some() && bvh_distance > best_hit.unwrap().4 { break; }
			let physics_body = &self.physics_bodies[body_index as usize];
			let grid = physics_body.grid_by_index(grid_index).unwrap();
			let sub_grid = grid.sub_grid_by_index(sub_grid_index).unwrap();
			if let Some((hit_pos, hit_normal, grid_distance)) = sub_grid.get_voxels().get_voxels().raycast(
				&(&Pose::from_translation(-grid.sub_grid_pos_to_grid_pos(&sub_grid.sub_grid_ipos()).as_vec3()) * grid.pose.inverse() * physics_body.pose.inverse() * Pose::new(
					pose.translation + pose.rotation * Vec3::Z * bvh_distance, pose.rotation)),
				max_length.map(|max_length| max_length - bvh_distance),
				// &(&physics_body.pose * grid.pose * Pose::from_translation(grid.sub_grid_pos_to_grid_pos(&sub_grid_pos).as_vec3()))
			) {
				if best_hit.is_none() || grid_distance + bvh_distance < best_hit.unwrap().4 {
					best_hit = Some((body_index, grid_index, hit_pos.as_ivec3() + grid.sub_grid_pos_to_grid_pos(&sub_grid.sub_grid_ipos()), hit_normal, grid_distance + bvh_distance));
				}
			}
		}
		best_hit
	}

	pub fn get_bvh(&'_ self) -> Ref<'_, BVH<(u32, u32, u32)>> {
		if self.bvh.borrow().is_none() {
			let mut bounds = vec![];
			{
				let _zone = span!("Collect aabb");
				for body_index in 0..self.physics_bodies.len() {
					let physics_body = &self.physics_bodies[body_index];
					for grid_index in 0..physics_body.grids().len() as u32 {
						for (sub_grid_index, _sub_grid) in physics_body.grid_by_index(grid_index).unwrap().sub_grids().iter().enumerate() {
							if let Some(bound) = physics_body.sub_grid_aabb_by_index(grid_index, sub_grid_index as u32) {
								bounds.push(((body_index as u32, grid_index as u32, sub_grid_index as u32), bound));
							}
						}
					}
				}
			}
			*self.bvh.borrow_mut() = Some(BVH::new(bounds));
		}
		Ref::map(self.bvh.borrow(), |bvh| { bvh.as_ref().unwrap() })
	}

	pub fn start_tracking(&mut self, body_id: PhysicsBodyId, grid_id: PhysicsBodyGridId, voxel: IVec3) -> u64 {
		self.voxel_tracker.start_tracking(body_id, grid_id, voxel)
	}
	pub fn stop_tracking(&mut self, tracked_voxel_id: u64) {
		self.voxel_tracker.stop_tracking(tracked_voxel_id);
	}
	pub fn get_tracked_voxel(&self, tracked_voxel_id: u64) -> Option<(PhysicsBodyId, PhysicsBodyGridId, IVec3)> {
		let tracked_voxel = self.voxel_tracker.get_tracked_voxel(tracked_voxel_id)?;
		Some((tracked_voxel.body_id, tracked_voxel.grid_id, tracked_voxel.voxel))
	}
}
