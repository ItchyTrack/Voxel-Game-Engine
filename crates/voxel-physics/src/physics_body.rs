use bevy::ecs::entity::Entity;
use bevy::transform::components::Transform;
use glam::{Quat, Vec3};

use crate::inertia_tensor::InertiaTensor;
use crate::transform_ext::TransformExt;

#[derive(Copy, Clone, Hash, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct PhysicsBodyId(pub Entity);

#[derive(Copy, Clone, Hash, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct GridId(pub Entity);

pub struct PhysicsBody {
	pub transform: Transform,
	pub velocity: Vec3,
	pub angular_velocity: Vec3,
	pub is_static: bool,
	id: PhysicsBodyId,
	grids: Vec<GridId>,
	pub mass: f32,
	pub center_of_mass: Vec3,
	pub rotational_inertia: InertiaTensor,
}

impl PhysicsBody {
	pub fn new(id: PhysicsBodyId) -> Self {
		Self {
			transform: Transform::IDENTITY,
			velocity: Vec3::ZERO,
			angular_velocity: Vec3::ZERO,
			is_static: false,
			id,
			grids: vec![],
			mass: 0.0,
			center_of_mass: Vec3::ZERO,
			rotational_inertia: InertiaTensor::ZERO,
		}
	}
	pub fn id(&self) -> PhysicsBodyId {
		self.id
	}
	pub fn mass(&self) -> f32 {
		self.mass
	}
	pub fn grids(&self) -> &[GridId] {
		&self.grids
	}
	pub fn push_grid(&mut self, grid_id: GridId) {
		self.grids.push(grid_id);
	}
	pub fn local_center_of_mass(&self) -> Vec3 {
		self.center_of_mass
	}
	pub fn global_rotated_center_of_mass(&self) -> Vec3 {
		self.transform.rotation * self.center_of_mass
	}
	pub fn global_center_of_mass(&self) -> Vec3 {
		self.transform * self.center_of_mass
	}
	pub fn rotational_inertia(&self) -> InertiaTensor {
		self.rotational_inertia.get_rotated(self.transform.rotation.as_dquat())
	}
	pub fn global_rotational_inertia(&self) -> InertiaTensor {
		self.rotational_inertia.get_rotated(self.transform.rotation.as_dquat())
	}

	pub fn world_to_local(&self, other: &Transform) -> Transform { self.transform.inverse() * *other }
	pub fn local_to_world(&self, other: &Transform) -> Transform { self.transform * *other }
	pub fn world_to_local_vec(&self, vec: &Vec3) -> Vec3 { self.transform.inverse() * *vec }
	pub fn local_to_world_vec(&self, vec: &Vec3) -> Vec3 { self.transform * *vec }
	pub fn world_to_local_rot(&self, rot: &Quat) -> Quat { self.transform.rotation.inverse() * *rot }
	pub fn local_to_world_rot(&self, rot: &Quat) -> Quat { self.transform.rotation * *rot }
}
