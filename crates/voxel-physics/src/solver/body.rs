use bevy::transform::components::Transform;
use bevy::math::Vec3;

use crate::PhysicsBodyId;
use crate::inertia_tensor::InertiaTensor;

/// Per-step solver scratch for one rigid body, gathered from its ECS components
/// at the start of [`crate::solver::Solver::solve`] and written back at the end.
pub struct SolverBody {
	pub transform: Transform,
	pub velocity: Vec3,
	pub angular_velocity: Vec3,
	pub is_static: bool,
	entity: PhysicsBodyId,
	pub mass: f32,
	pub center_of_mass: Vec3,
	pub rotational_inertia: InertiaTensor,
}

impl SolverBody {
	pub fn new(entity: PhysicsBodyId) -> Self {
		Self {
			transform: Transform::IDENTITY,
			velocity: Vec3::ZERO,
			angular_velocity: Vec3::ZERO,
			is_static: false,
			entity,
			mass: 0.0,
			center_of_mass: Vec3::ZERO,
			rotational_inertia: InertiaTensor::ZERO,
		}
	}

	pub fn id(&self) -> PhysicsBodyId { self.entity }
	pub fn mass(&self) -> f32 { self.mass }
	pub fn local_center_of_mass(&self) -> Vec3 { self.center_of_mass }
	pub fn global_rotated_center_of_mass(&self) -> Vec3 { self.transform.rotation * self.center_of_mass }
	pub fn global_center_of_mass(&self) -> Vec3 { self.transform * self.center_of_mass }
	pub fn rotational_inertia(&self) -> InertiaTensor {
		self.rotational_inertia.get_rotated(self.transform.rotation.as_dquat())
	}
}
