use bevy::math::Vec3;
use bevy::transform::components::Transform;

use voxel_mass::InertiaTensor;

/// Per-step solver scratch for one rigid body, gathered from its ECS components
/// at the start of [`crate::solving::Solver::solve`] and written back at the end.
pub struct SolverBody {
	pub transform: Transform,
	pub integrated_center_of_mass_transform: Transform,
	pub velocity: Vec3,
	pub angular_velocity: Vec3,
	pub is_static: bool,
	pub mass: f32,
	pub center_of_mass: Vec3,
	pub rotational_inertia: InertiaTensor,
}

impl SolverBody {
	pub fn new() -> Self {
		Self {
			transform: Transform::IDENTITY,
			integrated_center_of_mass_transform: Transform::IDENTITY,
			velocity: Vec3::ZERO,
			angular_velocity: Vec3::ZERO,
			is_static: false,
			mass: 0.0,
			center_of_mass: Vec3::ZERO,
			rotational_inertia: InertiaTensor::ZERO,
		}
	}

	pub fn mass(&self) -> f32 { self.mass }
	pub fn local_center_of_mass(&self) -> Vec3 { self.center_of_mass }
	pub fn global_rotated_center_of_mass(&self) -> Vec3 { self.transform.rotation * self.center_of_mass }
	pub fn rotational_inertia(&self) -> InertiaTensor {
		self.rotational_inertia.get_rotated(self.transform.rotation.as_dquat())
	}
}
