use voxel_math::FixedVec3;
use voxel_transform::Transform;

use voxel_mass::InertiaTensor;

/// Per-step solver scratch for one rigid body, gathered from its ECS components
/// at the start of the solver and written back at the end.
pub struct SolverBody {
	pub transform: Transform,
	pub integrated_center_of_mass_transform: Transform,
	pub velocity: FixedVec3,
	pub angular_velocity: FixedVec3,
	pub is_static: bool,
	pub mass: u64,
	pub center_of_mass: FixedVec3,
	pub rotational_inertia: InertiaTensor,
}

impl SolverBody {
	pub fn new() -> Self {
		Self {
			transform: Transform::IDENTITY,
			integrated_center_of_mass_transform: Transform::IDENTITY,
			velocity: FixedVec3::ZERO,
			angular_velocity: FixedVec3::ZERO,
			is_static: false,
			mass: 0,
			center_of_mass: FixedVec3::ZERO,
			rotational_inertia: InertiaTensor::ZERO,
		}
	}

	pub fn mass(&self) -> u64 { self.mass }
	pub fn local_center_of_mass(&self) -> FixedVec3 { self.center_of_mass }
	pub fn global_rotated_center_of_mass(&self) -> FixedVec3 { self.transform.rotation * self.center_of_mass }
	pub fn rotational_inertia(&self) -> InertiaTensor {
		self.rotational_inertia.get_rotated(self.transform.rotation.as_dquat())
	}
}
