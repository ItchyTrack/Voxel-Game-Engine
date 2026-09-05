use bevy::{math::DVec3, prelude::*};
use serde::{Deserialize, Serialize};

use crate::InertiaTensor;

#[derive(Component, Default, Debug, Clone, Copy)]
pub struct VoxelMass;

/// Whether all of a body's grids that require voxel mass have been initialized.
#[derive(Component, Default, Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct BodyMassInitialized(pub bool);

#[derive(Component, Default, Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Mass(pub u64);

/// Local-space inertia tensor about the center of mass.
#[derive(Component, Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RotationalInertia(pub InertiaTensor);

/// Local-space offset from the transform origin to the center of mass.
#[derive(Component, Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CenterOfMass(pub DVec3);

impl CenterOfMass {
	pub fn get_transformed(self, transform: &Transform) -> Self {
		Self(transform.rotation.as_dquat() * self.0 + transform.translation.as_dvec3())
	}
}
