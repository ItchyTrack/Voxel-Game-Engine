use bevy::prelude::*;

use crate::inertia_tensor::InertiaTensor;

/// Marker for an entity participating in the physics simulation.
///
/// A `RigidBody` entity is expected to also carry `Transform`, `Velocity`,
/// `AngularVelocity`, `Mass`, and `RotationalInertia`. Its `Grid` children
/// hold the voxel data attached to this body.
#[derive(Component, Default, Debug, Clone, Copy)]
pub struct RigidBody;

#[derive(Component, Default, Debug, Clone, Copy)]
pub struct Velocity(pub Vec3);

#[derive(Component, Default, Debug, Clone, Copy)]
pub struct AngularVelocity(pub Vec3);

#[derive(Component, Debug, Clone, Copy)]
pub struct Mass(pub f32);

impl Default for Mass {
	fn default() -> Self { Self(1.0) }
}

#[derive(Component, Debug, Clone, Copy)]
pub struct RotationalInertia(pub InertiaTensor);

impl Default for RotationalInertia {
	fn default() -> Self { Self(InertiaTensor::ZERO) }
}

/// Body is pinned in place: gravity, velocity, and impulses are ignored.
#[derive(Component, Default, Debug, Clone, Copy)]
pub struct IsStatic;
