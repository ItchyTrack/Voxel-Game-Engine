use bevy::prelude::*;

use crate::inertia_tensor::InertiaTensor;

// ----------------- Grid -----------------

#[derive(Component, Default, Debug, Clone, Copy)]
pub struct VoxelCollider;

#[derive(Component, Default, Debug, Clone, Copy)]
pub struct VoxelMass;

// ----------------- RigidBody -----------------

#[derive(Component, Default, Debug, Clone, Copy)]
#[require(Transform, PhysicsIntegratedCenterOfMassTransform, Velocity, AngularVelocity, Mass, RotationalInertia, CenterOfMass)]
pub struct RigidBody;

#[derive(Component, Debug, Clone, Copy)]
pub struct PhysicsIntegratedCenterOfMassTransform(pub Transform);

impl PhysicsIntegratedCenterOfMassTransform {
	pub const IDENTITY: Self = Self(Transform::IDENTITY);

	pub fn new(transform: Transform) -> Self { Self(transform) }
}

impl Default for PhysicsIntegratedCenterOfMassTransform {
	fn default() -> Self { Self::IDENTITY }
}

impl From<Transform> for PhysicsIntegratedCenterOfMassTransform {
	fn from(transform: Transform) -> Self { Self(transform) }
}

#[derive(Component, Default, Debug, Clone, Copy)]
pub struct Velocity(pub Vec3);

#[derive(Component, Default, Debug, Clone, Copy)]
pub struct AngularVelocity(pub Vec3);

#[derive(Component, Default, Debug, Clone, Copy)]
pub struct Mass(pub f32);

/// Local-space rotational
#[derive(Component, Default, Debug, Clone, Copy)]
pub struct RotationalInertia(pub InertiaTensor);

/// Local-space offset from the body's transform origin to its center of mass.
#[derive(Component, Default, Debug, Clone, Copy)]
pub struct CenterOfMass(pub Vec3);

#[derive(Component, Default, Debug, Clone, Copy)]
pub struct IsStatic;

