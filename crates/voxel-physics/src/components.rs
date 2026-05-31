use bevy::prelude::*;

use crate::inertia_tensor::InertiaTensor;

// ----------------- Grid -----------------

#[derive(Component, Default, Debug, Clone, Copy)]
pub struct VoxelCollider;

#[derive(Component, Default, Debug, Clone, Copy)]
pub struct VoxelMass;

// ----------------- RigidBody -----------------

#[derive(Component, Default, Debug, Clone, Copy)]
#[require(Transform, Velocity, AngularVelocity, Mass, RotationalInertia, CenterOfMass)]
pub struct RigidBody;

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

