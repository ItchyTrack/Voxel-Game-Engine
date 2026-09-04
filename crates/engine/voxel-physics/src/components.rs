use bevy::prelude::*;
use serde::{Deserialize, Serialize};
use voxel_mass::BodyMassError;

use crate::inertia_tensor::InertiaTensor;
use crate::integration::PhysicsIntegratedCenterOfMassTransform;

// ----------------- Grid -----------------

#[derive(Component, Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct VoxelCollider;

#[derive(Component, Default, Debug, Clone, Copy)]
pub struct VoxelMass;

// ----------------- RigidBody -----------------

#[derive(Component, Default, Debug, Clone, Copy)]
#[require(Transform, PhysicsIntegratedCenterOfMassTransform, Velocity, AngularVelocity, Mass, RotationalInertia, CenterOfMass, BodyMassError)]
pub struct RigidBody;

#[derive(Component, Default, Debug, Clone, Copy)]
pub struct Velocity(pub Vec3);

#[derive(Component, Default, Debug, Clone, Copy)]
pub struct AngularVelocity(pub Vec3);

#[derive(Component, Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Mass(pub f32);

/// Local-space rotational
#[derive(Component, Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RotationalInertia(pub InertiaTensor);

/// Local-space offset from the body's transform origin to its center of mass.
#[derive(Component, Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CenterOfMass(pub Vec3);

#[derive(Component, Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct IsStatic;

