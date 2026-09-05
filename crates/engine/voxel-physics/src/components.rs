use bevy::prelude::*;
use serde::{Deserialize, Serialize};
use voxel_mass::{BodyMassError, CenterOfMass, Mass, RotationalInertia};

use crate::integration::PhysicsIntegratedCenterOfMassTransform;

// ----------------- Grid -----------------

#[derive(Component, Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct VoxelCollider;

// ----------------- RigidBody -----------------

#[derive(Component, Default, Debug, Clone, Copy)]
#[require(Transform, PhysicsIntegratedCenterOfMassTransform, Velocity, AngularVelocity, Mass, RotationalInertia, CenterOfMass, BodyMassError)]
pub struct RigidBody;

#[derive(Component, Default, Debug, Clone, Copy)]
pub struct Velocity(pub Vec3);

#[derive(Component, Default, Debug, Clone, Copy)]
pub struct AngularVelocity(pub Vec3);

#[derive(Component, Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct IsStatic;

