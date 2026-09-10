use voxel_math::FixedVec3;
use bevy::prelude::*;
use serde::{Deserialize, Serialize};
use voxel_transform::Body;

use crate::integration::PhysicsIntegratedCenterOfMassTransform;

// ----------------- Grid -----------------

#[derive(Component, Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct VoxelCollider;

// ----------------- RigidBody -----------------

#[derive(Component, Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[require(Body, PhysicsIntegratedCenterOfMassTransform, Velocity, AngularVelocity)]
pub struct RigidBody;

#[derive(Component, Default, Debug, Clone, Copy)]
pub struct Velocity(pub FixedVec3);

#[derive(Component, Default, Debug, Clone, Copy)]
pub struct AngularVelocity(pub FixedVec3);

#[derive(Component, Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct IsStatic;
