use bevy::prelude::*;
use serde::{Deserialize, Serialize};

use crate::InertiaTensor;

#[derive(Component, Default, Debug, Clone, Copy)]
pub struct VoxelMass;

#[derive(Component, Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Mass(pub f32);

/// Local-space rotational
#[derive(Component, Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RotationalInertia(pub InertiaTensor);

/// Local-space offset from the body's transform origin to its center of mass.
#[derive(Component, Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CenterOfMass(pub Vec3);
