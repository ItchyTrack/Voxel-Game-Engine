pub mod semi_implicit_euler;

pub use semi_implicit_euler::SemiImplicitEulerPlugin;

use bevy::prelude::*;

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

