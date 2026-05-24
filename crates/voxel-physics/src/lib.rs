pub mod collision;
pub mod components;
pub mod inertia_tensor;
pub mod math;
mod solver;

use bevy::ecs::schedule::IntoScheduleConfigs;
use bevy::prelude::*;

pub use components::{AngularVelocity, IsStatic, Mass, RigidBody, RotationalInertia, Velocity};
pub use inertia_tensor::InertiaTensor;

#[derive(Resource, Debug, Clone, Copy)]
pub struct Gravity(pub Vec3);

impl Default for Gravity {
	fn default() -> Self { Self(Vec3::new(0.0, -98.0, 0.0)) }
}

#[derive(Default)]
pub struct VoxelPhysicsPlugin;

impl Plugin for VoxelPhysicsPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<Gravity>()
			.init_resource::<collision::PhysicsBvh>()
			.init_resource::<collision::ContactList>()
			.add_systems(FixedUpdate, (
				solver::integrate,
				collision::rebuild_bvh,
				collision::detect_contacts,
				collision::resolve_contacts,
			).chain());
	}
}
