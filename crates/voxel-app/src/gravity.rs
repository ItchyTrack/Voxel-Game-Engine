use bevy::prelude::*;

use voxel_physics::{Accelerations, FreezePhysics, IsStatic, PhysicsSet, RigidBody};

pub struct GravityPlugin;

const GRAVITY_ACCELERATION: Vec3 = Vec3::new(0.0, -150.0, 0.0);

impl Plugin for GravityPlugin {
	fn build(&self, app: &mut App) {
		app.add_systems(
			FixedUpdate,
			apply_gravity
				.in_set(PhysicsSet::Apply)
				.run_if(|freeze: Res<FreezePhysics>| !freeze.0),
		);
	}
}

fn apply_gravity(
	mut accelerations: ResMut<Accelerations>,
	bodies: Query<Entity, (With<RigidBody>, Without<IsStatic>)>,
) {
	for body in bodies.iter() {
		accelerations.apply_central_acceleration(body, GRAVITY_ACCELERATION);
	}
}
