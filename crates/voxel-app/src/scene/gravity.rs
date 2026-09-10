use bevy::prelude::*;
use voxel_math::{Fixed, FixedVec3};

use voxel_physics::{Accelerations, IsStatic, RigidBody, VoxelPhysicsAppExt};

pub struct GravityPlugin;

const GRAVITY_ACCELERATION: FixedVec3 = FixedVec3::new(Fixed::ZERO, Fixed::from_bits(-150 << 24), Fixed::ZERO);

impl Plugin for GravityPlugin {
	fn build(&self, app: &mut App) {
		app.add_physics_apply_systems(apply_gravity);
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
