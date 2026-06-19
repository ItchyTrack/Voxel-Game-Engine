use bevy::prelude::*;
use voxel_data::grid::Grid;

use crate::components::{AngularVelocity, CenterOfMass, IsStatic, Mass, PhysicsIntegratedCenterOfMassTransform, RigidBody, RotationalInertia, Velocity};
use super::{Accelerations, Impulse, Impulses};

pub(super) fn integrate_physics_center_of_mass_transforms(
	time: Res<Time>,
	impulses: Res<Impulses>,
	accelerations: Res<Accelerations>,
	mut bodies: Query<(
		Entity,
		&Transform,
		&mut PhysicsIntegratedCenterOfMassTransform,
		&Velocity,
		&AngularVelocity,
		&Mass,
		&RotationalInertia,
		&CenterOfMass,
		Has<IsStatic>,
	), (With<RigidBody>, Without<Grid>)>,
) {
	let dt = time.delta_secs();
	if dt <= 0.0 { return; }

	for (entity, transform, mut integrated_center_of_mass_transform, velocity, angular_velocity, mass, inertia, com, is_static) in bodies.iter_mut() {
		if is_static || mass.0 < f32::EPSILON {
			integrated_center_of_mass_transform.0 = Transform {
				translation: transform.translation + transform.rotation * com.0,
				rotation: transform.rotation,
				scale: transform.scale,
			};
			continue;
		}

		let mut acceleration = Vec3::ZERO;
		if let Some(body_accelerations) = accelerations.map.get(&entity) {
			for acc in body_accelerations {
				acceleration += *acc;
			}
		}

		let mut velocity = velocity.0 + acceleration * dt;
		let mut angular_velocity = angular_velocity.0;
		let rotational_inertia_inverse = inertia.0.get_rotated(transform.rotation.as_dquat()).mat.as_mat3().inverse();
		let global_center_of_mass = *transform * com.0;

		if let Some(body_impulses) = impulses.map.get(&entity) {
			for impulse in body_impulses {
				match impulse {
					Impulse::Impulse { impulse, impulse_pos } => {
						velocity += impulse / mass.0;
						angular_velocity += rotational_inertia_inverse * (impulse_pos - global_center_of_mass).cross(*impulse);
					},
					Impulse::CentralImpulse { central_impulse } => {
						velocity += central_impulse / mass.0;
					},
					Impulse::RotationalImpulse { rotational_impulse } => {
						angular_velocity += rotational_inertia_inverse * rotational_impulse;
					},
				}
			}
		}

		let next_rotation = (Quat::from_scaled_axis(angular_velocity * dt) * transform.rotation).normalize();
		integrated_center_of_mass_transform.0 = Transform {
			translation: transform.translation
				+ velocity * dt
				+ transform.rotation * com.0,
			rotation: next_rotation,
			scale: Vec3::ONE,
		};
	}
}
