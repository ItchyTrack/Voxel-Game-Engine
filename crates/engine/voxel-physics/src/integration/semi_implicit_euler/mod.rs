use voxel_transform::{Scale, Transform};
use crate::math::{Mat3, Mat6, Vec6, Wide, WideVec3, solve_symmetric};
use bevy::prelude::*;
use voxel_data::grid::Grid;
use voxel_mass::{CenterOfMass, Mass, RotationalInertia};

use crate::components::{AngularVelocity, IsStatic, RigidBody, Velocity};
use crate::solving::{Accelerations, Impulse, Impulses};
use crate::VoxelPhysicsAppExt;

use super::PhysicsIntegratedCenterOfMassTransform;

#[cfg(test)]
mod tests;

#[derive(Default)]
pub struct SemiImplicitEulerPlugin;

impl bevy::app::Plugin for SemiImplicitEulerPlugin {
	fn build(&self, app: &mut App) {
		app.add_physics_integration_systems(integrate_physics_center_of_mass_transforms);
	}
}

pub fn integrate_physics_center_of_mass_transforms(
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
	let dt = crate::math::fixed_duration(time.delta());
	if !crate::math::usable_timestep(dt) { return; }

	for (entity, transform, mut integrated_center_of_mass_transform, velocity, angular_velocity, mass, inertia, com, is_static) in bodies.iter_mut() {
		crate::math::require_unit_scale(transform);
		if is_static || mass.0 == 0 {
			integrated_center_of_mass_transform.0 = Transform {
				translation: transform.translation + transform.rotation * com.0,
				rotation: transform.rotation,
				scale: transform.scale,
			};
			continue;
		}

		let mut acceleration = WideVec3::ZERO;
		if let Some(body_accelerations) = accelerations.map.get(&entity) {
			for acc in body_accelerations {
				acceleration += WideVec3::from(*acc);
			}
		}

		let dt = Wide::from(dt);
		let mut velocity = WideVec3::from(velocity.0) + acceleration * dt;
		let mut angular_velocity = WideVec3::from(angular_velocity.0);
		let mut angular_impulse = WideVec3::ZERO;
		let global_center_of_mass = *transform * com.0;

		if let Some(body_impulses) = impulses.map.get(&entity) {
			for impulse in body_impulses {
				match impulse {
					Impulse::Impulse { impulse, impulse_pos } => {
						velocity += WideVec3::from(*impulse) / Wide::from_num(mass.0);
						angular_impulse += WideVec3::from(*impulse_pos - global_center_of_mass).cross((*impulse).into());
					},
					Impulse::CentralImpulse { central_impulse } => {
						velocity += WideVec3::from(*central_impulse) / Wide::from_num(mass.0);
					},
					Impulse::RotationalImpulse { rotational_impulse } => {
						angular_impulse += WideVec3::from(*rotational_impulse);
					},
				}
			}
		}

		if angular_impulse != WideVec3::ZERO {
			// Solve with inertia itself: a tiny inverse must not be rounded away.
			let rotational_inertia = Mat3::from_inertia(inertia.0.get_rotated(transform.rotation.as_dquat()).mat);
			let matrix = Mat6::from_mat3(Mat3::ZERO, Mat3::ZERO, Mat3::ZERO, rotational_inertia);
			angular_velocity += solve_symmetric(matrix, Vec6::from_vec3(WideVec3::ZERO, angular_impulse)).lower_vec3();
		}
		let next_rotation = crate::math::rotation_step(transform.rotation, (angular_velocity * dt).to_fixed());
		integrated_center_of_mass_transform.0 = Transform {
			translation: transform.translation + (velocity * dt).to_fixed() + transform.rotation * com.0,
			rotation: next_rotation,
			scale: Scale::ONE,
		};
	}
}
