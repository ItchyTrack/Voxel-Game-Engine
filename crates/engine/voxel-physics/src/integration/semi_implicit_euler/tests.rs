use super::*;
use bevy::math::{DMat3, DVec3};
use voxel_mass::InertiaTensor;
use voxel_math::{Fixed, FixedVec3};
use std::time::Duration;

fn world(inertia: DMat3, transform: Transform) -> (World, Entity, Schedule) {
	let mut world = World::new();
	let mut time = Time::<()>::default();
	time.advance_by(Duration::from_nanos(1_000_000_000 / 120));
	world.insert_resource(time);
	world.init_resource::<Impulses>();
	world.init_resource::<Accelerations>();
	let body = world.spawn((RigidBody, transform, Mass(400_000), RotationalInertia(InertiaTensor::from_mat3(inertia)), CenterOfMass::default())).id();
	let mut schedule = Schedule::default();
	schedule.add_systems(integrate_physics_center_of_mass_transforms);
	(world, body, schedule)
}

#[test]
fn rotational_impulses_survive_large_inertia_in_the_integration_system() {
	for (inertia, impulse) in [(1e12, 100_000_000u64), (1e14, 10_000_000_000)] {
		let (mut world, body, mut schedule) = world(DMat3::IDENTITY * inertia, Transform::IDENTITY);
		world.resource_mut::<Impulses>().apply_rotational_impulse(body, FixedVec3::Z * Fixed::from_num(impulse));
		schedule.run(&mut world);
		let integrated = world.get::<PhysicsIntegratedCenterOfMassTransform>(body).unwrap().0;
		let angle = FixedVec3::from_vec3(integrated.rotation.to_scaled_axis());
		assert!(angle.z > Fixed::ZERO);
		let expected = Fixed::from_num(0.0001) * crate::math::fixed_duration(world.resource::<Time>().delta());
		assert!(angle.z.abs_diff_eq(expected, Fixed::EPSILON));
	}
}

#[test]
fn off_center_impulse_uses_wide_torque_after_local_subtraction() {
	let transform = Transform::from_xyz(100_000_000_000i64, 0, 0);
	let (mut world, body, mut schedule) = world(DMat3::IDENTITY * 1e14, transform);
	let impulse_pos = transform.translation + FixedVec3::X * Fixed::from_num(1_000_000);
	world.resource_mut::<Impulses>().apply_impulse(body, impulse_pos, FixedVec3::Y * Fixed::from_num(100_000_000));
	schedule.run(&mut world);
	let integrated = world.get::<PhysicsIntegratedCenterOfMassTransform>(body).unwrap().0;
	let angle = FixedVec3::from_vec3(integrated.rotation.to_scaled_axis());
	let dt = crate::math::fixed_duration(world.resource::<Time>().delta());
	assert!(angle.z.abs_diff_eq(dt, Fixed::EPSILON));
	assert!(integrated.translation.y.abs_diff_eq(Fixed::from_num(250) * dt, Fixed::EPSILON));
	assert_eq!(integrated.translation.x, transform.translation.x);
}

#[test]
fn singular_inertia_does_not_invert_zero_axes() {
	let (mut world, body, mut schedule) = world(DMat3::from_diagonal(DVec3::new(1e12, 0.0, 1e12)), Transform::IDENTITY);
	world.resource_mut::<Impulses>().apply_rotational_impulse(body, FixedVec3::ONE * Fixed::from_num(100_000_000));
	schedule.run(&mut world);
	let angle = world.get::<PhysicsIntegratedCenterOfMassTransform>(body).unwrap().0.rotation.to_scaled_axis();
	assert!(angle.is_finite());
	assert!(angle.x > 0.0 && angle.z > 0.0);
	assert_eq!(angle.y, 0.0);
}
