use super::*;
use crate::integration::semi_implicit_euler::integrate_physics_center_of_mass_transforms;
use crate::solving::{Accelerations, Impulses};
use crate::collision::{Collision, HalfCollision, CubeFeature};
use bevy::math::DMat3;
use bevy::tasks::{ComputeTaskPool, TaskPool};
use voxel_mass::InertiaTensor;
use voxel_math::{Fixed, FixedVec3};
use std::time::Duration;

#[test]
fn active_jointed_bodies_run_repeated_integration_and_solver_steps() {
	ComputeTaskPool::get_or_init(TaskPool::new);
	for inertia in [3_000_000u64, 100_000_000_000_000] {
		let mut world = World::new();
		let mut time = Time::<()>::default();
		time.advance_by(Duration::from_nanos(1_000_000_000 / 120));
		world.insert_resource(time);
		world.init_resource::<PhysicsSolver>();
		world.init_resource::<Impulses>();
		world.init_resource::<Accelerations>();
		world.init_resource::<Collisions>();
		let spawn = |world: &mut World, mass, x, speed| world.spawn((
			RigidBody, Transform::from_xyz(x, 0, 0), Mass(mass),
			RotationalInertia(InertiaTensor::from_mat3(DMat3::IDENTITY * inertia as f64)),
			CenterOfMass::default(), Velocity(FixedVec3::Y * Fixed::from_num(speed)),
		)).id();
		let a = spawn(&mut world, 100_000, 0, 1);
		let b = spawn(&mut world, 400_000, 20, -1);
		let attach_a = Transform::from_xyz(10, 0, 0);
		let attach_b = Transform::from_xyz(-10, 0, 0);
		world.spawn(BallJoint::new(a, b, &attach_a, &attach_b, Fixed::MAX, Fixed::ZERO));
		world.resource_mut::<Impulses>().apply_rotational_impulse(a, FixedVec3::Z * Fixed::from_num(if inertia > 1_000_000_000 { 10_000_000_000u64 } else { 30_000 }));
		let mut schedule = Schedule::default();
		schedule.add_systems((sync_ball_joint_constraints, integrate_physics_center_of_mass_transforms, solve_physics, clear_queued_impulses_and_accelerations).chain());
		for _ in 0..120 {
			for body in [a, b] {
				world.resource_mut::<Accelerations>().apply_central_acceleration(body, FixedVec3::Y * Fixed::from_num(-9.81));
			}
			schedule.run(&mut world);
		}
		let ta = *world.get::<Transform>(a).unwrap();
		let tb = *world.get::<Transform>(b).unwrap();
		assert!(ta.translation.y < Fixed::from_num(-1));
		assert!(tb.translation.y < Fixed::from_num(-1));
		assert!((ta * attach_a.translation - tb * attach_b.translation).length() < Fixed::from_num(0.1));
		for body in [a, b] {
			assert!(world.get::<Transform>(body).unwrap().rotation.is_normalized());
			assert!(world.get::<Velocity>(body).unwrap().0.length() < Fixed::from_num(100));
			assert!(world.get::<AngularVelocity>(body).unwrap().0.length() < Fixed::from_num(10));
		}
	}
}

#[test]
fn active_contacts_solve_and_warm_start_with_large_inertia() {
	ComputeTaskPool::get_or_init(TaskPool::new);
	let mut world = World::new();
	let mut time = Time::<()>::default();
	time.advance_by(Duration::from_nanos(1_000_000_000 / 120));
	world.insert_resource(time);
	world.init_resource::<PhysicsSolver>();
	world.init_resource::<Impulses>();
	world.init_resource::<Accelerations>();
	world.init_resource::<Collisions>();
	let dynamic = world.spawn((RigidBody, Transform::IDENTITY, Mass(400_000), CenterOfMass::default(), RotationalInertia(InertiaTensor::from_mat3(DMat3::IDENTITY * 1e14)))).id();
	let fixed = world.spawn((RigidBody, IsStatic, Transform::IDENTITY, Mass(0), CenterOfMass::default(), RotationalInertia::default())).id();
	let mut schedule = Schedule::default();
	schedule.add_systems((integrate_physics_center_of_mass_transforms, solve_physics, clear_queued_impulses_and_accelerations).chain());
	for _ in 0..10 {
		let transform = *world.get::<Transform>(dynamic).unwrap();
		world.resource_mut::<Collisions>().0 = [-1000, 1000].into_iter().map(|x| {
			let local = FixedVec3::new(Fixed::from_num(x), Fixed::ZERO, Fixed::ZERO);
			let part1 = HalfCollision { body_id: dynamic, grid_id: dynamic, voxel_pos: IVec3::new(x, 0, 0), feature: CubeFeature::Face { xyzs: 0 }, collision: transform * local, local_collision: local };
			let part2 = HalfCollision { body_id: fixed, grid_id: fixed, collision: local + FixedVec3::Y, local_collision: local + FixedVec3::Y, ..part1 };
			Collision { part1, part2 }
		}).collect();
		schedule.run(&mut world);
	}
	let transform = world.get::<Transform>(dynamic).unwrap();
	assert!(transform.translation.y > Fixed::ZERO);
	assert!(transform.translation.length() < Fixed::from_num(100));
	assert!(transform.rotation.is_normalized());
	assert_eq!(*world.get::<Transform>(fixed).unwrap(), Transform::IDENTITY);
}
