use std::f32::consts::PI;

use bevy::math::{IVec3, Quat, Vec3};
use bevy::prelude::*;

use voxel_data::voxels::Voxel;
use voxel_physics::components::{VoxelCollider, VoxelMass};
use voxel_physics::{
	AngularVelocity, BallJointConstraint, BallJointConstraints,
	FreezePhysics, Impulses, IsStatic, PhysicsSet,
	RigidBody, RotationalInertia,
};

use crate::streaming_test::{spawn_grid, StreamingVoxels, WorldStore};
#[cfg(not(target_arch = "wasm32"))]
use crate::vox_loader::load_vox;
#[cfg(target_arch = "wasm32")]
use crate::vox_loader::load_vox_bytes;

pub struct ScenePlugin;

impl Plugin for ScenePlugin {
	fn build(&self, app: &mut App) {
		app.add_systems(Startup, setup_scene)
			.add_systems(
				FixedUpdate,
				drive_orientation
					.in_set(PhysicsSet::Apply)
					.run_if(|freeze: Res<FreezePhysics>| !freeze.0),
			);
	}
}

#[derive(Component, Debug, Clone, Copy)]
pub struct Orientation {
	pub target: Quat,
	pub gain: f32,
	pub damping: f32,
}

impl Default for Orientation {
	fn default() -> Self {
		Self { target: Quat::IDENTITY, gain: 10.0, damping: 2.0 }
	}
}

fn drive_orientation(
	bodies: Query<(Entity, &Transform, &AngularVelocity, &RotationalInertia, &Orientation), With<RigidBody>>,
	mut impulses: ResMut<Impulses>,
) {
	for (entity, transform, angular_velocity, inertia, orientation) in bodies.iter() {
		let error = transform.rotation * orientation.target.inverse();
		let (axis, angle) = error.to_axis_angle();
		if !axis.is_finite() || angle.abs() < 1e-6 { continue; }
		let angular_in_dir = angular_velocity.0.dot(axis);
		let impulse = inertia.0.mat.as_mat3() * (
			axis * (-angle * orientation.gain - angular_in_dir * orientation.damping)
			- (angular_velocity.0 - axis * angular_in_dir)
		);
		impulses.apply_rotational_impulse(entity, impulse);
	}
}

fn setup_scene(
	mut commands: Commands,
	mut constraints: ResMut<BallJointConstraints>,
	mut store: ResMut<WorldStore>,
) {
	spawn_church(&mut commands, &mut store);
	// spawn_ball_cluster(&mut commands, &mut constraints, &mut store);
	// spawn_bb8(&mut commands, &mut constraints, &mut store, Vec3::new(0.0, 120.0, 0.0));
	// spawn_bb8(&mut commands, &mut constraints, &mut store, Vec3::new(30.0, 120.0, 0.0));
	// spawn_bb8(&mut commands, &mut constraints, &mut store, Vec3::new(-30.0, 120.0, 0.0));
	// for x in 0..3 {
	// 	for y in 0..2 {
	// 		for z in 0..3 {
	// 			spawn_bb8(&mut commands, &mut constraints, &mut store, Vec3::new(30.0 * x as f32, 30.0 * y as f32 + 200.0, 30.0 * z as f32));
	// 		}
	// 	}
	// }
}

fn spawn_church(commands: &mut Commands, store: &mut WorldStore) {
	let mut grid = StreamingVoxels::new();
	if !load_church(&mut grid) { return }

	// let parent = commands
	// 	.spawn((
	// 		RigidBody,
	// 		IsStatic,
	// 		Transform::from_translation(Vec3::new(0.0, -350.0, 0.0)),
	// 	))
	// 	.id();
	spawn_grid(commands, store, None, Transform::IDENTITY, grid, VoxelCollider);
}

#[cfg(target_arch = "wasm32")]
fn load_church(grid: &mut StreamingVoxels) -> bool {
	load_vox_bytes(grid, include_bytes!("../../../res/Church_Of_St_Sophia.vox"), Vec3::ZERO)
}

#[cfg(not(target_arch = "wasm32"))]
fn load_church(grid: &mut StreamingVoxels) -> bool {
	let candidate_paths = [
		std::path::PathBuf::from("res/Church_Of_St_Sophia.vox"),
		std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
			.join("../../res/Church_Of_St_Sophia.vox"),
	];
	candidate_paths.iter().any(|p| load_vox(grid, p, Vec3::ZERO))
}

fn spawn_ball_cluster(
	commands: &mut Commands,
	constraints: &mut BallJointConstraints,
	store: &mut WorldStore,
) {
	let r = 5;
	let base_y = 80.0;
	let base_z = -20.0;

	let main = spawn_ball(commands, store, Vec3::new(0.0, base_y, base_z), 2);
	let satellites = [
		(spawn_ball(commands, store, Vec3::new(0.0, base_y, base_z + 10.0), r), Vec3::new(0.0, 0.0, 10.0)),
		(spawn_ball(commands, store, Vec3::new(0.0, base_y, base_z - 10.0), r), Vec3::new(0.0, 0.0, -10.0)),
		(spawn_ball(commands, store, Vec3::new(10.0, base_y, base_z), r), Vec3::new(10.0, 0.0, 0.0)),
		(spawn_ball(commands, store, Vec3::new(-10.0, base_y, base_z), r), Vec3::new(-10.0, 0.0, 0.0)),
	];

	for (satellite, attachment) in satellites {
		constraints.insert(
			main,
			satellite,
			BallJointConstraint::new(
				&Transform::IDENTITY,
				&Transform::from_translation(attachment),
				f32::INFINITY,
				0.0,
			),
		);
	}
}

fn spawn_bb8(
	commands: &mut Commands,
	constraints: &mut BallJointConstraints,
	store: &mut WorldStore,
	position: Vec3,
) {
	let mut base_grid = StreamingVoxels::new();
	for x in -6..=6 { for y in 0..3 { for z in -6..=6 {
		base_grid.add_voxel(&IVec3::new(x, y, z), &Voxel { color: [128, 128, 128, 255], mass: 200 });
	}}}
	base_grid.add_voxel(&IVec3::new(0, 3, 0), &Voxel { color: [255, 0, 0, 255], mass: 200 });

	let base = commands.spawn((
		RigidBody,
		Orientation::default(),
		Transform::from_translation(position),
	)).id();
	spawn_grid(commands, store, Some(base), Transform::IDENTITY, base_grid, (VoxelCollider, VoxelMass));

	let ball = spawn_ball(commands, store, position - Vec3::new(0.0, 12.0, 0.0), 10);

	constraints.insert(
		base,
		ball,
		BallJointConstraint::new(
			&Transform::IDENTITY,
			&Transform::from_translation(Vec3::new(0.0, -12.0, 0.0)),
			f32::INFINITY,
			0.0,
		),
	);
}

fn spawn_ball(commands: &mut Commands, store: &mut WorldStore, position: Vec3, radius: i32) -> Entity {
	let radius_sq = (radius as f32 - 0.5).powi(2);

	let mut top = StreamingVoxels::new();
	for x in -radius..=radius {
		for y in 0..=radius {
			for z in -radius..=radius {
				let p = IVec3::new(x, y, z);
				if p.as_vec3().length_squared() > radius_sq { continue; }
				top.add_voxel(&p, &Voxel {
					color: [(x as u8 / 10) * 10, (y as u8 / 10) * 10, (z as u8 / 10) * 10, 255],
					mass: 100,
				});
			}
		}
	}

	let mut bottom = StreamingVoxels::new();
	for x in -radius..=radius {
		for y in -radius..0 {
			for z in -radius..=radius {
				let p = IVec3::new(x, y, z);
				if p.as_vec3().length_squared() > radius_sq { continue; }
				bottom.add_voxel(&p, &Voxel {
					color: [(x as u8 / 10) * 10, (y as u8 / 10) * 10, (z as u8 / 10) * 10, 255],
					mass: 100,
				});
			}
		}
	}

	let body = commands.spawn((
		RigidBody,
		Transform::from_translation(position),
	)).id();
	spawn_grid(commands, store, Some(body), Transform::from_translation(Vec3::new(-0.5, -0.5, -0.5)), top, (VoxelCollider, VoxelMass));
	spawn_grid(commands, store, Some(body), Transform {
			translation: Vec3::new(-std::f32::consts::FRAC_1_SQRT_2, -0.5, 0.0),
			rotation: Quat::from_rotation_y(PI / 4.0),
			scale: Vec3::ONE,
		}, bottom, (VoxelCollider, VoxelMass));
	body
}
