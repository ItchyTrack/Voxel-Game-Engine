use std::f32::consts::PI;

use bevy::math::{IVec3, Quat, Vec3};
use bevy::prelude::*;

use voxel_data::grid::Grid;
use voxel_data::voxels::Voxel;
use voxel_physics::components::{VoxelCollider, VoxelMass};
use voxel_physics::{
	AngularVelocity, BallJointConstraint, BallJointConstraints,
	FreezePhysics, Impulses, IsStatic, PhysicsSet,
	RigidBody, RotationalInertia,
};

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

fn setup_scene(mut commands: Commands, mut constraints: ResMut<BallJointConstraints>) {
	spawn_church(&mut commands);
	spawn_ball_cluster(&mut commands, &mut constraints);
	spawn_bb8(&mut commands, &mut constraints, Vec3::new(0.0, 120.0, 0.0));
	spawn_bb8(&mut commands, &mut constraints, Vec3::new(30.0, 120.0, 0.0));
	spawn_bb8(&mut commands, &mut constraints, Vec3::new(-30.0, 120.0, 0.0));
	for x in 0..3 {
		for y in 0..1 {
			for z in 0..3 {
				spawn_bb8(&mut commands, &mut constraints, Vec3::new(30.0 * x as f32, 30.0 * y as f32 + 200.0, 30.0 * z as f32));
			}
		}
	}
}

fn spawn_church(commands: &mut Commands) {
	let candidate_paths = [
		std::path::PathBuf::from("res/Church_Of_St_Sophia.vox"),
		std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
			.join("../../res/Church_Of_St_Sophia.vox"),
	];
	let Some(bytes) = candidate_paths.iter().find_map(|p| std::fs::read(p).ok()) else { return };
	let Ok(dot_vox_data) = dot_vox::load_bytes(&bytes) else { return };

	let mut grid = Grid::new();

	#[derive(Clone, Copy)]
	struct Frame { translation: Vec3, rotation: Quat, flip: IVec3 }
	let mut stack: Vec<(u32, Frame)> = vec![(0, Frame {
		translation: Vec3::ZERO,
		rotation: Quat::IDENTITY,
		flip: IVec3::new(1, 1, -1),
	})];
	while let Some((scene_id, pose)) = stack.pop() {
		let Some(node) = dot_vox_data.scenes.get(scene_id as usize) else { continue };
		match node {
			dot_vox::SceneNode::Transform { frames, child, .. } => {
				let Some(frame) = frames.first() else { continue };
				let pos = frame.position().unwrap_or(dot_vox::Position { x: 0, y: 0, z: 0 });
				let (rot, flip_vec) = frame.orientation().and_then(|q| {
					let (qarr, varr) = q.to_quat_scale();
					let q = Quat::from_array(qarr);
					let q = Quat::from_xyzw(q.x, q.z, -q.y, q.w);
					Some((q, Vec3::from_array(varr).as_ivec3()))
				}).unwrap_or((Quat::IDENTITY, IVec3::ONE));
				stack.push((*child, Frame {
					translation: pose.translation + pose.rotation * Vec3::new(pos.x as f32, pos.z as f32, -pos.y as f32),
					rotation: pose.rotation * rot,
					flip: pose.flip * IVec3::new(flip_vec.x, flip_vec.z, flip_vec.y),
				}));
			}
			dot_vox::SceneNode::Group { children, .. } => {
				for child in children { stack.push((*child, pose)); }
			}
			dot_vox::SceneNode::Shape { models, .. } => {
				for shape_model in models {
					let Some(model) = dot_vox_data.models.get(shape_model.model_id as usize) else { continue };
					let size = Vec3::new(model.size.x as f32, model.size.z as f32, model.size.y as f32);
					let half = (size / 2.0).floor();
					let pose_transform = Transform { translation: pose.translation, rotation: pose.rotation, scale: Vec3::ONE };
					let half_offset = Transform::from_translation(-half * pose.flip.as_vec3());
					for voxel in &model.voxels {
						let local = IVec3::new(voxel.x as i32, voxel.z as i32, voxel.y as i32) * pose.flip + pose.flip.min(IVec3::ZERO);
						let world_pos = (pose_transform * half_offset).transform_point(local.as_vec3()).as_ivec3();
						let palette_entry = dot_vox_data.palette[voxel.i as usize];
						grid.add_voxel(&world_pos, &Voxel {
							color: [palette_entry.r, palette_entry.g, palette_entry.b, palette_entry.a],
							mass: 100,
						});
					}
				}
			}
		}
	}
	commands
		.spawn((
			RigidBody,
			IsStatic,
			Transform::from_translation(Vec3::new(0.0, -350.0, 0.0)),
		))
		.with_child((Transform::IDENTITY, grid, VoxelCollider));
}

fn spawn_ball_cluster(commands: &mut Commands, constraints: &mut BallJointConstraints) {
	let r = 5;
	let base_y = 80.0;
	let base_z = -20.0;

	let main = spawn_ball(commands, Vec3::new(0.0, base_y, base_z), 2);
	let satellites = [
		(spawn_ball(commands, Vec3::new(0.0, base_y, base_z + 10.0), r), Vec3::new(0.0, 0.0, 10.0)),
		(spawn_ball(commands, Vec3::new(0.0, base_y, base_z - 10.0), r), Vec3::new(0.0, 0.0, -10.0)),
		(spawn_ball(commands, Vec3::new(10.0, base_y, base_z), r), Vec3::new(10.0, 0.0, 0.0)),
		(spawn_ball(commands, Vec3::new(-10.0, base_y, base_z), r), Vec3::new(-10.0, 0.0, 0.0)),
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

fn spawn_bb8(commands: &mut Commands, constraints: &mut BallJointConstraints, position: Vec3) {
	let mut base_grid = Grid::new();
	for x in -6..=6 { for y in 0..3 { for z in -6..=6 {
		base_grid.add_voxel(&IVec3::new(x, y, z), &Voxel { color: [128, 128, 128, 255], mass: 200 });
	}}}
	base_grid.add_voxel(&IVec3::new(0, 3, 0), &Voxel { color: [255, 0, 0, 255], mass: 200 });

	let base = commands.spawn((
		RigidBody,
		Orientation::default(),
		Transform::from_translation(position),
	))
		.with_child((Transform::IDENTITY, base_grid, VoxelCollider, VoxelMass))
		.id();

	let ball = spawn_ball(commands, position - Vec3::new(0.0, 12.0, 0.0), 10);

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

fn spawn_ball(commands: &mut Commands, position: Vec3, radius: i32) -> Entity {
	let radius_sq = (radius as f32 - 0.5).powi(2);

	let mut top = Grid::new();
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

	let mut bottom = Grid::new();
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

	commands.spawn((
		RigidBody,
		Transform::from_translation(position),
	))
		.with_child((Transform::from_translation(Vec3::new(-0.5, -0.5, -0.5)), top, VoxelCollider, VoxelMass))
		.with_child((Transform {
				translation: Vec3::new(-std::f32::consts::FRAC_1_SQRT_2, -0.5, 0.0),
				rotation: Quat::from_rotation_y(PI / 4.0),
				scale: Vec3::ONE,
			}, bottom, VoxelCollider, VoxelMass))
		.id()
}
