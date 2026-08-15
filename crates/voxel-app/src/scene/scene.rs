use std::f32::consts::PI;

use bevy::math::{IVec3, Quat, Vec3};
use bevy::prelude::*;
use std::path::PathBuf;

use basic_voxel::{BasicVoxel, MarchingVoxel};
use voxel_content::{SdfSource, StreamingVoxels, VoxFileSource, VoxelStoreSource};
use voxel_data::grid::Grid;
use voxel_data::voxels::VoxelType;
use voxel_streaming::GridEdits;
use voxel_physics::components::{VoxelCollider, VoxelMass};
use voxel_gpu::{RenderingContext, RenderingType};
use voxel_lightyear::ReplicateVoxels;
use tile_data::TileGenerationContext;
use voxel_physics::{
	AngularVelocity, BallJoint, Impulses, IsStatic, RigidBody, RotationalInertia, VoxelPhysicsAppExt
};
use voxel_sources::VoxelSourcesAppExt;
use voxel_streaming::{GridStreaming, RequestChunkPresence};

use crate::voxel::spawn_grid::spawn_grid;

type SceneVoxFileSource = VoxFileSource<BasicVoxel>;
type MarchingVoxFileSource = VoxFileSource<MarchingVoxel>;

pub struct ScenePlugin;

impl Plugin for ScenePlugin {
	fn build(&self, app: &mut App) {
		let vox_source = SceneVoxFileSource::new();
		let marching_vox_source = MarchingVoxFileSource::new();
		let sdf_source = SdfSource::new();
		app
			.insert_resource(vox_source.clone())
			.insert_resource(marching_vox_source.clone())
			.insert_resource(sdf_source.clone())
			.register_voxel_source(vox_source)
			.register_voxel_source(marching_vox_source)
			.register_voxel_source(sdf_source)
			.add_systems(Startup, setup_scene)
			.add_physics_apply_systems(drive_orientation);
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
	store: Res<VoxelStoreSource>,
	vox_source: Res<SceneVoxFileSource>,
	marching_vox_source: Res<MarchingVoxFileSource>,
	_sdf_source: Res<SdfSource>,
) {
	spawn_church(&mut commands, &vox_source);
	spawn_sponza(&mut commands, &marching_vox_source);
	// spawn_ball_cluster(&mut commands, &mut store);
	// spawn_bb8(&mut commands, &mut store, Vec3::new(0.0, 120.0, 0.0));
	spawn_bb8(&mut commands, &store, Vec3::new(30.0, 120.0, 0.0));
	spawn_bb8(&mut commands, &store, Vec3::new(-30.0, 120.0, 0.0));
	// for x in 0..3 {
	// 	for y in 0..2 {
	// 		for z in 0..3 {
	// 			spawn_bb8(&mut commands, &mut store, Vec3::new(30.0 * x as f32, 30.0 * y as f32 + 200.0, 30.0 * z as f32));
	// 		}
	// 	}
	// }
}

fn spawn_sponza(commands: &mut Commands, vox_source: &MarchingVoxFileSource) {
	let Some(path) = sponza_vox_path() else { return };

	let parent = commands
		.spawn((
			RigidBody,
			IsStatic,
			Transform::from_xyz(1500.0, 0.0, 0.0),
		))
		.id();
	let grid = commands.spawn((
		Transform::IDENTITY,
		Grid::new::<MarchingVoxel>(),
		GridEdits::default(),
		GridStreaming::default(),
		RequestChunkPresence,
		ReplicateVoxels,
		VoxelCollider,
		TileGenerationContext::new(RenderingContext { rendering_type: RenderingType::Raster }),
	)).id();
	commands.entity(parent).add_child(grid);
	vox_source.set_grid_vox_file(grid, Vec3::ZERO, path);
}

fn spawn_church(commands: &mut Commands, vox_source: &SceneVoxFileSource) {
	let Some(path) = church_vox_path() else { return };

	let parent = commands
		.spawn((
			RigidBody,
			IsStatic,
			Transform::from_xyz(0.0, -350.0, 0.0),
		))
		.id();
	let grid = commands.spawn((
			Transform::IDENTITY,
			Grid::new::<BasicVoxel>(),
			GridEdits::default(),
			GridStreaming::default(),
			RequestChunkPresence,
			ReplicateVoxels,
			VoxelCollider,
		))
		.id();
	commands.entity(parent).add_child(grid);
	vox_source.set_grid_vox_file(grid, Vec3::ZERO, path);
}

#[cfg(target_arch = "wasm32")]
fn sponza_vox_path() -> Option<PathBuf> { None }

#[cfg(not(target_arch = "wasm32"))]
fn sponza_vox_path() -> Option<PathBuf> {
	[
		PathBuf::from("res/sponza.vox"),
		PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../res/sponza.vox"),
	]
	.into_iter()
	.find(|p| p.exists())
}

#[cfg(target_arch = "wasm32")]
fn church_vox_path() -> Option<PathBuf> {
	None
}

#[cfg(not(target_arch = "wasm32"))]
fn church_vox_path() -> Option<PathBuf> {
	[
		PathBuf::from("res/Church_Of_St_Sophia.vox"),
		PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../res/Church_Of_St_Sophia.vox"),
	]
	.into_iter()
	.find(|p| p.exists())
}

fn spawn_ball_cluster(
	commands: &mut Commands,
	store: &VoxelStoreSource,
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
		commands.spawn(BallJoint::new(
			main,
			satellite,
			&Transform::from_translation(attachment),
			&Transform::IDENTITY,
			f32::INFINITY,
			0.0,
		));
	}
}

fn voxel(color: [u8; 4], mass: u32) -> BasicVoxel {
	BasicVoxel { color, mass }
}

fn spawn_bb8(
	commands: &mut Commands,
	store: &VoxelStoreSource,
	position: Vec3,
) {
	let mut base_grid = StreamingVoxels::new::<BasicVoxel>();
	for x in -6..=6 { for y in 0..3 { for z in -6..=6 {
		base_grid.add_voxel(&IVec3::new(x, y, z), voxel([128, 128, 128, 255], 200).get_ref());
	}}}
	base_grid.add_voxel(&IVec3::new(0, 3, 0), voxel([255, 0, 0, 255], 200).get_ref());

	let base = commands.spawn((
		RigidBody,
		Orientation::default(),
		Transform::from_translation(position),
	)).id();
	spawn_grid(commands, store, Some(base), Transform::IDENTITY, base_grid, (VoxelCollider, VoxelMass));

	let ball = spawn_ball(commands, store, position - Vec3::new(0.0, 12.0, 0.0), 10);

	commands.spawn(BallJoint::new(
		base,
		ball,
		&Transform::from_translation(Vec3::new(0.0, -12.0, 0.0)),
		&Transform::IDENTITY,
		f32::INFINITY,
		0.0,
	));
}

fn spawn_ball(commands: &mut Commands, store: &VoxelStoreSource, position: Vec3, radius: i32) -> Entity {
	let radius_sq = (radius as f32 - 0.5).powi(2);

	let mut top = StreamingVoxels::new::<BasicVoxel>();
	for x in -radius..=radius {
		for y in 0..=radius {
			for z in -radius..=radius {
				let p = IVec3::new(x, y, z);
				if p.as_vec3().length_squared() > radius_sq { continue; }
				top.add_voxel(&p, voxel([(x as u8 / 10) * 10, (y as u8 / 10) * 10, (z as u8 / 10) * 10, 255], 100).get_ref());
			}
		}
	}

	let mut bottom = StreamingVoxels::new::<BasicVoxel>();
	for x in -radius..=radius {
		for y in -radius..0 {
			for z in -radius..=radius {
				let p = IVec3::new(x, y, z);
				if p.as_vec3().length_squared() > radius_sq { continue; }
				bottom.add_voxel(&p, voxel([(x as u8 / 10) * 10, (y as u8 / 10) * 10, (z as u8 / 10) * 10, 255], 100).get_ref());
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
