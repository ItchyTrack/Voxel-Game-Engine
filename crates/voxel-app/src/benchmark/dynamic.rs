//! Moving objects for the `dynamic` scenario.
//!
//! The swarm is ~100 voxel grids orbiting and spinning above the church village. Their motion is a
//! pure function of benchmark time, so every run renders identical frames. The drop is a pile of
//! physics balls that fall onto the roofs with the solver running.

use std::f32::consts::TAU;

use basic_voxel::BasicVoxel;
use bevy::prelude::*;
use voxel_content::{StreamingVoxels, VoxelStoreSource};
use voxel_data::voxels::VoxelType;
use voxel_sources::SourceManager;

use crate::voxel::spawn_grid::spawn_grid;

pub const SWARM_CENTER: Vec3 = Vec3::new(0.0, 60.0, 0.0);
pub const SWARM_COUNT: u32 = 112;
pub const DROP_CENTER: Vec3 = Vec3::new(450.0, 260.0, -250.0);
const DROP_GRID: IVec3 = IVec3::new(4, 3, 4);
const DROP_SPACING: f32 = 26.0;

/// Seconds of benchmark time driving the swarm. The driver holds it at 0 until recording starts.
#[derive(Resource, Default)]
pub struct SwarmClock(pub f32);

#[derive(Component)]
pub struct SwarmObject {
	orbit_radius: f32,
	height: f32,
	orbit_speed: f32,
	phase: f32,
	bob: f32,
	spin_axis: Vec3,
	spin_speed: f32,
}

fn hash(mut x: u32) -> u32 {
	x ^= x >> 16;
	x = x.wrapping_mul(0x7feb_352d);
	x ^= x >> 15;
	x = x.wrapping_mul(0x846c_a68b);
	x ^ (x >> 16)
}

/// Deterministic value in 0..1 for object `index` and parameter `slot`.
fn unit(index: u32, slot: u32) -> f32 {
	(hash(index.wrapping_mul(0x9e37_79b9) ^ slot.wrapping_mul(0x85eb_ca6b)) & 0xFF_FFFF) as f32 / 16_777_216.0
}

fn color(index: u32) -> [u8; 4] {
	let h = hash(index ^ 0xc2b2_ae35);
	[64 + (h & 0x7F) as u8, 64 + ((h >> 8) & 0x7F) as u8, 64 + ((h >> 16) & 0x7F) as u8, 255]
}

fn object_voxels(index: u32) -> StreamingVoxels {
	let mut voxels = StreamingVoxels::new::<BasicVoxel>();
	let size = 4 + (unit(index, 0) * 7.0) as i32;
	let primary = BasicVoxel { color: color(index), mass: 100 };
	let stripe = BasicVoxel { color: color(index + 1000), mass: 100 };
	let is_ball = index % 3 != 0;
	for x in -size..=size { for y in -size..=size { for z in -size..=size {
		let p = IVec3::new(x, y, z);
		if is_ball && p.as_vec3().length_squared() > (size as f32 - 0.5).powi(2) { continue; }
		let voxel = if (y + size) % 4 == 0 { &stripe } else { &primary };
		voxels.add_voxel(p, voxel.get_ref());
	}}}
	voxels
}

fn swarm_transform(object: &SwarmObject, time: f32) -> Transform {
	let angle = object.phase + object.orbit_speed * time;
	let translation = SWARM_CENTER + Vec3::new(
		object.orbit_radius * angle.cos(),
		object.height + 15.0 * (object.bob + 0.7 * object.orbit_speed * time).sin(),
		object.orbit_radius * angle.sin(),
	);
	Transform::from_translation(translation)
		.with_rotation(Quat::from_axis_angle(object.spin_axis, object.phase + object.spin_speed * time))
}

pub fn spawn_moving_objects(commands: &mut Commands, sources: &mut SourceManager) {
	let store = sources.get_source_mut::<VoxelStoreSource>().expect("voxel store source not registered");
	for index in 0..SWARM_COUNT {
		let spin_axis = Vec3::new(unit(index, 6) - 0.5, unit(index, 7) - 0.5, unit(index, 8) - 0.5).normalize_or(Vec3::Y);
		let object = SwarmObject {
			orbit_radius: 60.0 + 200.0 * unit(index, 1),
			height: 20.0 + 180.0 * unit(index, 2),
			orbit_speed: (0.2 + 0.4 * unit(index, 3)) * if index % 2 == 0 { 1.0 } else { -1.0 },
			phase: TAU * unit(index, 4),
			bob: TAU * unit(index, 5),
			spin_axis,
			spin_speed: 0.5 + 1.5 * unit(index, 9),
		};
		let body = commands.spawn((swarm_transform(&object, 0.0), object)).id();
		spawn_grid(commands, store, Some(body), Transform::IDENTITY, object_voxels(index), ());
	}
	for x in 0..DROP_GRID.x { for y in 0..DROP_GRID.y { for z in 0..DROP_GRID.z {
		let offset = (IVec3::new(x, y, z).as_vec3() - (DROP_GRID - 1).as_vec3() * 0.5) * DROP_SPACING;
		crate::scene::scene::spawn_ball(commands, store, DROP_CENTER + offset, 6 + (x + y + z) % 4);
	}}}
}

pub fn move_swarm(clock: Res<SwarmClock>, mut objects: Query<(&SwarmObject, &mut Transform)>) {
	for (object, mut transform) in &mut objects {
		*transform = swarm_transform(object, clock.0);
	}
}
