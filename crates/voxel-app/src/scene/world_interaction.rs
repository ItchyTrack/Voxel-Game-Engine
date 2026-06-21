use bevy::ecs::message::MessageWriter;
use bevy::input::ButtonInput;
use bevy::math::{Vec2, Vec3};
use bevy::prelude::*;
use bevy::transform::components::{GlobalTransform, Transform};
use bevy_egui::input::EguiWantsInput;

use std::sync::Arc;

use voxel_data::voxels::Voxel;
use voxel_data::world_query::VoxelWorldQueryParam;
use voxel_edit::GridEdits;
use voxel_physics::{CenterOfMass, FreezePhysics, Impulses, IsStatic, Mass, PhysicsSet, Velocity};

use crate::audio::plugin::PlaySfx;

pub struct WorldInteractionPlugin;

impl Plugin for WorldInteractionPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<HeldBody>()
			.add_systems(Update, (
				voxel_place_break_system,
				sdf_place_system,
				pickup_toggle_system,
				push_system.run_if(|freeze: Res<FreezePhysics>| !freeze.0),
			))
			.add_systems(
				FixedUpdate,
				hold_held_body_system
					.in_set(PhysicsSet::Apply)
					.run_if(|freeze: Res<FreezePhysics>| !freeze.0),
			);
	}
}

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct HeldBody(pub Option<Entity>);

const HOLD_DISTANCE: f32 = 40.0;
const PUSH_IMPULSE: f32 = 1_600_000.0;
const MAX_GRAB_ACCEL: f32 = 8_000.0;

const PLACE_VOXEL: Voxel = Voxel { color: [180, 180, 180, 255], mass: 100 };
const PLACE_SDF_VOXEL: Voxel = Voxel { color: [80, 180, 255, 255], mass: 100 };
const PLACE_SDF_RADIUS: f32 = 5.0;

fn voxel_place_break_system(
	keys: Res<ButtonInput<KeyCode>>,
	egui_wants: Option<Res<EguiWantsInput>>,
	cameras: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
	voxel_world: VoxelWorldQueryParam,
	mut grids: Query<(&GlobalTransform, &mut GridEdits)>,
	mut sfx: Option<MessageWriter<PlaySfx>>,
) {
	if egui_wants.is_some_and(|e| e.wants_any_keyboard_input()) { return; }
	let place = keys.just_pressed(KeyCode::Space) || keys.pressed(KeyCode::KeyC);
	let destroy = keys.just_pressed(KeyCode::KeyX) || keys.pressed(KeyCode::KeyZ);
	if !place && !destroy { return; }

	let Some((origin, dir)) = camera_ray(&cameras) else { return };
	let Some(hit) = voxel_world.raycast(origin, dir, None) else { return };

	let Ok((grid_global_transform, mut edits)) = grids.get_mut(hit.grid) else { return };

	if place {
		let pos = hit.voxel_pos + hit.normal;
		edits.add_voxel(&pos, &PLACE_VOXEL);
		if let Some(sfx) = &mut sfx {
			sfx.write(PlaySfx::block_place(grid_global_transform.transform_point(pos.as_vec3() + Vec3::splat(0.5))));
		}
	} else {
		edits.remove_voxel(&hit.voxel_pos);
		if let Some(sfx) = &mut sfx {
			sfx.write(PlaySfx::block_break(grid_global_transform.transform_point(hit.voxel_pos.as_vec3() + Vec3::splat(0.5))));
		}
	}
}

fn sdf_place_system(
	keys: Res<ButtonInput<KeyCode>>,
	egui_wants: Option<Res<EguiWantsInput>>,
	cameras: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
	voxel_world: VoxelWorldQueryParam,
	mut grids: Query<(&GlobalTransform, &mut GridEdits)>,
	mut sfx: Option<MessageWriter<PlaySfx>>,
) {
	if egui_wants.is_some_and(|e| e.wants_any_keyboard_input()) { return; }
	if !keys.just_pressed(KeyCode::KeyG) { return; }

	let Some((origin, dir)) = camera_ray(&cameras) else { return };
	let Some(hit) = voxel_world.raycast(origin, dir, None) else { return };
	let Ok((grid_global_transform, mut edits)) = grids.get_mut(hit.grid) else { return };

	let center_voxel = hit.voxel_pos + hit.normal;
	let center = center_voxel.as_vec3() + Vec3::splat(0.5);
	let offsets = [
		Vec3::new(-16.0, 0.0, 0.0),
		Vec3::new(18.0, 8.0, 8.0),
		Vec3::new(7.0, -9.0, -18.0),
	];
	let centers: Vec<Vec3> = offsets.into_iter().map(|offset| center + offset).collect();
	let radius = (PLACE_SDF_RADIUS + 2.5) * 2.0;
	let smooth_k = 16.0;
	let reach = (PLACE_SDF_RADIUS + 20.0) * 2.0;
	let slider_radius = 9.0;
	let slider_half_length = 54.0;
	let sdf = Arc::new(move |p: Vec3| {
		let Some(first) = centers.first() else { return f32::INFINITY };
		let mut blob = (p - *first).length() - radius;
		for center in centers.iter().skip(1) {
			let b = (p - *center).length() - radius;
			let h = (0.5 + 0.5 * (b - blob) / smooth_k).clamp(0.0, 1.0);
			blob = b * (1.0 - h) + blob * h - smooth_k * h * (1.0 - h);
		}
		let q = p - center;
		let angle = std::f32::consts::FRAC_PI_4;
		let axis_q = Vec3::new(
			q.x,
			q.y * angle.cos() - q.z * angle.sin(),
			q.y * angle.sin() + q.z * angle.cos(),
		);
		let d = Vec2::new(Vec2::new(axis_q.x, axis_q.z).length(), axis_q.y.abs()) - Vec2::new(slider_radius, slider_half_length);
		let capsule = d.max(Vec2::ZERO).length() + d.x.max(d.y).min(0.0);
		blob.max(-capsule)
	});
	edits.apply_sdf(center - Vec3::splat(reach), center + Vec3::splat(reach), PLACE_SDF_VOXEL, sdf);
	if let Some(sfx) = &mut sfx {
		sfx.write(PlaySfx::block_place(grid_global_transform.transform_point(center)));
	}
}

fn camera_ray(cameras: &Query<(&Camera, &GlobalTransform), With<Camera3d>>) -> Option<(Vec3, Vec3)> {
	let (_, camera_global_transform) = cameras.iter().find(|(c, _)| c.is_active)?;
	let t = camera_global_transform.compute_transform();
	Some((t.translation, t.forward().as_vec3()))
}

fn pickup_toggle_system(
	keys: Res<ButtonInput<KeyCode>>,
	egui_wants: Option<Res<EguiWantsInput>>,
	cameras: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
	voxel_world: VoxelWorldQueryParam,
	parents: Query<&ChildOf>,
	bodies: Query<Has<IsStatic>, With<voxel_physics::RigidBody>>,
	mut held: ResMut<HeldBody>,
) {
	if egui_wants.is_some_and(|e| e.wants_any_keyboard_input()) { return; }
	if !keys.just_pressed(KeyCode::KeyF) { return; }
	if held.0.is_some() { held.0 = None; return; }

	let Some((origin, dir)) = camera_ray(&cameras) else { return };
	let Some(hit) = voxel_world.raycast(origin, dir, None) else { return };
	let Ok(child_of) = parents.get(hit.grid) else { return };
	let body = child_of.parent();
	let Ok(is_static) = bodies.get(body) else { return };
	if is_static { return; }
	held.0 = Some(body);
}

fn push_system(
	keys: Res<ButtonInput<KeyCode>>,
	egui_wants: Option<Res<EguiWantsInput>>,
	cameras: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
	voxel_world: VoxelWorldQueryParam,
	parents: Query<&ChildOf>,
	bodies: Query<(), (With<voxel_physics::RigidBody>, Without<IsStatic>)>,
	mut impulses: ResMut<Impulses>,
) {
	if egui_wants.is_some_and(|e| e.wants_any_keyboard_input()) { return; }
	if !keys.just_pressed(KeyCode::KeyR) { return; }
	let Some((origin, dir)) = camera_ray(&cameras) else { return };
	let Some(hit) = voxel_world.raycast(origin, dir, None) else { return };
	let Ok(child_of) = parents.get(hit.grid) else { return };
	let body = child_of.parent();
	if bodies.get(body).is_err() { return; }
	impulses.apply_impulse(body, hit.world_position, dir * PUSH_IMPULSE);
}

fn hold_held_body_system(
	held: Res<HeldBody>,
	time: Res<Time>,
	cameras: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
	bodies: Query<(&Transform, &Velocity, &Mass, &CenterOfMass), (With<voxel_physics::RigidBody>, Without<IsStatic>)>,
	mut impulses: ResMut<Impulses>,
) {
	let Some(body_entity) = held.0 else { return };
	let Ok((transform, velocity, mass, com)) = bodies.get(body_entity) else { return };
	let Some((origin, forward)) = camera_ray(&cameras) else { return };

	let target = origin + forward * HOLD_DISTANCE;
	let body_com_world = *transform * com.0;
	let offset = target - body_com_world;
	if offset.length_squared() < 1e-6 { return; }
	let dir = offset.normalize();
	let velocity_in_dir = velocity.0.dot(dir);
	let delta_v = dir * (offset.length() * 4.0 - velocity_in_dir * 0.5)
		- (velocity.0 - dir * velocity_in_dir);
	let delta_v = delta_v.clamp_length_max(MAX_GRAB_ACCEL * time.delta_secs());
	impulses.apply_central_impulse(body_entity, mass.0 * delta_v);
}
