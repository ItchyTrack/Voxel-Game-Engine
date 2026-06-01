use bevy::ecs::message::MessageWriter;
use bevy::input::ButtonInput;
use bevy::math::{IVec3, Vec3};
use bevy::prelude::*;
use bevy::transform::components::{GlobalTransform, Transform};
use bevy_egui::input::EguiWantsInput;

use voxel_data::grid::{Grid, GridId};
use voxel_data::voxels::Voxel;
use voxel_physics::{CenterOfMass, FreezePhysics, Impulses, IsStatic, Mass, PhysicsSet, Velocity};

use crate::audio_plugin::PlaySfx;

pub struct WorldInteractionPlugin;

impl Plugin for WorldInteractionPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<HeldBody>()
			.add_systems(Update, (
				voxel_place_break_system,
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

fn voxel_place_break_system(
	keys: Res<ButtonInput<KeyCode>>,
	egui_wants: Option<Res<EguiWantsInput>>,
	cameras: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
	mut grids: Query<(Entity, &GlobalTransform, &mut Grid)>,
	mut sfx: MessageWriter<PlaySfx>,
) {
	if egui_wants.is_some_and(|e| e.wants_any_keyboard_input()) { return; }
	let place = keys.just_pressed(KeyCode::Space) || keys.pressed(KeyCode::KeyC);
	let destroy = keys.just_pressed(KeyCode::KeyX) || keys.pressed(KeyCode::KeyZ);
	if !place && !destroy { return; }

	let Some((origin, dir)) = camera_ray(&cameras) else { return };
	let hit = raycast_grids(grids.iter().map(|(e, gt, g)| (e, gt, g)), origin, dir);
	let Some(hit) = hit else { return };

	let Ok((_, grid_global_transform, mut grid)) = grids.get_mut(hit.grid_entity) else { return };

	if place {
		let pos = hit.voxel_pos + hit.normal;
		grid.add_voxel(&pos, &PLACE_VOXEL);
		sfx.write(PlaySfx::block_place(grid_global_transform.transform_point(pos.as_vec3() + Vec3::splat(0.5))));
	} else {
		grid.remove_voxel(&hit.voxel_pos);
		sfx.write(PlaySfx::block_break(grid_global_transform.transform_point(hit.voxel_pos.as_vec3() + Vec3::splat(0.5))));
	}
}

struct RaycastHit {
	grid_entity: GridId,
	voxel_pos: IVec3,
	normal: IVec3,
	distance: f32,
}

fn raycast_grid(
	entity: GridId,
	grid_world: &Transform,
	grid: &Grid,
	world_origin: Vec3,
	world_dir: Vec3,
) -> Option<RaycastHit> {
	let inv = grid_world.to_matrix().inverse();
	let origin = inv.transform_point3(world_origin);
	let dir = inv.transform_vector3(world_dir).normalize();

	grid.raycast(origin, dir).map(|hit| RaycastHit {
		grid_entity: entity,
		voxel_pos: hit.voxel_pos,
		normal: hit.normal,
		distance: hit.distance,
	})
}

fn camera_ray(cameras: &Query<(&Camera, &GlobalTransform), With<Camera3d>>) -> Option<(Vec3, Vec3)> {
	let (_, camera_global_transform) = cameras.iter().find(|(c, _)| c.is_active)?;
	let t = camera_global_transform.compute_transform();
	Some((t.translation, t.forward().as_vec3()))
}

fn raycast_grids<'a>(
	grids: impl Iterator<Item = (GridId, &'a GlobalTransform, &'a Grid)>,
	origin: Vec3,
	dir: Vec3,
) -> Option<RaycastHit> {
	grids
		.filter_map(|(entity, grid_global_transform, grid)| {
			raycast_grid(entity, &grid_global_transform.compute_transform(), grid, origin, dir)
		})
		.min_by(|a, b| a.distance.total_cmp(&b.distance))
}

fn pickup_toggle_system(
	keys: Res<ButtonInput<KeyCode>>,
	egui_wants: Option<Res<EguiWantsInput>>,
	cameras: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
	grids: Query<(Entity, &GlobalTransform, &Grid)>,
	parents: Query<&ChildOf>,
	bodies: Query<Has<IsStatic>, With<voxel_physics::RigidBody>>,
	mut held: ResMut<HeldBody>,
) {
	if egui_wants.is_some_and(|e| e.wants_any_keyboard_input()) { return; }
	if !keys.just_pressed(KeyCode::KeyF) { return; }
	if held.0.is_some() { held.0 = None; return; }

	let Some((origin, dir)) = camera_ray(&cameras) else { return };
	let Some(hit) = raycast_grids(grids.iter(), origin, dir) else { return };
	let Ok(child_of) = parents.get(hit.grid_entity) else { return };
	let body = child_of.parent();
	let Ok(is_static) = bodies.get(body) else { return };
	if is_static { return; }
	held.0 = Some(body);
}

fn push_system(
	keys: Res<ButtonInput<KeyCode>>,
	egui_wants: Option<Res<EguiWantsInput>>,
	cameras: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
	grids: Query<(Entity, &GlobalTransform, &Grid)>,
	parents: Query<&ChildOf>,
	bodies: Query<(), (With<voxel_physics::RigidBody>, Without<IsStatic>)>,
	mut impulses: ResMut<Impulses>,
) {
	if egui_wants.is_some_and(|e| e.wants_any_keyboard_input()) { return; }
	if !keys.just_pressed(KeyCode::KeyR) { return; }
	let Some((origin, dir)) = camera_ray(&cameras) else { return };
	let Some(hit) = raycast_grids(grids.iter(), origin, dir) else { return };
	let Ok(child_of) = parents.get(hit.grid_entity) else { return };
	let body = child_of.parent();
	if bodies.get(body).is_err() { return; }
	let hit_world = origin + dir * hit.distance;
	impulses.apply_impulse(body, hit_world, dir * PUSH_IMPULSE);
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
