use bevy::ecs::message::MessageWriter;
use bevy::input::ButtonInput;
use bevy::math::{IVec3, Quat, Vec3};
use bevy::prelude::*;
use bevy::transform::components::{GlobalTransform, Transform};
use bevy_egui::input::EguiWantsInput;

use voxel_data::grid::{Grid, SubGrid};
use voxel_data::voxels::Voxel;
use voxel_physics::{CenterOfMass, Impulses, IsStatic, Mass, PhysicsBodyId, PhysicsSet, Velocity};

use crate::audio_plugin::PlaySfx;

pub struct WorldInteractionPlugin;

impl Plugin for WorldInteractionPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<HeldBody>()
			.add_systems(Update, (
				voxel_place_break_system,
				pickup_toggle_system,
				push_system,
			))
			.add_systems(FixedUpdate, hold_held_body_system.in_set(PhysicsSet::Apply));
	}
}

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct HeldBody(pub Option<Entity>);

const HOLD_DISTANCE: f32 = 40.0;
const PUSH_IMPULSE: f32 = 1_600_000.0;

const PLACE_VOXEL: Voxel = Voxel { color: [180, 180, 180, 255], mass: 100 };

fn voxel_place_break_system(
	keys: Res<ButtonInput<KeyCode>>,
	egui_wants: Option<Res<EguiWantsInput>>,
	cameras: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
	mut grids: Query<(Entity, Option<&GlobalTransform>, &mut Grid)>,
	mut sfx: MessageWriter<PlaySfx>,
) {
	if egui_wants.is_some_and(|e| e.wants_any_keyboard_input()) { return; }
	let place = keys.just_pressed(KeyCode::Space) || keys.pressed(KeyCode::KeyC);
	let destroy = keys.just_pressed(KeyCode::KeyX) || keys.pressed(KeyCode::KeyZ);
	if !place && !destroy { return; }

	let Some((_, camera_gt)) = cameras.iter().find(|(c, _)| c.is_active) else { return };
	let camera_transform = camera_gt.compute_transform();
	let origin = camera_transform.translation;
	let dir = camera_transform.forward().as_vec3();

	let hit = grids.iter()
		.filter_map(|(entity, grid_gt, grid)| {
			let grid_world = grid_world_transform(grid_gt, &grid);
			raycast_grid(entity, &grid_world, &grid, origin, dir)
		})
		.min_by(|a, b| a.distance.total_cmp(&b.distance));

	let Some(hit) = hit else { return };

	let Ok((_, grid_gt, mut grid)) = grids.get_mut(hit.grid_entity) else { return };
	let grid_world = grid_world_transform(grid_gt, &grid);

	if place {
		let pos = hit.voxel_pos + hit.normal;
		grid.add_voxel(&pos, &PLACE_VOXEL);
		sfx.write(PlaySfx::block_place(grid_world.transform_point(pos.as_vec3() + Vec3::splat(0.5))));
	} else {
		grid.remove_voxel(&hit.voxel_pos);
		sfx.write(PlaySfx::block_break(grid_world.transform_point(hit.voxel_pos.as_vec3() + Vec3::splat(0.5))));
	}
}

struct RaycastHit {
	grid_entity: Entity,
	voxel_pos: IVec3,
	normal: IVec3,
	distance: f32,
}

fn grid_world_transform(grid_gt: Option<&GlobalTransform>, grid: &Grid) -> Transform {
	grid_gt
		.map(|gt| gt.compute_transform() * *grid.transform())
		.unwrap_or(*grid.transform())
}

fn raycast_grid(
	entity: Entity,
	grid_world: &Transform,
	grid: &Grid,
	world_origin: Vec3,
	world_dir: Vec3,
) -> Option<RaycastHit> {
	let inv = grid_world.to_matrix().inverse();
	let grid_origin = inv.transform_point3(world_origin);
	let grid_dir = inv.transform_vector3(world_dir).normalize();

	grid.sub_grids().iter()
		.filter_map(|(_, sub_grid)| {
			let sub_origin = sub_grid.sub_grid_pos().as_vec3();
			raycast_sub_grid(sub_grid, grid_origin - sub_origin, grid_dir)
				.map(|(hit_local, normal_local, distance)| RaycastHit {
					grid_entity: entity,
					voxel_pos: sub_grid.sub_grid_pos() + hit_local.as_ivec3(),
					normal: normal_local.as_ivec3(),
					distance,
				})
		})
		.min_by(|a, b| a.distance.total_cmp(&b.distance))
}

fn raycast_sub_grid(
	sub_grid: &SubGrid,
	origin: Vec3,
	dir: Vec3,
) -> Option<(bevy::math::I16Vec3, bevy::math::I8Vec3, f32)> {
	// GridTree::raycast takes a Transform whose rotation maps +Z to the ray dir.
	let transform = Transform {
		translation: origin,
		rotation: Quat::from_rotation_arc(Vec3::Z, dir),
		scale: Vec3::ONE,
	};
	sub_grid.get_voxels().get_voxels().raycast(&transform, None)
}

fn camera_ray(cameras: &Query<(&Camera, &GlobalTransform), With<Camera3d>>) -> Option<(Vec3, Vec3)> {
	let (_, camera_gt) = cameras.iter().find(|(c, _)| c.is_active)?;
	let t = camera_gt.compute_transform();
	Some((t.translation, t.forward().as_vec3()))
}

fn raycast_bodies(
	origin: Vec3,
	dir: Vec3,
	grids: &Query<(Entity, Option<&GlobalTransform>, &Grid)>,
) -> Option<RaycastHit> {
	grids.iter()
		.filter_map(|(entity, grid_gt, grid)| {
			let grid_world = grid_world_transform(grid_gt, grid);
			raycast_grid(entity, &grid_world, grid, origin, dir)
		})
		.min_by(|a, b| a.distance.total_cmp(&b.distance))
}

fn pickup_toggle_system(
	keys: Res<ButtonInput<KeyCode>>,
	egui_wants: Option<Res<EguiWantsInput>>,
	cameras: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
	grids: Query<(Entity, Option<&GlobalTransform>, &Grid)>,
	parents: Query<&ChildOf>,
	bodies: Query<Has<IsStatic>, With<voxel_physics::RigidBody>>,
	mut held: ResMut<HeldBody>,
) {
	if egui_wants.is_some_and(|e| e.wants_any_keyboard_input()) { return; }
	if !keys.just_pressed(KeyCode::KeyF) { return; }
	if held.0.is_some() { held.0 = None; return; }

	let Some((origin, dir)) = camera_ray(&cameras) else { return };
	let Some(hit) = raycast_bodies(origin, dir, &grids) else { return };
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
	grids: Query<(Entity, Option<&GlobalTransform>, &Grid)>,
	parents: Query<&ChildOf>,
	bodies: Query<(), (With<voxel_physics::RigidBody>, Without<IsStatic>)>,
	mut impulses: ResMut<Impulses>,
) {
	if egui_wants.is_some_and(|e| e.wants_any_keyboard_input()) { return; }
	if !keys.just_pressed(KeyCode::KeyR) { return; }
	let Some((origin, dir)) = camera_ray(&cameras) else { return };
	let Some(hit) = raycast_bodies(origin, dir, &grids) else { return };
	let Ok(child_of) = parents.get(hit.grid_entity) else { return };
	let body = child_of.parent();
	if bodies.get(body).is_err() { return; }
	let hit_world = origin + dir * hit.distance;
	impulses.apply_impulse(PhysicsBodyId(body), hit_world, dir * PUSH_IMPULSE);
}

fn hold_held_body_system(
	held: Res<HeldBody>,
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
	let impulse = mass.0 * (
		dir * (offset.length() * 4.0 - velocity_in_dir * 0.5)
		- (velocity.0 - dir * velocity_in_dir)
	);
	impulses.apply_central_impulse(PhysicsBodyId(body_entity), impulse);
}
