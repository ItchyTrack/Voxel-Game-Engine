use std::collections::HashSet;

use bevy::ecs::message::MessageWriter;
use bevy::input::ButtonInput;
use bevy::math::Vec3;
use bevy::prelude::*;
use bevy::transform::components::{GlobalTransform, Transform};
use bevy_egui::input::EguiWantsInput;

use tile_data::{CHUNK_SIZE, NonZeroChunkRegion, TileKey};
use voxel_content::grid_store::GridStoreEditApi;
use voxel_trees::region::NonZeroVoxelRegion;
use voxel_data::grid::{Grid, GridId};
use voxel_data::voxels::{Voxel, VoxelType};
use voxel_query::{OccupancyTileClass, VoxelWorldQueryParam};
use voxel_sources::edit::{AddArea, RemoveArea};
use voxel_physics::{CenterOfMass, FreezePhysics, Impulses, IsStatic, Mass, Velocity, VoxelPhysicsAppExt};
use voxel_streaming::{GridStreaming, TileRequester};

use crate::audio::plugin::PlaySfx;
use basic_voxel::{BasicVoxel, MarchingVoxel};

pub struct WorldInteractionPlugin;

impl Plugin for WorldInteractionPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<HeldBody>()
			.init_resource::<RayOccupancyRequests>()
			.configure_sets(Update, (WorldInteractionSet::LoadOccupancy, WorldInteractionSet::Interact).chain())
			.add_systems(Update, update_ray_occupancy_requests.in_set(WorldInteractionSet::LoadOccupancy))
			.add_systems(Update, (
				voxel_place_break_system,
				pickup_toggle_system,
				push_system.run_if(|freeze: Res<FreezePhysics>| !freeze.0),
			).in_set(WorldInteractionSet::Interact))
			.add_physics_apply_systems(hold_held_body_system);
	}
}

#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum WorldInteractionSet {
	LoadOccupancy,
	Interact,
}

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct HeldBody(pub Option<Entity>);

#[derive(Resource, Default)]
struct RayOccupancyRequests(HashSet<RayOccupancyRequest>);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct RayOccupancyRequest {
	requester: Entity,
	grid: GridId,
	key: TileKey,
}

#[derive(Debug, Clone, Copy)]
struct PlayerRay {
	requester: Entity,
	origin: Vec3,
	direction: Vec3,
}

const RAY_OCCUPANCY_DISTANCE: f32 = 200.0;
const HOLD_DISTANCE: f32 = 40.0;
const PUSH_IMPULSE: f32 = 1_600_000.0;
const MAX_GRAB_ACCEL: f32 = 8_000.0;

const PLACE_VOXEL: BasicVoxel = BasicVoxel { color: [180, 180, 180, 255], mass: 100 };
const PLACE_SDF_VOXEL: BasicVoxel = BasicVoxel { color: [80, 180, 255, 255], mass: 100 };
const PLACE_SDF_RADIUS: f32 = 5.0;

fn edit_voxel_for_grid(grid: &Grid, voxel: BasicVoxel) -> Option<Voxel> {
	let voxel_type = grid.voxel_type_info().id;
	if voxel_type == BasicVoxel::TYPE_ID {
		Some(voxel.into_voxel())
	} else if voxel_type == MarchingVoxel::TYPE_ID {
		Some(MarchingVoxel(voxel).into_voxel())
	} else {
		warn!(?voxel_type, "cannot edit grid with unsupported voxel type");
		None
	}
}

fn update_ray_occupancy_requests(
	cameras: Query<(Entity, &Camera, &GlobalTransform), With<Camera3d>>,
	class: Res<OccupancyTileClass>,
	mut requested: ResMut<RayOccupancyRequests>,
	mut streaming: ParamSet<(
		TileRequester,
		Query<(GridId, &GlobalTransform, &GridStreaming)>,
	)>,
) {
	let mut desired = HashSet::new();
	if let Some(ray) = player_ray(&cameras) {
		let grids = streaming.p1();
		for (grid, grid_global, grid_streaming) in &grids {
			let grid_transform = grid_global.compute_transform();
			if !grid_transform.scale.abs_diff_eq(Vec3::ONE, 1e-5) { continue; }

			let inverse_rotation = grid_transform.rotation.inverse();
			let local_origin = inverse_rotation * (ray.origin - grid_transform.translation);
			let local_direction = inverse_rotation * ray.direction;
			for_each_ray_chunk(local_origin, local_direction, RAY_OCCUPANCY_DISTANCE, |chunk| {
				if !grid_streaming.presence().is_present(chunk) { return; }
				desired.insert(RayOccupancyRequest {
					requester: ray.requester,
					grid,
					key: TileKey::new(NonZeroChunkRegion::from_single(chunk), 0, class.0),
				});
			});
		}
	}

	let acquisitions: Vec<_> = desired.difference(&requested.0).copied().collect();
	let releases: Vec<_> = requested.0.difference(&desired).copied().collect();
	let mut retained: HashSet<_> = requested.0.intersection(&desired).copied().collect();
	let mut requester = streaming.p0();

	// Acquire first so shared tiles survive an active-camera switch.
	for request in acquisitions {
		if requester.fetch_tile(request.grid, request.requester, request.key, 0.0, None) {
			retained.insert(request);
		}
	}
	for request in releases {
		requester.release_tile(request.grid, request.requester, request.key);
	}
	requested.0 = retained;
}

fn for_each_ray_chunk(origin: Vec3, direction: Vec3, max_distance: f32, mut visit: impl FnMut(IVec3)) {
	let direction = direction.normalize_or_zero();
	if direction == Vec3::ZERO { return; }

	let chunk_size = CHUNK_SIZE as f32;
	let mut chunk = (origin / chunk_size).floor().as_ivec3();
	let step = IVec3::new(
		if direction.x > 0.0 { 1 } else if direction.x < 0.0 { -1 } else { 0 },
		if direction.y > 0.0 { 1 } else if direction.y < 0.0 { -1 } else { 0 },
		if direction.z > 0.0 { 1 } else if direction.z < 0.0 { -1 } else { 0 },
	);
	let next_boundary = Vec3::new(
		(if step.x > 0 { chunk.x + 1 } else { chunk.x }) as f32 * chunk_size,
		(if step.y > 0 { chunk.y + 1 } else { chunk.y }) as f32 * chunk_size,
		(if step.z > 0 { chunk.z + 1 } else { chunk.z }) as f32 * chunk_size,
	);
	let mut boundary_distance = Vec3::new(
		if step.x == 0 { f32::INFINITY } else { (next_boundary.x - origin.x) / direction.x },
		if step.y == 0 { f32::INFINITY } else { (next_boundary.y - origin.y) / direction.y },
		if step.z == 0 { f32::INFINITY } else { (next_boundary.z - origin.z) / direction.z },
	);
	let boundary_interval = Vec3::new(
		if step.x == 0 { f32::INFINITY } else { chunk_size / direction.x.abs() },
		if step.y == 0 { f32::INFINITY } else { chunk_size / direction.y.abs() },
		if step.z == 0 { f32::INFINITY } else { chunk_size / direction.z.abs() },
	);
	let origin_boundary_mask = u8::from(origin.x == chunk.x as f32 * chunk_size)
		| (u8::from(origin.y == chunk.y as f32 * chunk_size) << 1)
		| (u8::from(origin.z == chunk.z as f32 * chunk_size) << 2);
	let stationary_boundary_mask = origin_boundary_mask & (
		u8::from(step.x == 0)
			| (u8::from(step.y == 0) << 1)
			| (u8::from(step.z == 0) << 2)
	);
	visit_chunk_boundary_neighbors(chunk, origin_boundary_mask, &mut visit);

	loop {
		let distance = boundary_distance.min_element();
		if distance > max_distance { break; }

		let crossing_mask = u8::from(boundary_distance.x == distance)
			| (u8::from(boundary_distance.y == distance) << 1)
			| (u8::from(boundary_distance.z == distance) << 2);
		for subset in 1u8..8 {
			if subset & !crossing_mask != 0 { continue; }
			let touched = chunk + IVec3::new(
				if subset & 1 != 0 { step.x } else { 0 },
				if subset & 2 != 0 { step.y } else { 0 },
				if subset & 4 != 0 { step.z } else { 0 },
			);
			visit_chunk_boundary_neighbors(touched, stationary_boundary_mask, &mut visit);
		}

		if crossing_mask & 1 != 0 {
			chunk.x += step.x;
			boundary_distance.x += boundary_interval.x;
		}
		if crossing_mask & 2 != 0 {
			chunk.y += step.y;
			boundary_distance.y += boundary_interval.y;
		}
		if crossing_mask & 4 != 0 {
			chunk.z += step.z;
			boundary_distance.z += boundary_interval.z;
		}
	}
}

fn visit_chunk_boundary_neighbors(chunk: IVec3, negative_axis_mask: u8, visit: &mut impl FnMut(IVec3)) {
	for subset in 0u8..8 {
		if subset & !negative_axis_mask != 0 { continue; }
		visit(chunk - IVec3::new(
			i32::from(subset & 1 != 0),
			i32::from(subset & 2 != 0),
			i32::from(subset & 4 != 0),
		));
	}
}

fn voxel_place_break_system(
	keys: Res<ButtonInput<KeyCode>>,
	egui_wants: Option<Res<EguiWantsInput>>,
	cameras: Query<(Entity, &Camera, &GlobalTransform), With<Camera3d>>,
	voxel_world: VoxelWorldQueryParam,
	grids: Query<(&GlobalTransform, &Grid)>,
	mut edits: GridStoreEditApi,
	mut sfx: Option<MessageWriter<PlaySfx>>,
) {
	if egui_wants.is_some_and(|e| e.wants_any_keyboard_input()) { return; }
	let place = keys.just_pressed(KeyCode::Space) || keys.pressed(KeyCode::KeyC);
	let destroy = keys.just_pressed(KeyCode::KeyX) || keys.pressed(KeyCode::KeyZ);
	if !place && !destroy { return; }

	let Some(ray) = player_ray(&cameras) else { return };
	let Some(hit) = voxel_world.raycast(ray.origin, ray.direction, None) else { return };

	let Ok((grid_global_transform, grid)) = grids.get(hit.grid) else { return };

	if place {
		let Some(voxel) = edit_voxel_for_grid(grid, PLACE_VOXEL) else { return };
		let pos = hit.voxel_pos + hit.normal;
		edits.apply(hit.grid, AddArea::new(NonZeroVoxelRegion::from_single(pos), voxel));
		if let Some(sfx) = &mut sfx {
			sfx.write(PlaySfx::block_place(grid_global_transform.transform_point(pos.as_vec3() + Vec3::splat(0.5))));
		}
	} else {
		edits.apply(hit.grid, RemoveArea::new(NonZeroVoxelRegion::from_single(hit.voxel_pos)));
		if let Some(sfx) = &mut sfx {
			sfx.write(PlaySfx::block_break(grid_global_transform.transform_point(hit.voxel_pos.as_vec3() + Vec3::splat(0.5))));
		}
	}
}

// fn sdf_place_system(
// 	keys: Res<ButtonInput<KeyCode>>,
// 	egui_wants: Option<Res<EguiWantsInput>>,
// 	cameras: Query<(Entity, &Camera, &GlobalTransform), With<Camera3d>>,
// 	voxel_world: VoxelWorldQueryParam,
// 	mut grids: Query<(&GlobalTransform, &Grid, &mut GridEditIdManager)>,
// 	mut sfx: Option<MessageWriter<PlaySfx>>,
// ) {
// 	if egui_wants.is_some_and(|e| e.wants_any_keyboard_input()) { return; }
// 	if !keys.just_pressed(KeyCode::KeyG) { return; }

// 	let Some(ray) = player_ray(&cameras) else { return };
// 	let Some(hit) = voxel_world.raycast(ray.origin, ray.direction, None) else { return };
// 	let Ok((grid_global_transform, grid, mut edits)) = grids.get_mut(hit.grid) else { return };
// 	let Some(voxel) = edit_voxel_for_grid(grid, PLACE_SDF_VOXEL) else { return };

// 	let center_voxel = hit.voxel_pos + hit.normal;
// 	let center = center_voxel.as_vec3() + Vec3::splat(0.5);
// 	let offsets = [
// 		Vec3::new(-16.0, 0.0, 0.0),
// 		Vec3::new(18.0, 8.0, 8.0),
// 		Vec3::new(7.0, -9.0, -18.0),
// 	];
// 	let centers: Vec<Vec3> = offsets.into_iter().map(|offset| center + offset).collect();
// 	let radius = (PLACE_SDF_RADIUS + 2.5) * 2.0;
// 	let smooth_k = 16.0;
// 	let reach = (PLACE_SDF_RADIUS + 20.0) * 2.0;
// 	let slider_radius = 9.0;
// 	let slider_half_length = 54.0;
// 	let sdf = Arc::new(move |p: Vec3| {
// 		let Some(first) = centers.first() else { return f32::INFINITY };
// 		let mut blob = (p - *first).length() - radius;
// 		for center in centers.iter().skip(1) {
// 			let b = (p - *center).length() - radius;
// 			let h = (0.5 + 0.5 * (b - blob) / smooth_k).clamp(0.0, 1.0);
// 			blob = b * (1.0 - h) + blob * h - smooth_k * h * (1.0 - h);
// 		}
// 		let q = p - center;
// 		let angle = std::f32::consts::FRAC_PI_4;
// 		let axis_q = Vec3::new(
// 			q.x,
// 			q.y * angle.cos() - q.z * angle.sin(),
// 			q.y * angle.sin() + q.z * angle.cos(),
// 		);
// 		let d = Vec2::new(Vec2::new(axis_q.x, axis_q.z).length(), axis_q.y.abs()) - Vec2::new(slider_radius, slider_half_length);
// 		let capsule = d.max(Vec2::ZERO).length() + d.x.max(d.y).min(0.0);
// 		blob.max(-capsule)
// 	});
// 	edits.apply_sdf(center - Vec3::splat(reach), center + Vec3::splat(reach), voxel, sdf);
// 	if let Some(sfx) = &mut sfx {
// 		sfx.write(PlaySfx::block_place(grid_global_transform.transform_point(center)));
// 	}
// }

fn player_ray(cameras: &Query<(Entity, &Camera, &GlobalTransform), With<Camera3d>>) -> Option<PlayerRay> {
	let (requester, _, camera_global_transform) = cameras.iter().find(|(_, camera, _)| camera.is_active)?;
	let transform = camera_global_transform.compute_transform();
	Some(PlayerRay {
		requester,
		origin: transform.translation,
		direction: transform.forward().as_vec3(),
	})
}

fn pickup_toggle_system(
	keys: Res<ButtonInput<KeyCode>>,
	egui_wants: Option<Res<EguiWantsInput>>,
	cameras: Query<(Entity, &Camera, &GlobalTransform), With<Camera3d>>,
	voxel_world: VoxelWorldQueryParam,
	parents: Query<&ChildOf>,
	bodies: Query<Has<IsStatic>, With<voxel_physics::RigidBody>>,
	mut held: ResMut<HeldBody>,
) {
	if egui_wants.is_some_and(|e| e.wants_any_keyboard_input()) { return; }
	if !keys.just_pressed(KeyCode::KeyF) { return; }
	if held.0.is_some() { held.0 = None; return; }

	let Some(ray) = player_ray(&cameras) else { return };
	let Some(hit) = voxel_world.raycast(ray.origin, ray.direction, None) else { return };
	let Ok(child_of) = parents.get(hit.grid) else { return };
	let body = child_of.parent();
	let Ok(is_static) = bodies.get(body) else { return };
	if is_static { return; }
	held.0 = Some(body);
}

fn push_system(
	keys: Res<ButtonInput<KeyCode>>,
	egui_wants: Option<Res<EguiWantsInput>>,
	cameras: Query<(Entity, &Camera, &GlobalTransform), With<Camera3d>>,
	voxel_world: VoxelWorldQueryParam,
	parents: Query<&ChildOf>,
	bodies: Query<(), (With<voxel_physics::RigidBody>, Without<IsStatic>)>,
	mut impulses: ResMut<Impulses>,
) {
	if egui_wants.is_some_and(|e| e.wants_any_keyboard_input()) { return; }
	if !keys.just_pressed(KeyCode::KeyR) { return; }
	let Some(ray) = player_ray(&cameras) else { return };
	let Some(hit) = voxel_world.raycast(ray.origin, ray.direction, None) else { return };
	let Ok(child_of) = parents.get(hit.grid) else { return };
	let body = child_of.parent();
	if bodies.get(body).is_err() { return; }
	impulses.apply_impulse(body, hit.world_position, ray.direction * PUSH_IMPULSE);
}

fn hold_held_body_system(
	held: Res<HeldBody>,
	time: Res<Time>,
	cameras: Query<(Entity, &Camera, &GlobalTransform), With<Camera3d>>,
	bodies: Query<(&Transform, &Velocity, &Mass, &CenterOfMass), (With<voxel_physics::RigidBody>, Without<IsStatic>)>,
	mut impulses: ResMut<Impulses>,
) {
	let Some(body_entity) = held.0 else { return };
	let Ok((transform, velocity, mass, com)) = bodies.get(body_entity) else { return };
	let Some(ray) = player_ray(&cameras) else { return };

	let target = ray.origin + ray.direction * HOLD_DISTANCE;
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
