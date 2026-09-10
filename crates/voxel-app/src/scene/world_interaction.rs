use std::collections::HashSet;

use bevy::ecs::message::MessageWriter;
use bevy::input::ButtonInput;
use bevy::prelude::*;
use voxel_math::{Fixed, FixedVec3};
use voxel_transform::TransformQuery;
use bevy_egui::input::EguiWantsInput;

use tile_data::{CHUNK_SIZE, NonZeroChunkRegion, TileKey};
use voxel_content::grid_store::GridStoreEditApi;
use voxel_trees::region::NonZeroVoxelRegion;
use voxel_data::grid::{Grid, GridId};
use voxel_data::voxels::{Voxel, VoxelType};
use voxel_query::{OccupancyTileClass, VoxelWorldQueryParam};
use voxel_sources::edit::{AddArea, RemoveArea};
use voxel_mass::{CenterOfMass, Mass};
use voxel_physics::{FreezePhysics, Impulses, IsStatic, Velocity, VoxelPhysicsAppExt};
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
	origin: FixedVec3,
	direction: FixedVec3,
}

const RAY_OCCUPANCY_DISTANCE: Fixed = Fixed::from_bits(200 << 24);
const HOLD_DISTANCE: Fixed = Fixed::from_bits(40 << 24);
const PUSH_IMPULSE: Fixed = Fixed::from_bits(1_600_000 << 24);
const MAX_GRAB_ACCEL: Fixed = Fixed::from_bits(8_000 << 24);

const PLACE_VOXEL: BasicVoxel = BasicVoxel { color: [180, 180, 180, 255], mass: 100 };
const PLACE_SDF_VOXEL: BasicVoxel = BasicVoxel { color: [80, 180, 255, 255], mass: 100 };
const PLACE_SDF_RADIUS: Fixed = Fixed::from_bits(5 << 24);

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
	cameras: Query<(Entity, &Camera), With<Camera3d>>,
	transforms: TransformQuery,
	class: Res<OccupancyTileClass>,
	mut requested: ResMut<RayOccupancyRequests>,
	mut streaming: ParamSet<(
		TileRequester,
		Query<(GridId, &GridStreaming)>,
	)>,
) {
	let mut desired = HashSet::new();
	if let Some(ray) = player_ray(&cameras, &transforms) {
		let grids = streaming.p1();
		for (grid, grid_streaming) in &grids {
			let Some(grid_transform) = transforms.get_world(grid) else { continue };
			let local_origin = grid_transform.inverse_transform_point(ray.origin);
			let local_direction = grid_transform.inverse_transform_vector(ray.direction);
			let local_distance = RAY_OCCUPANCY_DISTANCE / grid_transform.scale.to_fixed();
			for_each_ray_chunk(local_origin, local_direction, local_distance, |chunk| {
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
		if requester.fetch_tile(request.grid, request.requester, request.key, Fixed::ZERO, false, None) {
			retained.insert(request);
		}
	}
	for request in releases {
		requester.release_tile(request.grid, request.requester, request.key);
	}
	requested.0 = retained;
}

fn for_each_ray_chunk(origin: FixedVec3, direction: FixedVec3, max_distance: Fixed, mut visit: impl FnMut(IVec3)) {
	let direction = direction.normalize_or_zero();
	if direction == FixedVec3::ZERO { return; }

	let chunk_size = Fixed::from_num(CHUNK_SIZE);
	let mut chunk = (origin / chunk_size).floor().as_ivec3();
	let step = direction.signum().as_ivec3();
	let next_boundary = FixedVec3::from(IVec3::new(
		if step.x > 0 { chunk.x + 1 } else { chunk.x },
		if step.y > 0 { chunk.y + 1 } else { chunk.y },
		if step.z > 0 { chunk.z + 1 } else { chunk.z },
	)) * chunk_size;
	let mut boundary_distance = FixedVec3::new(
		if step.x == 0 { Fixed::MAX } else { (next_boundary.x - origin.x) / direction.x },
		if step.y == 0 { Fixed::MAX } else { (next_boundary.y - origin.y) / direction.y },
		if step.z == 0 { Fixed::MAX } else { (next_boundary.z - origin.z) / direction.z },
	);
	let boundary_interval = FixedVec3::new(
		if step.x == 0 { Fixed::MAX } else { chunk_size / direction.x.abs() },
		if step.y == 0 { Fixed::MAX } else { chunk_size / direction.y.abs() },
		if step.z == 0 { Fixed::MAX } else { chunk_size / direction.z.abs() },
	);
	let origin_boundary_mask = u8::from(origin.x == Fixed::from_num(chunk.x) * chunk_size)
		| (u8::from(origin.y == Fixed::from_num(chunk.y) * chunk_size) << 1)
		| (u8::from(origin.z == Fixed::from_num(chunk.z) * chunk_size) << 2);
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
	cameras: Query<(Entity, &Camera), With<Camera3d>>,
	transforms: TransformQuery,
	voxel_world: VoxelWorldQueryParam,
	grids: Query<&Grid>,
	mut edits: GridStoreEditApi,
	mut sfx: Option<MessageWriter<PlaySfx>>,
) {
	if egui_wants.is_some_and(|e| e.wants_any_keyboard_input()) { return; }
	let place = keys.just_pressed(KeyCode::Space) || keys.pressed(KeyCode::KeyC);
	let destroy = keys.just_pressed(KeyCode::KeyX) || keys.pressed(KeyCode::KeyZ);
	if !place && !destroy { return; }

	let Some(ray) = player_ray(&cameras, &transforms) else { return };
	let Some(hit) = voxel_world.raycast(ray.origin, ray.direction, None) else { return };

	let (Some(grid_global_transform), Ok(grid)) = (transforms.get_world(hit.grid), grids.get(hit.grid)) else { return };

	if place {
		let Some(voxel) = edit_voxel_for_grid(grid, PLACE_VOXEL) else { return };
		let pos = hit.voxel_pos + hit.normal;
		edits.apply(hit.grid, AddArea::new(NonZeroVoxelRegion::from_single(pos), voxel));
		if let Some(sfx) = &mut sfx {
			sfx.write(PlaySfx::block_place(grid_global_transform.transform_point(FixedVec3::from(pos) + FixedVec3::splat(Fixed::from_num(0.5)))));
		}
	} else {
		edits.apply(hit.grid, RemoveArea::new(NonZeroVoxelRegion::from_single(hit.voxel_pos)));
		if let Some(sfx) = &mut sfx {
			sfx.write(PlaySfx::block_break(grid_global_transform.transform_point(FixedVec3::from(hit.voxel_pos) + FixedVec3::splat(Fixed::from_num(0.5)))));
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

fn player_ray(cameras: &Query<(Entity, &Camera), With<Camera3d>>, transforms: &TransformQuery) -> Option<PlayerRay> {
	let (requester, _) = cameras.iter().find(|(_, camera)| camera.is_active)?;
	let transform = transforms.get_world(requester)?;
	Some(PlayerRay {
		requester,
		origin: transform.translation,
		direction: transform.forward(),
	})
}

fn pickup_toggle_system(
	keys: Res<ButtonInput<KeyCode>>,
	egui_wants: Option<Res<EguiWantsInput>>,
	cameras: Query<(Entity, &Camera), With<Camera3d>>,
	transforms: TransformQuery,
	voxel_world: VoxelWorldQueryParam,
	parents: Query<&ChildOf>,
	bodies: Query<Has<IsStatic>, With<voxel_physics::RigidBody>>,
	mut held: ResMut<HeldBody>,
) {
	if egui_wants.is_some_and(|e| e.wants_any_keyboard_input()) { return; }
	if !keys.just_pressed(KeyCode::KeyF) { return; }
	if held.0.is_some() { held.0 = None; return; }

	let Some(ray) = player_ray(&cameras, &transforms) else { return };
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
	cameras: Query<(Entity, &Camera), With<Camera3d>>,
	transforms: TransformQuery,
	voxel_world: VoxelWorldQueryParam,
	parents: Query<&ChildOf>,
	bodies: Query<(), (With<voxel_physics::RigidBody>, Without<IsStatic>)>,
	mut impulses: ResMut<Impulses>,
) {
	if egui_wants.is_some_and(|e| e.wants_any_keyboard_input()) { return; }
	if !keys.just_pressed(KeyCode::KeyR) { return; }
	let Some(ray) = player_ray(&cameras, &transforms) else { return };
	let Some(hit) = voxel_world.raycast(ray.origin, ray.direction, None) else { return };
	let Ok(child_of) = parents.get(hit.grid) else { return };
	let body = child_of.parent();
	if bodies.get(body).is_err() { return; }
	impulses.apply_impulse(body, hit.world_position, ray.direction * PUSH_IMPULSE);
}

fn hold_held_body_system(
	held: Res<HeldBody>,
	time: Res<Time>,
	cameras: Query<(Entity, &Camera), With<Camera3d>>,
	transforms: TransformQuery,
	bodies: Query<(&Velocity, &Mass, &CenterOfMass), (With<voxel_physics::RigidBody>, Without<IsStatic>)>,
	mut impulses: ResMut<Impulses>,
) {
	let Some(body_entity) = held.0 else { return };
	let Ok((velocity, mass, com)) = bodies.get(body_entity) else { return };
	let Some(transform) = transforms.get_world(body_entity) else { return };
	let Some(ray) = player_ray(&cameras, &transforms) else { return };

	let target = ray.origin + ray.direction * HOLD_DISTANCE;
	let body_com_world = transform * com.0;
	let offset = target - body_com_world;
	if offset.length_squared() < Fixed::from_num(1e-6) { return; }
	let dir = offset.normalize();
	let velocity_in_dir = velocity.0.dot(dir);
	let delta_v = dir * (offset.length() * Fixed::from_num(4) - velocity_in_dir * Fixed::from_num(0.5))
		- (velocity.0 - dir * velocity_in_dir);
	let dt = Fixed::from_num(time.delta().as_nanos()) / Fixed::from_num(1_000_000_000);
	let delta_v = FixedVec3::ZERO.move_towards(delta_v, MAX_GRAB_ACCEL * dt);
	impulses.apply_central_impulse(body_entity, Fixed::from_num(mass.0) * delta_v);
}
