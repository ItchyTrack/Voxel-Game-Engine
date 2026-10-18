use std::collections::HashSet;

use bevy::math::Affine3A;
use bevy::prelude::*;

use bevy::ecs::component::ComponentIdFor;
use bile_math::NonZeroRegion;
use bile_scene::NodeBounds;
use bile_tiles::{TileClassReady, TileInstances, TileKey};
use rustc_hash::FxHashMap;
use voxel_data::aabb::{aabb_corners, aabb_of_transformed_aabb};
use voxel_data::bvh::BVH;
use tile_data::{CHUNK_SIZE, chunk_origin, chunk_region, chunks_in};
use voxel_query::OccupancyTileClass;

use crate::components::{IsStatic, RigidBody, VoxelCollider};

#[derive(Component, Default)]
pub struct PhysicsConsumer {
	pending: HashSet<(Entity, TileKey)>,
}

#[derive(Component, Default)]
pub struct WantedChunks(HashSet<IVec3>);

fn overlap(a: (Vec3, Vec3), b: (Vec3, Vec3)) -> bool {
	a.0.cmple(b.1).all() && b.0.cmple(a.1).all()
}

pub fn ensure_wanted_chunks(
	mut commands: Commands,
	grids: Query<Entity, (With<VoxelCollider>, With<NodeBounds>, Without<WantedChunks>)>,
) {
	for entity in &grids {
		commands.entity(entity).insert(WantedChunks::default());
	}
}

struct GridReq {
	entity: Entity,
	body: Entity,
	is_static: bool,
	grid_affine: Affine3A,
	grid_inv_affine: Affine3A,
	chunk_world_half: Vec3,
}

/// Non-static bodies request every present chunk of their collider grids;
/// static bodies request only chunks intersecting an overlapping body's AABB.
pub fn request_collision_chunks(
	bodies: Query<(&Transform, Has<IsStatic>), With<RigidBody>>,
	class: ComponentIdFor<OccupancyTileClass>,
	mut consumers: Query<&mut PhysicsConsumer>,
	mut grids: Query<(Entity, &ChildOf, &Transform, &NodeBounds, &mut TileInstances, &mut WantedChunks), With<VoxelCollider>>,
) {
	let Ok(mut consumer) = consumers.single_mut() else { return };

	// Pass 1: accumulate each body's world AABB from its grids' cached extents.
	let mut reqs: Vec<GridReq> = Vec::new();
	let mut body_aabb: FxHashMap<Entity, (Vec3, Vec3)> = FxHashMap::default();
	for (entity, child_of, local_tf, bounds, _, _) in grids.iter() {
		let body = child_of.parent();
		let Ok((body_tf, is_static)) = bodies.get(body) else { continue };
		let grid_tf = *body_tf * *local_tf;
		let grid_affine = grid_tf.compute_affine();
		let (gmin, gmax) = aabb_of_transformed_aabb(&grid_tf, bounds.region().min().as_vec3(), bounds.region().end().as_vec3());
		body_aabb
			.entry(body)
			.and_modify(|a| {
				a.0 = a.0.min(gmin);
				a.1 = a.1.max(gmax);
			})
			.or_insert((gmin, gmax));
		let chunk_half = Vec3::splat(CHUNK_SIZE as f32 * 0.5);
		let chunk_world_half = (grid_affine.matrix3.x_axis.abs() * chunk_half.x
			+ grid_affine.matrix3.y_axis.abs() * chunk_half.y
			+ grid_affine.matrix3.z_axis.abs() * chunk_half.z)
			.into();
		reqs.push(GridReq {
			entity,
			body,
			is_static,
			grid_inv_affine: grid_affine.inverse(),
			grid_affine,
			chunk_world_half,
		});
	}
	let body_bvh = BVH::new(body_aabb.iter().map(|(&body, &aabb)| (body, aabb)).collect());

	// Pass 2: compute the wanted chunk set per grid under the policy above.
	let mut desired: FxHashMap<Entity, HashSet<IVec3>> = FxHashMap::default();
	for req in &reqs {
		let Ok((_, _, _, bounds, _, _)) = grids.get(req.entity) else { continue };
		let want = desired.entry(req.entity).or_default();
		if !req.is_static {
			want.extend(chunks_in(bounds.region()));
			continue;
		}

		let mine = body_aabb[&req.body];
		let partners: Vec<(Vec3, Vec3)> = body_bvh
			.collisions(&mine)
			.into_iter()
			.filter(|body| *body != req.body)
			.filter_map(|body| body_aabb.get(&body).copied())
			.collect();
		if partners.is_empty() {
			continue;
		}

		// Local chunk-space region covering every partner AABB. The grid may be
		// rotated, so take the AABB of the inverse-transformed partner corners.
		let mut cmin = IVec3::splat(i32::MAX);
		let mut cmax = IVec3::splat(i32::MIN);
		for (pmin, pmax) in &partners {
			for corner in aabb_corners(*pmin, *pmax) {
				let chunk = (req.grid_inv_affine.transform_point3(corner) / CHUNK_SIZE as f32).floor().as_ivec3();
				cmin = cmin.min(chunk);
				cmax = cmax.max(chunk);
			}
		}

		let Some(partner_region) = NonZeroRegion::from_min_max(chunk_origin(cmin), chunk_origin(cmax)) else { continue };
		let Some(region) = bounds.region().intersection(partner_region) else { continue };
		let chunk_local_center_offset = Vec3::splat(CHUNK_SIZE as f32 * 0.5);
		for chunk in chunks_in(region) {
			let local_center = chunk_origin(chunk).as_vec3() + chunk_local_center_offset;
			let world_center = req.grid_affine.transform_point3(local_center);
			let chunk_box = (world_center - req.chunk_world_half, world_center + req.chunk_world_half);
			if partners.iter().any(|p| overlap(chunk_box, *p)) {
				want.insert(chunk);
			}
		}
	}

	// Pass 3: diff against last tick — fetch newly wanted occupancy tiles and
	// release dropped ones. Grids absent from `desired` release everything.
	for (entity, _, _, _, mut instances, mut wanted) in grids.iter_mut() {
		let want = desired.remove(&entity).unwrap_or_default();
		for &chunk in wanted.0.difference(&want) {
			let key = TileKey::new(chunk_region(chunk), 0);
			consumer.pending.remove(&(entity, key));
			instances.remove_class(key, class.get());
		}
		for &chunk in want.difference(&wanted.0) {
			let key = TileKey::new(chunk_region(chunk), 0);
			consumer.pending.insert((entity, key));
			instances.add_class(key, class.get());
		}
		wanted.0 = want;
	}
}

pub fn remove_stale_collision_requests(
	grids: Query<(), With<VoxelCollider>>,
	mut consumers: Query<&mut PhysicsConsumer>,
) {
	let Ok(mut consumer) = consumers.single_mut() else { return };
	consumer.pending.retain(|(grid, _)| grids.get(*grid).is_ok());
}

pub fn receive_collision_tiles(
	mut ready: MessageReader<TileClassReady>,
	class: ComponentIdFor<OccupancyTileClass>,
	mut consumers: Query<&mut PhysicsConsumer>,
) {
	let Ok(mut consumer) = consumers.single_mut() else { return };
	for ready in ready.read() {
		if ready.class == class.get() {
			consumer.pending.remove(&(ready.entity, ready.tile_key));
		}
	}
}

pub fn collision_tiles_ready(consumers: Query<&PhysicsConsumer>) -> bool {
	consumers.single().is_ok_and(|consumer| consumer.pending.is_empty())
}
