use voxel_math::Fixed;
use voxel_transform::Transform;
use voxel_math::FixedVec3;
use std::collections::HashSet;

use bevy::prelude::*;

use rustc_hash::FxHashMap;
use voxel_transform::aabb::{aabb_corners, aabb_of_transformed_aabb};
use voxel_transform::bvh::BVH;
use tile_data::{CHUNK_SIZE, NonZeroChunkRegion, TileKey, chunks_covering_nonzero_voxel_region};
use voxel_query::OccupancyTileClass;
use voxel_sources::edit::GridEditMessage;
use voxel_streaming::{GridStreaming, TileLoadUpdate, TileRequester};

use crate::components::{IsStatic, RigidBody, VoxelCollider};

#[derive(Component, Default)]
pub struct PhysicsConsumer {
	pending: HashSet<(Entity, TileKey)>,
}

#[derive(Component, Default)]
pub struct WantedChunks(HashSet<IVec3>);

fn overlap(a: (FixedVec3, FixedVec3), b: (FixedVec3, FixedVec3)) -> bool {
	a.0.cmple(b.1).all() && b.0.cmple(a.1).all()
}

/// Cached grid-local AABB of a grid's present chunks.
#[derive(Component)]
pub struct PresenceAabb {
	lo: FixedVec3,
	hi: FixedVec3,
}

pub fn cache_presence_aabb(
	mut commands: Commands,
	grids: Query<(Entity, &GridStreaming), (With<VoxelCollider>, Without<PresenceAabb>)>,
) {
	for (entity, streaming) in grids.iter() {
		let mut min = IVec3::splat(i32::MAX);
		let mut max = IVec3::splat(i32::MIN);
		let mut any = false;
		for (origin, size) in streaming.presence().iter_present() {
			any = true;
			min = min.min(origin);
			max = max.max(origin + IVec3::splat(size as i32));
		}
		if !any {
			continue;
		}
		commands.entity(entity).insert((
			PresenceAabb {
				lo: FixedVec3::from(min) * Fixed::from_num(CHUNK_SIZE),
				hi: FixedVec3::from(max) * Fixed::from_num(CHUNK_SIZE),
			},
			WantedChunks::default(),
		));
	}
}

struct GridReq {
	entity: Entity,
	body: Entity,
	is_static: bool,
	grid_transform: Transform,
	grid_inverse: Transform,
	chunk_world_half: FixedVec3,
}

/// Expand a grid's present chunks into a flat list (a tree node covers `size^3`).
fn present_chunks(streaming: &GridStreaming) -> Vec<IVec3> {
	let mut chunks = Vec::new();
	for (origin, size) in streaming.presence().iter_present() {
		let size = size as i32;
		for dx in 0..size {
			for dy in 0..size {
				for dz in 0..size {
					chunks.push(origin + IVec3::new(dx, dy, dz));
				}
			}
		}
	}
	chunks
}

/// Non-static bodies request every present chunk of their collider grids;
/// static bodies request only chunks intersecting an overlapping body's AABB.
pub fn request_collision_chunks(
	bodies: Query<(&Transform, Has<IsStatic>), With<RigidBody>>,
	class: Res<OccupancyTileClass>,
	mut consumers: Query<(Entity, &mut PhysicsConsumer)>,
	mut streaming: ParamSet<(
		TileRequester,
		Query<(Entity, &ChildOf, &Transform, &GridStreaming, &PresenceAabb, &mut WantedChunks), With<VoxelCollider>>,
	)>,
) {
	let Ok((consumer_entity, mut consumer)) = consumers.single_mut() else { return };
	let mut grids = streaming.p1();

	// Pass 1: accumulate each body's world AABB from its grids' cached extents.
	let mut reqs: Vec<GridReq> = Vec::new();
	let mut body_aabb: FxHashMap<Entity, (FixedVec3, FixedVec3)> = FxHashMap::default();
	for (entity, child_of, local_tf, _, aabb, _) in grids.iter() {
		let body = child_of.parent();
		let Ok((body_tf, is_static)) = bodies.get(body) else { continue };
		crate::math::require_unit_scale(body_tf);
		crate::math::require_unit_scale(local_tf);
		let grid_tf = *body_tf * *local_tf;
		let (gmin, gmax) = aabb_of_transformed_aabb(&grid_tf, aabb.lo, aabb.hi);
		body_aabb
			.entry(body)
			.and_modify(|a| {
				a.0 = a.0.min(gmin);
				a.1 = a.1.max(gmax);
			})
			.or_insert((gmin, gmax));
		let chunk_half = Fixed::from_num(CHUNK_SIZE) / Fixed::from_num(2);
		let rotation = grid_tf.rotation;
		let chunk_world_half = ((rotation * FixedVec3::X).abs() + (rotation * FixedVec3::Y).abs() + (rotation * FixedVec3::Z).abs()) * chunk_half;
		reqs.push(GridReq {
			entity,
			body,
			is_static,
			grid_inverse: grid_tf.inverse(),
			grid_transform: grid_tf,
			chunk_world_half,
		});
	}
	let body_bvh = BVH::new(body_aabb.iter().map(|(&body, &aabb)| (body, aabb)).collect());

	// Pass 2: compute the wanted chunk set per grid under the policy above.
	let mut desired: FxHashMap<Entity, HashSet<IVec3>> = FxHashMap::default();
	for req in &reqs {
		let Ok((_, _, _, streaming, _, _)) = grids.get(req.entity) else { continue };
		let want = desired.entry(req.entity).or_default();
		if !req.is_static {
			want.extend(present_chunks(streaming));
			continue;
		}

		let mine = body_aabb[&req.body];
		let partners: Vec<(FixedVec3, FixedVec3)> = body_bvh
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
				let chunk = (req.grid_inverse.transform_point(corner) / Fixed::from_num(CHUNK_SIZE)).floor().as_ivec3();
				cmin = cmin.min(chunk);
				cmax = cmax.max(chunk);
			}
		}

		let chunk_local_center_offset = FixedVec3::splat(Fixed::from_num(CHUNK_SIZE) / Fixed::from_num(2));
		let mut skipped_regions: Vec<(IVec3, u32)> = Vec::new();
		streaming.presence().for_each_node_in_region(cmin, cmax, |origin, size, is_leaf| {
			let node_end = origin + IVec3::splat(size as i32);
			while let Some((skip_origin, skip_size)) = skipped_regions.last().copied() {
				let skip_end = skip_origin + IVec3::splat(skip_size as i32);
				if origin.cmpge(skip_origin).all() && node_end.cmple(skip_end).all() {
					break;
				}
				skipped_regions.pop();
			}
			if !skipped_regions.is_empty() {
				return;
			}

			let local_center = FixedVec3::from(origin) * Fixed::from_num(CHUNK_SIZE) + FixedVec3::splat(Fixed::from_num(CHUNK_SIZE) * Fixed::from_num(size) / Fixed::from_num(2));
			let world_center = req.grid_transform.transform_point(local_center);
			let node_world_half = req.chunk_world_half * Fixed::from_num(size);
			let node_box = (world_center - node_world_half, world_center + node_world_half);
			if !partners.iter().any(|p| overlap(node_box, *p)) {
				if !is_leaf {
					skipped_regions.push((origin, size));
				}
				return;
			}

			if !is_leaf {
				return;
			}
			if size == 1 {
				want.insert(origin);
				return;
			}

			for x in origin.x..origin.x + size as i32 {
				for y in origin.y..origin.y + size as i32 {
					for z in origin.z..origin.z + size as i32 {
						let chunk = IVec3::new(x, y, z);
						let local_center = FixedVec3::from(chunk) * Fixed::from_num(CHUNK_SIZE) + chunk_local_center_offset;
						let world_center = req.grid_transform.transform_point(local_center);
						let chunk_box = (world_center - req.chunk_world_half, world_center + req.chunk_world_half);
						if partners.iter().any(|p| overlap(chunk_box, *p)) {
							want.insert(chunk);
						}
					}
				}
			}
		});
	}

	// Pass 3: diff against last tick — fetch newly wanted occupancy tiles and
	// release dropped ones. Grids absent from `desired` release everything.
	let mut acquisitions = Vec::new();
	let mut releases = Vec::new();
	for (entity, _, _, _, _, mut wanted) in grids.iter_mut() {
		let want = desired.remove(&entity).unwrap_or_default();
		acquisitions.extend(want.difference(&wanted.0).map(|&chunk| {
			(entity, TileKey::new(NonZeroChunkRegion::from_single(chunk), 0, class.0))
		}));
		releases.extend(wanted.0.difference(&want).map(|&chunk| {
			(entity, TileKey::new(NonZeroChunkRegion::from_single(chunk), 0, class.0))
		}));
		wanted.0 = want;
	}
	drop(grids);

	let mut requester = streaming.p0();
	for (grid, key) in releases {
		consumer.pending.remove(&(grid, key));
		requester.release_tile(grid, consumer_entity, key);
	}
	for (grid, key) in acquisitions {
		if requester.fetch_tile(grid, consumer_entity, key, Fixed::ZERO, true, None) {
			consumer.pending.insert((grid, key));
		}
	}
}

pub fn mark_edited_collision_tiles_pending(
	mut edits: MessageReader<GridEditMessage>,
	class: Res<OccupancyTileClass>,
	grids: Query<&WantedChunks>,
	mut consumers: Query<&mut PhysicsConsumer>,
) {
	let Ok(mut consumer) = consumers.single_mut() else { return };
	for edit in edits.read() {
		let Ok(wanted) = grids.get(edit.grid_id()) else { continue };
		let affected = chunks_covering_nonzero_voxel_region(edit.edit().affected_region());
		for chunk in wanted.0.iter().copied().filter(|chunk| affected.contains(*chunk)) {
			consumer.pending.insert((edit.grid_id(), TileKey::new(NonZeroChunkRegion::from_single(chunk), 0, class.0)));
		}
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
	mut updates: MessageReader<TileLoadUpdate>,
	mut consumers: Query<(Entity, &mut PhysicsConsumer)>,
) {
	let Ok((consumer_entity, mut consumer)) = consumers.single_mut() else { return };
	for update in updates.read() {
		if update.requester == consumer_entity {
			consumer.pending.remove(&(update.grid, update.key));
		}
	}
}

pub fn collision_tiles_ready(consumers: Query<&PhysicsConsumer>) -> bool {
	consumers.single().is_ok_and(|consumer| consumer.pending.is_empty())
}
