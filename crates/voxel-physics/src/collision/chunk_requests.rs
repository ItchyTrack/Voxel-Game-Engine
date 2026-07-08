use std::collections::{HashMap, HashSet};

use bevy::math::Affine3A;
use bevy::prelude::*;

use voxel_data::aabb::{aabb_corners, aabb_of_transformed_aabb};
use voxel_data::bvh::BVH;
use voxel_streaming::{GridStreaming, VoxelSourceRequests, CHUNK_SIZE};

use crate::components::{IsStatic, RigidBody, VoxelCollider};

voxel_streaming::chunk_consumer!(pub PhysicsConsumer);

#[derive(Component, Default)]
pub struct WantedChunks(HashSet<IVec3>);

fn overlap(a: (Vec3, Vec3), b: (Vec3, Vec3)) -> bool {
	a.0.cmple(b.1).all() && b.0.cmple(a.1).all()
}

/// Cached grid-local AABB of a grid's present chunks.
#[derive(Component)]
pub struct PresenceAabb {
	lo: Vec3,
	hi: Vec3,
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
				lo: (min * CHUNK_SIZE).as_vec3(),
				hi: (max * CHUNK_SIZE).as_vec3(),
			},
			WantedChunks::default(),
		));
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
	mut grids: Query<(Entity, &ChildOf, &Transform, &mut GridStreaming, &PresenceAabb, &mut WantedChunks), With<VoxelCollider>>,
	mut consumers: Query<&mut PhysicsConsumer>,
	requests: VoxelSourceRequests,
) {
	let Ok(mut consumer) = consumers.single_mut() else { return };

	// Pass 1: accumulate each body's world AABB from its grids' cached extents.
	let mut reqs: Vec<GridReq> = Vec::new();
	let mut body_aabb: HashMap<Entity, (Vec3, Vec3)> = HashMap::new();
	for (entity, child_of, local_tf, _, aabb, _) in grids.iter() {
		let body = child_of.parent();
		let Ok((body_tf, is_static)) = bodies.get(body) else { continue };
		let grid_tf = *body_tf * *local_tf;
		let grid_affine = grid_tf.compute_affine();
		let (gmin, gmax) = aabb_of_transformed_aabb(&grid_tf, aabb.lo, aabb.hi);
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
	let mut desired: HashMap<Entity, HashSet<IVec3>> = HashMap::new();
	for req in &reqs {
		let Ok((_, _, _, streaming, _, _)) = grids.get(req.entity) else { continue };
		let want = desired.entry(req.entity).or_default();
		if !req.is_static {
			want.extend(present_chunks(streaming));
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

		let chunk_local_center_offset = Vec3::splat(CHUNK_SIZE as f32 * 0.5);
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

			let local_center = (origin * CHUNK_SIZE).as_vec3() + Vec3::splat(CHUNK_SIZE as f32 * size as f32 * 0.5);
			let world_center = req.grid_affine.transform_point3(local_center);
			let node_world_half = req.chunk_world_half * size as f32;
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
						let local_center = (chunk * CHUNK_SIZE).as_vec3() + chunk_local_center_offset;
						let world_center = req.grid_affine.transform_point3(local_center);
						let chunk_box = (world_center - req.chunk_world_half, world_center + req.chunk_world_half);
						if partners.iter().any(|p| overlap(chunk_box, *p)) {
							want.insert(chunk);
						}
					}
				}
			}
		});
	}

	// Pass 3: diff against last tick — fetch newly wanted chunks, release dropped
	// ones. Grids absent from `desired` (their body vanished) release everything.
	for (entity, _, _, mut streaming, _, mut wanted) in grids.iter_mut() {
		let want = desired.remove(&entity).unwrap_or_default();
		for &chunk in want.difference(&wanted.0) {
			streaming.fetch_needed(entity, consumer.as_mut(), &requests, chunk);
		}
		for &chunk in wanted.0.difference(&want) {
			streaming.release_needed(entity, consumer.as_mut(), chunk);
		}
		wanted.0 = want;
	}
}
