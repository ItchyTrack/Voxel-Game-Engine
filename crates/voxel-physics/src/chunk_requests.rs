use std::collections::{HashMap, HashSet};

use bevy::prelude::*;

use voxel_data::grid::Grid;
use voxel_streaming::{ChunkRequestChannel, GridStreaming, CHUNK_SIZE};

use crate::components::{IsStatic, RigidBody, VoxelCollider};

voxel_streaming::chunk_consumer!(pub PhysicsConsumer);

#[derive(Component, Default)]
pub struct WantedChunks(HashSet<IVec3>);

fn box_corners(lo: Vec3, hi: Vec3) -> [Vec3; 8] {
	[
		Vec3::new(lo.x, lo.y, lo.z),
		Vec3::new(hi.x, lo.y, lo.z),
		Vec3::new(lo.x, hi.y, lo.z),
		Vec3::new(lo.x, lo.y, hi.z),
		Vec3::new(hi.x, hi.y, lo.z),
		Vec3::new(hi.x, lo.y, hi.z),
		Vec3::new(lo.x, hi.y, hi.z),
		Vec3::new(hi.x, hi.y, hi.z),
	]
}

fn transform_box(tf: &Transform, lo: Vec3, hi: Vec3) -> (Vec3, Vec3) {
	let mut min = Vec3::splat(f32::MAX);
	let mut max = Vec3::splat(f32::MIN);
	for corner in box_corners(lo, hi) {
		let w = *tf * corner;
		min = min.min(w);
		max = max.max(w);
	}
	(min, max)
}

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
	grid_tf: Transform,
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
	mut grids: Query<(Entity, &ChildOf, &Transform, &mut GridStreaming, &PresenceAabb, &mut Grid, &mut WantedChunks), With<VoxelCollider>>,
	mut consumers: Query<&mut PhysicsConsumer>,
	channel: Res<ChunkRequestChannel>,
) {
	let Ok(mut consumer) = consumers.single_mut() else { return };

	// Pass 1: accumulate each body's world AABB from its grids' cached extents.
	let mut reqs: Vec<GridReq> = Vec::new();
	let mut body_aabb: HashMap<Entity, (Vec3, Vec3)> = HashMap::new();
	for (entity, child_of, local_tf, _, aabb, _, _) in grids.iter() {
		let body = child_of.parent();
		let Ok((body_tf, is_static)) = bodies.get(body) else { continue };
		let grid_tf = *body_tf * *local_tf;
		let (gmin, gmax) = transform_box(&grid_tf, aabb.lo, aabb.hi);
		body_aabb
			.entry(body)
			.and_modify(|a| {
				a.0 = a.0.min(gmin);
				a.1 = a.1.max(gmax);
			})
			.or_insert((gmin, gmax));
		reqs.push(GridReq { entity, body, is_static, grid_tf });
	}

	// Pass 2: compute the wanted chunk set per grid under the policy above.
	let mut desired: HashMap<Entity, HashSet<IVec3>> = HashMap::new();
	for req in &reqs {
		let Ok((_, _, _, streaming, _, _, _)) = grids.get(req.entity) else { continue };
		let want = desired.entry(req.entity).or_default();
		if !req.is_static {
			want.extend(present_chunks(&streaming));
			continue;
		}

		let mine = body_aabb[&req.body];
		let partners: Vec<(Vec3, Vec3)> = body_aabb
			.iter()
			.filter(|(body, aabb)| **body != req.body && overlap(mine, **aabb))
			.map(|(_, aabb)| *aabb)
			.collect();
		if partners.is_empty() {
			continue;
		}

		// Local chunk-space region covering every partner AABB. The grid may be
		// rotated, so take the AABB of the inverse-transformed partner corners.
		let inv = req.grid_tf.compute_affine().inverse();
		let mut cmin = IVec3::splat(i32::MAX);
		let mut cmax = IVec3::splat(i32::MIN);
		for (pmin, pmax) in &partners {
			for corner in box_corners(*pmin, *pmax) {
				let chunk = (inv.transform_point3(corner) / CHUNK_SIZE as f32).floor().as_ivec3();
				cmin = cmin.min(chunk);
				cmax = cmax.max(chunk);
			}
		}

		let mut candidates = Vec::new();
		streaming.presence().for_each_in_region(cmin, cmax, |chunk| candidates.push(chunk));
		for chunk in candidates {
			let lo = (chunk * CHUNK_SIZE).as_vec3();
			let hi = ((chunk + IVec3::ONE) * CHUNK_SIZE).as_vec3();
			let chunk_box = transform_box(&req.grid_tf, lo, hi);
			if partners.iter().any(|p| overlap(chunk_box, *p)) {
				want.insert(chunk);
			}
		}
	}

	// Pass 3: diff against last tick — fetch newly wanted chunks, release dropped
	// ones. Grids absent from `desired` (their body vanished) release everything.
	for (entity, _, _, mut streaming, _, mut grid, mut wanted) in grids.iter_mut() {
		let want = desired.remove(&entity).unwrap_or_default();
		for &chunk in want.difference(&wanted.0) {
			streaming.fetch_needed(entity, consumer.as_mut(), &channel, chunk);
		}
		for &chunk in wanted.0.difference(&want) {
			streaming.release(chunk, grid.as_mut());
		}
		wanted.0 = want;
	}
}
