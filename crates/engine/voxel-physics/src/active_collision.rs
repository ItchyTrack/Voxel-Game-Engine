use bevy::prelude::*;
use tile_data::{CHUNK_SIZE, DynamicTileData, LoadedTile};
use voxel_data::aabb::aabb_of_transformed_aabb;
use voxel_data::bvh::BVH;
use voxel_query::OccupancyTileData;

use crate::collision::{Collision, Collisions, HalfCollision};
use crate::GridId;
use crate::components::{IsStatic, RigidBody, VoxelCollider};
use crate::narrowphase::get_collisions_between_tiles;
use crate::transform_ext::TransformExt;

struct TileCollider<'a> {
	body: Entity,
	body_transform: Transform,
	grid: GridId,
	tree: &'a voxel_query::OccupancyTree,
	transform: Transform,
	tile_origin: IVec3,
	is_static: bool,
}

pub(crate) fn detect_collisions(
	mut collisions: ResMut<Collisions>,
	bodies: Query<(Entity, &Transform, Has<IsStatic>), With<RigidBody>>,
	grids: Query<(&Transform, &ChildOf), With<VoxelCollider>>,
	tiles: Query<(&LoadedTile, &DynamicTileData)>,
) {
	let mut colliders = Vec::new();
	for (loaded, data) in &tiles {
		let Some(occupancy) = data.downcast_ref::<OccupancyTileData>() else { continue };
		let Ok((grid_transform, parent)) = grids.get(loaded.grid) else { continue };
		let Ok((body, body_transform, is_static)) = bodies.get(parent.parent()) else { continue };
		if !body_transform.scale.abs_diff_eq(Vec3::ONE, 1e-5)
			|| !grid_transform.scale.abs_diff_eq(Vec3::ONE, 1e-5)
		{
			continue;
		}
		let tile_origin = loaded.key.region.min() * CHUNK_SIZE as i32;
		let transform = *body_transform * *grid_transform * Transform::from_translation(tile_origin.as_vec3());
		colliders.push(TileCollider {
			body,
			body_transform: *body_transform,
			grid: loaded.grid,
			tree: &occupancy.tree,
			transform,
			tile_origin,
			is_static,
		});
	}

	let bounds = colliders.iter().enumerate().filter_map(|(index, collider)| {
		let bounds = collider.tree.occupied_bounds()?;
		Some((index, aabb_of_transformed_aabb(&collider.transform, bounds.min().as_vec3(), bounds.end().as_vec3())))
	}).collect();
	let bvh = BVH::new(bounds);
	let mut detected = Vec::new();

	for (index_a, collider_a) in colliders.iter().enumerate() {
		if collider_a.is_static { continue; }
		let Some(bounds_a) = collider_a.tree.occupied_bounds() else { continue };
		let world_bounds_a = aabb_of_transformed_aabb(&collider_a.transform, bounds_a.min().as_vec3(), bounds_a.end().as_vec3());
		for index_b in bvh.collisions(&world_bounds_a) {
			let collider_b = &colliders[index_b];
			if collider_a.body == collider_b.body { continue; }
			if !collider_b.is_static && index_b <= index_a { continue; }

			let transform_a_in_b = collider_b.transform.inverse() * collider_a.transform;
			for contact in get_collisions_between_tiles(collider_a.tree, collider_b.tree, &transform_a_in_b) {
				let collision_1 = collider_b.transform * contact.0;
				let collision_2 = collider_b.transform * contact.2;
				detected.push(Collision {
					part1: HalfCollision {
						body_id: collider_a.body,
						grid_id: collider_a.grid,
						voxel_pos: collider_a.tile_origin + contact.4.as_ivec3(),
						feature: contact.1,
						collision: collision_1,
						local_collision: collider_a.body_transform.inverse() * collision_1,
					},
					part2: HalfCollision {
						body_id: collider_b.body,
						grid_id: collider_b.grid,
						voxel_pos: collider_b.tile_origin + contact.5.as_ivec3(),
						feature: contact.3,
						collision: collision_2,
						local_collision: collider_b.body_transform.inverse() * collision_2,
					},
				});
			}
		}
	}

	collisions.0 = detected;
}
