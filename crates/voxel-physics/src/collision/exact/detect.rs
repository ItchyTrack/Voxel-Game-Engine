use std::collections::HashMap;

use bevy::prelude::*;
use num::Zero;
use voxel_data::bvh::BVH;
use voxel_data::grid::Grid;
use voxel_data::subgrid::{SubGrid, SubGridId};

use crate::collision::{BodyView, Collisions, GridCollider};
use crate::components::{IsStatic, Mass, RigidBody, VoxelCollider};
use crate::sparse_set::SparseSet;
use crate::{GridId, PhysicsBodyId};

use super::broadphase::get_collisions;

pub fn detect_collisions(
	mut collisions: ResMut<Collisions>,
	bodies: Query<(Entity, &Transform, &Mass, Has<IsStatic>), (With<RigidBody>, Without<Grid>)>,
	grid_entities: Query<(Entity, &Transform, &ChildOf, &Grid), With<VoxelCollider>>,
	sub_grid_query: Query<(Entity, &SubGrid)>,
) {
	let mut body_views: SparseSet<PhysicsBodyId, BodyView> = SparseSet::with_capacity(bodies.iter().count());
	for (entity, transform, mass, is_static) in bodies.iter() {
		body_views.insert(entity, BodyView { transform: *transform, is_static: is_static || mass.0.is_zero() });
	}

	let mut subgrids_by_grid: HashMap<GridId, Vec<(SubGridId, &SubGrid)>> = HashMap::new();
	for (sub_grid_id, sub_grid) in sub_grid_query.iter() {
		subgrids_by_grid.entry(sub_grid.grid()).or_default().push((sub_grid_id, sub_grid));
	}

	let mut grids: SparseSet<GridId, GridCollider> = SparseSet::with_capacity(grid_entities.iter().count());
	for (grid_entity, transform, child_of, grid) in grid_entities.iter() {
		let body = child_of.parent();
		if !body_views.contains_key(&body) { continue; }
		let sub_grids = subgrids_by_grid
			.remove(&grid_entity)
			.unwrap_or_default()
			.into_iter()
			.filter_map(|(id, sub_grid)| grid.view(sub_grid).map(|view| (id, view)))
			.collect();
		grids.insert(grid_entity, GridCollider { body, local_transform: transform, sub_grids });
	}

	let mut bounds = Vec::new();
	for (grid_id, grid_col) in grids.iter() {
		let body = body_views.get(&grid_col.body).unwrap();
		let grid_transform = body.transform * *grid_col.local_transform;
		for (sub_grid_id, sub_grid) in grid_col.sub_grids.iter() {
			let sub_grid_transform = grid_transform * Transform::from_translation(sub_grid.sub_grid_pos().as_vec3());
			if let Some(aabb) = sub_grid.aabb(&sub_grid_transform) {
				bounds.push(((grid_col.body, *grid_id, *sub_grid_id), aabb));
			}
		}
	}
	let bvh = BVH::new(bounds);

	collisions.0 = get_collisions(&body_views, &grids, &bvh);
}
