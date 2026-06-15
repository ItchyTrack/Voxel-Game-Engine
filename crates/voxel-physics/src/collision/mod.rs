use std::collections::HashMap;

use bevy::prelude::*;
use num::Zero;

use voxel_data::bvh::BVH;
use voxel_data::grid::Grid;
use voxel_data::subgrid::{SubGrid, SubGridId, SubGridRef};

use crate::components::{IsStatic, Mass, RigidBody, VoxelCollider};
use crate::sparse_set::SparseSet;
use crate::{FreezePhysics, GridId, PhysicsBodyId, PhysicsSet};

mod broadphase;
mod narrowphase;

pub use broadphase::get_collisions;

/// Minimal per-body view the collision pass needs (no mass/inertia).
pub struct BodyView {
	pub transform: Transform,
	pub is_static: bool,
}

pub struct GridCollider<'a> {
	pub body: PhysicsBodyId,
	pub local_transform: &'a Transform,
	pub sub_grids: Vec<(SubGridId, SubGridRef<'a>)>,
}

#[derive(Copy, Clone, Hash, PartialEq, Eq)]
pub enum CubeFeature {
	Vertex { xyz: u8 },
	Edge { vertex_vertex: u8 },
	Face { xyzs: u8 },
}

#[derive(Copy, Clone)]
pub struct HalfCollision {
	pub body_id: PhysicsBodyId,
	pub grid_id: GridId,
	pub voxel_pos: IVec3,
	pub feature: CubeFeature,
	pub collision: Vec3,
	pub local_collision: Vec3,
}

#[derive(Copy, Clone)]
pub struct Collision {
	pub part1: HalfCollision,
	pub part2: HalfCollision,
}

impl Collision {
	pub fn get_swapped(&self) -> Collision {
		Collision {
			part1: self.part2,
			part2: self.part1,
		}
	}
}

/// Contacts produced by [`detect_collisions`] and consumed by the solver.
#[derive(Resource, Default)]
pub struct Collisions(pub Vec<Collision>);

#[derive(Default)]
pub struct CollisionPlugin;

impl Plugin for CollisionPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<Collisions>()
			.add_systems(
				FixedUpdate,
				detect_collisions
					.in_set(PhysicsSet::Detect)
					.run_if(|freeze: Res<FreezePhysics>| !freeze.0),
			);
	}
}

fn detect_collisions(
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
