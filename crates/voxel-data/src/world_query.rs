use bevy::ecs::query::QueryFilter;
use bevy::ecs::system::SystemParam;
use bevy::math::{IVec3, Quat, Vec3};
use bevy::prelude::{Entity, GlobalTransform, Query, Transform};

use crate::bvh::BVH;
use crate::grid::{Grid, GridId};
use crate::subgrid::{SubGrid, SubGridId};

pub type VoxelAabb = (Vec3, Vec3);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct VoxelWorldKey {
	pub grid: GridId,
	pub subgrid: SubGridId,
}

pub struct VoxelWorldQuery {
	bvh: BVH<VoxelWorldKey>,
}

#[derive(Debug, Clone, Copy)]
pub struct VoxelWorldAreaHit {
	pub grid: GridId,
	pub subgrid: SubGridId,
	pub subgrid_pos: IVec3,
	pub bounds: VoxelAabb,
}

#[derive(Debug, Clone, Copy)]
pub struct VoxelWorldRaycastHit {
	pub grid: GridId,
	pub subgrid: SubGridId,
	/// Grid-local voxel position.
	pub voxel_pos: IVec3,
	/// Grid-local face normal.
	pub normal: IVec3,
	pub world_position: Vec3,
	/// World-space ray distance.
	pub distance: f32,
}

#[derive(SystemParam)]
pub struct VoxelWorldQueryParam<'w, 's, GridFilter = (), SubGridFilter = ()>
where
	GridFilter: QueryFilter + 'static,
	SubGridFilter: QueryFilter + 'static,
{
	pub grids: Query<'w, 's, (Entity, &'static GlobalTransform, &'static Grid), GridFilter>,
	pub subgrids: Query<'w, 's, (Entity, &'static SubGrid), SubGridFilter>,
}

impl VoxelWorldQuery {
	pub fn new(items: Vec<(VoxelWorldKey, VoxelAabb)>) -> Self {
		Self { bvh: BVH::new(items) }
	}

	pub fn from_queries<GridFilter, SubGridFilter>(
		grids: &Query<(Entity, &GlobalTransform, &Grid), GridFilter>,
		subgrids: &Query<(Entity, &SubGrid), SubGridFilter>,
	) -> Self
	where
		GridFilter: QueryFilter,
		SubGridFilter: QueryFilter,
	{
		let mut items = Vec::new();
		for (subgrid_entity, subgrid) in subgrids.iter() {
			let Ok((_grid_entity, grid_transform, grid)) = grids.get(subgrid.grid()) else { continue };
			let Some(view) = grid.view(subgrid) else { continue };
			let subgrid_world = grid_transform.compute_transform() * Transform::from_translation(subgrid.sub_grid_pos().as_vec3());
			let Some(bounds) = view.aabb(&subgrid_world) else { continue };
			items.push((VoxelWorldKey { grid: subgrid.grid(), subgrid: subgrid_entity }, bounds));
		}
		Self::new(items)
	}

	pub fn subgrids_in_aabb<GridFilter, SubGridFilter>(
		&self,
		grids: &Query<(Entity, &GlobalTransform, &Grid), GridFilter>,
		subgrids: &Query<(Entity, &SubGrid), SubGridFilter>,
		bounds: VoxelAabb,
	) -> Vec<VoxelWorldAreaHit>
	where
		GridFilter: QueryFilter,
		SubGridFilter: QueryFilter,
	{
		self.bvh
			.collisions(&bounds)
			.into_iter()
			.filter_map(|key| self.resolve_area_hit(grids, subgrids, key))
			.collect()
	}

	pub fn raycast<GridFilter, SubGridFilter>(
		&self,
		grids: &Query<(Entity, &GlobalTransform, &Grid), GridFilter>,
		subgrids: &Query<(Entity, &SubGrid), SubGridFilter>,
		origin: Vec3,
		dir: Vec3,
		max_distance: Option<f32>,
	) -> Option<VoxelWorldRaycastHit>
	where
		GridFilter: QueryFilter,
		SubGridFilter: QueryFilter,
	{
		let dir = dir.normalize_or_zero();
		if dir == Vec3::ZERO {
			return None;
		}

		let transform = Transform { translation: origin, rotation: Quat::from_rotation_arc(Vec3::Z, dir), scale: Vec3::ONE };
		let mut best: Option<VoxelWorldRaycastHit> = None;

		for (key, aabb_distance) in self.bvh.raycast(&transform, max_distance) {
			if best.is_some_and(|hit| aabb_distance > hit.distance) {
				break;
			}
			let Some(hit) = self.raycast_key(grids, subgrids, key, origin, dir, max_distance) else { continue };
			if best.is_none_or(|best| hit.distance < best.distance) {
				best = Some(hit);
			}
		}

		best
	}

	fn resolve_area_hit<GridFilter, SubGridFilter>(
		&self,
		grids: &Query<(Entity, &GlobalTransform, &Grid), GridFilter>,
		subgrids: &Query<(Entity, &SubGrid), SubGridFilter>,
		key: VoxelWorldKey,
	) -> Option<VoxelWorldAreaHit>
	where
		GridFilter: QueryFilter,
		SubGridFilter: QueryFilter,
	{
		let Ok((_subgrid_entity, subgrid)) = subgrids.get(key.subgrid) else { return None };
		let Ok((_grid_entity, grid_transform, grid)) = grids.get(key.grid) else { return None };
		let view = grid.view(subgrid)?;
		let subgrid_world = grid_transform.compute_transform() * Transform::from_translation(subgrid.sub_grid_pos().as_vec3());
		let bounds = view.aabb(&subgrid_world)?;
		Some(VoxelWorldAreaHit { grid: key.grid, subgrid: key.subgrid, subgrid_pos: subgrid.sub_grid_pos(), bounds })
	}

	fn raycast_key<GridFilter, SubGridFilter>(
		&self,
		grids: &Query<(Entity, &GlobalTransform, &Grid), GridFilter>,
		subgrids: &Query<(Entity, &SubGrid), SubGridFilter>,
		key: VoxelWorldKey,
		origin: Vec3,
		dir: Vec3,
		max_distance: Option<f32>,
	) -> Option<VoxelWorldRaycastHit>
	where
		GridFilter: QueryFilter,
		SubGridFilter: QueryFilter,
	{
		let Ok((_subgrid_entity, subgrid)) = subgrids.get(key.subgrid) else { return None };
		let Ok((_grid_entity, grid_transform, grid)) = grids.get(key.grid) else { return None };
		let view = grid.view(subgrid)?;

		let grid_transform = grid_transform.compute_transform();
		let inv_grid = grid_transform.to_matrix().inverse();
		let grid_origin = inv_grid.transform_point3(origin);
		let grid_dir = inv_grid.transform_vector3(dir).normalize_or_zero();
		if grid_dir == Vec3::ZERO {
			return None;
		}

		let subgrid_origin = grid_origin - subgrid.sub_grid_pos().as_vec3();
		let hit = view.raycast(subgrid_origin, grid_dir, None)?;
		let grid_hit_position = grid_origin + grid_dir * hit.distance;
		let world_position = grid_transform.transform_point(grid_hit_position);
		let distance = (world_position - origin).dot(dir);
		if distance < 0.0 || max_distance.is_some_and(|max| distance > max) {
			return None;
		}

		Some(VoxelWorldRaycastHit {
			grid: key.grid,
			subgrid: key.subgrid,
			voxel_pos: subgrid.sub_grid_pos() + hit.voxel_pos.as_ivec3(),
			normal: hit.normal.as_ivec3(),
			world_position,
			distance,
		})
	}
}

impl<'w, 's, GridFilter, SubGridFilter> VoxelWorldQueryParam<'w, 's, GridFilter, SubGridFilter>
where
	GridFilter: QueryFilter + 'static,
	SubGridFilter: QueryFilter + 'static,
{
	pub fn build(&self) -> VoxelWorldQuery {
		VoxelWorldQuery::from_queries(&self.grids, &self.subgrids)
	}

	pub fn subgrids_in_aabb(&self, bounds: VoxelAabb) -> Vec<VoxelWorldAreaHit> {
		self.build().subgrids_in_aabb(&self.grids, &self.subgrids, bounds)
	}

	pub fn raycast(&self, origin: Vec3, dir: Vec3, max_distance: Option<f32>) -> Option<VoxelWorldRaycastHit> {
		self.build().raycast(&self.grids, &self.subgrids, origin, dir, max_distance)
	}
}
