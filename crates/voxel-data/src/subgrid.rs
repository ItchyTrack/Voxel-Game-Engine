use bevy::prelude::*;
use bevy::math::{I16Vec3, IVec3, Vec3};

use crate::aabb::aabb_of_transformed_aabb;
use crate::grid::GridId;
use crate::voxels::{Voxel, Voxels};

pub type SubGridId = Entity;

/// Handle entity for one sub-grid. The voxel data lives in the owning [`Grid`](crate::grid::Grid);
/// this is just a back-pointer so other systems can hang per-sub-grid components off a stable id.
#[derive(Debug, Component)]
pub struct SubGrid {
	grid: GridId,
	sub_grid_pos: IVec3,
}

impl SubGrid {
	pub(crate) fn new(grid: GridId, sub_grid_pos: IVec3) -> Self {
		Self { grid, sub_grid_pos }
	}

	pub fn grid(&self) -> GridId { self.grid }
	pub fn sub_grid_pos(&self) -> IVec3 { self.sub_grid_pos }
}

#[derive(Clone, Copy)]
pub struct SubGridRef<'a> {
	voxels: &'a Voxels,
	sub_grid_pos: IVec3,
}

impl<'a> SubGridRef<'a> {
	pub(crate) fn new(voxels: &'a Voxels, sub_grid_pos: IVec3) -> Self {
		Self { voxels, sub_grid_pos }
	}

	pub fn voxels(&self) -> &'a Voxels { self.voxels }
	pub fn voxel(&self, pos: &I16Vec3) -> Option<&'a Voxel> { self.voxels.voxel(pos) }
	pub fn sub_grid_pos(&self) -> IVec3 { self.sub_grid_pos }

	pub fn aabb(&self, transform: &Transform) -> Option<(Vec3, Vec3)> {
		let (min, max) = self.voxels.bounding_box()?;
		Some(aabb_from_bounds(min, max, transform))
	}
}

pub fn aabb_from_bounds(min: I16Vec3, max: I16Vec3, transform: &Transform) -> (Vec3, Vec3) {
	aabb_of_transformed_aabb(transform, min.as_vec3(), max.as_vec3() + Vec3::ONE)
}
