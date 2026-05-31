use bevy::prelude::*;
use bevy::math::{I16Vec3, IVec3, Vec3};

use crate::grid::GridId;
use crate::voxels::{Voxel, Voxels};

pub type SubGridId = Entity;

#[derive(Debug, Component)]
pub struct SubGrid {
	voxels: Voxels,
	grid: GridId,
	sub_grid_pos: IVec3,
}

impl SubGrid {
	pub(crate) fn new(grid: GridId, sub_grid_pos: IVec3) -> Self {
		Self {
			voxels: Voxels::new(),
			grid,
			sub_grid_pos,
		}
	}

	pub fn grid(&self) -> GridId { self.grid }
	pub fn sub_grid_pos(&self) -> IVec3 { self.sub_grid_pos }
	pub fn voxels(&self) -> &Voxels { &self.voxels }
	pub fn voxel(&self, pos: &I16Vec3) -> Option<&Voxel> { self.voxels.voxel(pos) }

	pub(crate) fn add_voxel(&mut self, pos: I16Vec3, voxel: Voxel) -> Option<Voxel> {
		self.voxels.add_voxel(pos, voxel)
	}
	pub(crate) fn remove_voxel(&mut self, pos: &I16Vec3) -> Option<Voxel> {
		self.voxels.remove_voxel(pos)
	}

	pub fn is_empty(&self) -> bool {
		self.voxels.grid_tree().len() == 0
	}

	pub fn aabb(&self, transform: &Transform) -> Option<(Vec3, Vec3)> {
		let (min, max) = self.voxels.bounding_box()?;
		let min = min.as_vec3();
		let max = max.as_vec3() + Vec3::new(1.0, 1.0, 1.0);
		let corners = [
			min,
			Vec3::new(max.x, min.y, min.z),
			Vec3::new(min.x, max.y, min.z),
			Vec3::new(min.x, min.y, max.z),
			Vec3::new(max.x, max.y, min.z),
			Vec3::new(max.x, min.y, max.z),
			Vec3::new(min.x, max.y, max.z),
			max,
		];
		let rotated_corners = corners.map(|c| *transform * c);
		Some((
			rotated_corners.iter().fold(Vec3::splat(f32::MAX), |acc, c| acc.min(*c)),
			rotated_corners.iter().fold(Vec3::splat(f32::MIN), |acc, c| acc.max(*c)),
		))
	}
}
