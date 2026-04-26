use std::sync::atomic::{AtomicBool, Ordering};

use crate::{voxels, world::{grid::GridId, resource_manager::ResourceUUID}};
use glam::{I16Vec3, IVec3, Quat, Vec3};

#[derive(Copy, Clone, Hash, PartialEq, Eq, Debug)]
pub struct SubGridId(pub u32);
pub struct SubGridVersionId(pub u64);

pub struct SubGrid {
	voxels: voxels::Voxels,
	id: SubGridId,
	grid_id: GridId,
	uuid: ResourceUUID,
	sub_grid_pos: IVec3,
	version_id: SubGridVersionId,
	reupload_gpu_grid: AtomicBool,
}

impl SubGrid {
	pub fn new(sub_grid_uuid: ResourceUUID, sub_grid_id: SubGridId, sub_grid_pos: IVec3, grid_id: GridId) -> Self {
		Self {
			voxels: voxels::Voxels::new(),
			id: sub_grid_id,
			grid_id,
			uuid: sub_grid_uuid,
			sub_grid_pos,
			version_id: SubGridVersionId(0),
			reupload_gpu_grid: AtomicBool::new(false),
		}
	}
	pub fn id(&self) -> SubGridId {
		self.id
	}
	pub fn grid_id(&self) -> GridId {
		self.grid_id
	}
	pub fn sub_grid_pos(&self) -> IVec3 {
		self.sub_grid_pos
	}
	pub fn add_voxel(&mut self, pos: I16Vec3, voxel: voxels::Voxel) -> Option<voxels::Voxel> {
		self.reupload_gpu_grid.store(true, Ordering::Relaxed);
		self.voxels.add_voxel(pos, voxel)
	}
	pub fn remove_voxel(&mut self, pos: &I16Vec3) -> Option<voxels::Voxel> {
		self.reupload_gpu_grid.store(true, Ordering::Relaxed);
		self.voxels.remove_voxel(pos)
	}
	pub fn get_voxel(&self, pos: I16Vec3) -> Option<&voxels::Voxel> { self.voxels.get_voxel(pos) }
	pub fn get_voxels(&self) -> &voxels::Voxels { &self.voxels }

	// pub fn set_updated_gpu_grid_tree(&self, tree_id: u32, voxel_id: u32, lod: f32) {
		// self.gpu_grid_tree_id.set(Some((tree_id, voxel_id, lod)));
	// }
	pub fn reupload_gpu_grid(&self) -> bool {
		self.reupload_gpu_grid.load(Ordering::Relaxed)
	}

	// return true is this should be deleted
	pub fn clean_up(&mut self) -> bool {
		self.get_voxels().get_voxels().is_empty()
	}

	pub fn local_aabb(&self, quat: &Quat) -> Option<(Vec3, Vec3)> {
		let (min, max) = self.voxels.get_bounding_box()?;
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
		let rotated_corners = corners.map(|c| quat * c);
		Some((
			rotated_corners.iter().fold(Vec3::splat(f32::MAX), |acc, c| acc.min(*c)),
			rotated_corners.iter().fold(Vec3::splat(f32::MIN), |acc, c| acc.max(*c))
		))
	}
}
