use crate::{world::resource_manager::ResourceUUID, voxels};
use glam::{I16Vec3, IVec3};

#[derive(Copy, Clone, Hash, PartialEq, Eq, Debug)]
pub struct SubGridId(pub u32);

pub struct SubGrid {
	voxels: voxels::Voxels,
	id: SubGridId,
	uuid: ResourceUUID,
	sub_grid_pos: I16Vec3,
}

impl SubGrid {
	pub fn new(sub_grid_uuid: ResourceUUID, sub_grid_id: SubGridId, sub_grid_pos: I16Vec3) -> Self {
		Self {
			voxels: voxels::Voxels::new(),
			id: sub_grid_id,
			uuid: sub_grid_uuid,
			sub_grid_pos,
		}
	}
	pub fn id(&self) -> SubGridId {
		self.id
	}
	pub fn sub_grid_pos(&self) -> I16Vec3 {
		self.sub_grid_pos
	}
	pub fn sub_grid_ipos(&self) -> IVec3 {
		self.sub_grid_pos.as_ivec3()
	}
	pub fn add_voxel(&mut self, pos: I16Vec3, voxel: voxels::Voxel) -> Option<voxels::Voxel> {
		// self.reupload_gpu_grid.set(true);
		self.voxels.add_voxel(pos, voxel)
	}
	pub fn remove_voxel(&mut self, pos: &I16Vec3) -> Option<voxels::Voxel> {
		// self.reupload_gpu_grid.set(true);
		self.voxels.remove_voxel(pos)
	}
	pub fn get_voxel(&self, pos: I16Vec3) -> Option<&voxels::Voxel> { self.voxels.get_voxel(pos) }
	pub fn get_voxels(&self) -> &voxels::Voxels { &self.voxels }

	// pub fn get_voxels(&self) -> &voxels::Voxels { &self.voxels }

	// pub fn set_updated_gpu_grid_tree(&self, tree_id: u32, voxel_id: u32, lod: f32) {
	// 	self.gpu_grid_tree_id.set(Some((tree_id, voxel_id, lod)));
	// }

	// return true is this should be deleted
	pub fn clean_up(&mut self) -> bool {
		self.get_voxels().get_voxels().is_empty()
	}
}
