use std::collections::HashSet;

use crate::voxels;
use crate::pose::Pose;
use super::physics_solver::inertia_tensor::InertiaTensor;
use super::resource_manager::{ResourceManager, ResourceUUID};
use super::{subgrid::SubGridId};

use glam::{I16Vec3, I64Vec3, IVec3, Vec3, Quat};

#[derive(Copy, Clone, Hash, PartialEq, Eq, Debug)]
pub struct GridId(pub u32);

pub struct Grid {
	pub pose: Pose,
	sub_grids: HashSet<SubGridId>,
	id: GridId,
	uuid: ResourceUUID,
	mass: u64,
	voxel_center_of_mass_times_mass: I64Vec3,
	inertia_tensor_at_zero: InertiaTensor,
}

const SUB_GRID_SIZE: u16 = 64;

impl Grid {
	pub fn new(grid_uuid: ResourceUUID, grid_id: GridId, pose: &Pose) -> Self {
		Self {
			pose: *pose,
			sub_grids: vec![],
			id: grid_id,
			uuid: grid_uuid,
			mass: 0,
			voxel_center_of_mass_times_mass: I64Vec3::ZERO,
			inertia_tensor_at_zero: InertiaTensor::ZERO,
		}
	}

	fn pos_to_sub_grid_pos(&self, pos: &IVec3) -> I16Vec3{
		pos.div_euclid(IVec3::splat(SUB_GRID_SIZE as i32)).as_i16vec3()
	}

	// pub fn add_sub_grid(&mut self, uuid: ResourceUUID, pos: &IVec3) -> SubGridId {
	// 	self.sub_grid_id_to_index.insert(self.next_sub_grid_id, self.sub_grids.len() as u32);
	// 	let sub_grid_pos = self.pos_to_sub_grid_pos(pos);
	// 	self.sub_grid_pos_to_index.insert(sub_grid_pos, self.sub_grids.len() as u32);
	// 	self.sub_grids.push(SubGrid::new(uuid, self.next_sub_grid_id, sub_grid_pos));
	// 	self.next_sub_grid_id.0 += 1;
	// 	SubGridId(self.next_sub_grid_id.0 - 1)
	// }

	// pub fn remove_sub_grid(&mut self, sub_grid_id: SubGridId) {
	// 	let index = match self.sub_grid_id_to_index.remove(&sub_grid_id) {
	// 		Some(i) => i,
	// 		None => return,
	// 	};
	// 	let sub_grid = self.sub_grids.swap_remove(index as usize);
	// 	self.sub_grid_pos_to_index.remove(&sub_grid.sub_grid_pos);
	// 	if index != self.sub_grids.len() as u32 {
	// 		let other_id = self.sub_grids[index as usize].id();
	// 		self.sub_grid_id_to_index.insert(other_id, index);
	// 	}
	// }

	// pub fn remove_sub_grid_by_index(&mut self, index: u32) {
	// 	if let Some(sub_grid) = self.sub_grids.get(index as usize) {
	// 		self.sub_grid_id_to_index.remove(&sub_grid.id());
	// 	} else { return; }
	// 	self.sub_grids.swap_remove(index as usize);
	// 	if index != self.sub_grids.len() as u32 {
	// 		let other_id = self.sub_grids[index as usize].id();
	// 		self.sub_grid_id_to_index.insert(other_id, index);
	// 	}
	// }

	pub fn sub_grids(&self) -> &[SubGridId] {
		&self.sub_grids
	}

	// pub fn get_sub_grid(&self, pos: &IVec3) -> Option<(&SubGrid, I16Vec3)> {
	// 	Some((self.sub_grids.get(*self.sub_grid_pos_to_index.get(&self.pos_to_sub_grid_pos(pos))? as usize)?, pos.rem_euclid(IVec3::splat(SUB_GRID_SIZE as i32)).as_i16vec3()))
	// }

	// fn get_sub_grid_mut(&mut self, pos: &IVec3) -> Option<(&mut SubGrid, I16Vec3)> {
	// 	let sub_grid_pos = self.pos_to_sub_grid_pos(pos);
	// 	Some((self.sub_grids.get_mut(*self.sub_grid_pos_to_index.get(&sub_grid_pos)? as usize)?, pos.rem_euclid(IVec3::splat(SUB_GRID_SIZE as i32)).as_i16vec3()))
	// }

	// fn get_sub_grid_mut_create(&mut self, pos: &IVec3, resource_manager: &mut ResourceManager) -> (&mut SubGrid, I16Vec3) {
	// 	let sub_grid_pos = self.pos_to_sub_grid_pos(pos);
	// 	if let Some(sub_grid_index) = self.sub_grid_pos_to_index.get(&sub_grid_pos) {
	// 		return (self.sub_grid_by_index_mut(*sub_grid_index).unwrap(), pos.rem_euclid(IVec3::splat(SUB_GRID_SIZE as i32)).as_i16vec3());
	// 	}

	// 	let sub_grid_uuid = ResourceUUID(Uuid::new_v4());
	// 	let sub_grid_resource = resource_manager.create_resource_blank(sub_grid_uuid.clone(), ResourceInfoType::SubGrid { sub_grid_resource: SubGridResource::new(sub_grid_uuid.clone(), self.uuid.clone()) }).unwrap();
	// 	let sub_grid_id = self.add_sub_grid(sub_grid_uuid, pos);
	// 	match sub_grid_resource.info_mut() {
	// 		ResourceInfoType::SubGrid { sub_grid_resource } => sub_grid_resource.set_sub_grid_id(Some(sub_grid_id)),
	// 		_ => unreachable!()
	// 	}
	// 	(self.sub_grid_mut(sub_grid_id).unwrap(), pos.rem_euclid(IVec3::splat(SUB_GRID_SIZE as i32)).as_i16vec3())
	// }

	// pub fn sub_grid_pos_to_grid_pos(&self, sub_grid_pos: &IVec3) -> IVec3 {
	// 	sub_grid_pos * SUB_GRID_SIZE as i32
	// }

	pub fn id(&self) -> GridId {
		self.id
	}

	pub fn get_body_center_of_mass(&self) -> Vec3 {
		if self.mass == 0 {
			Vec3::ZERO
		} else {
			self.local_to_body_vec(&(self.voxel_center_of_mass_times_mass.as_vec3() / self.mass as f32 + 0.5))
		}
	}

	pub fn get_local_center_of_mass(&self) -> Vec3 {
		if self.mass == 0 {
			Vec3::ZERO
		} else {
			(self.voxel_center_of_mass_times_mass.as_vec3() / self.mass as f32) + 0.5
		}
	}

	pub fn mass(&self) -> f32 { self.mass as f32 }

	// return true is this should be deleted
	pub fn clean_up(&mut self) -> bool {
		let mut sub_grids_to_remove = vec![];
		for sub_grid in &mut self.sub_grids {
			if sub_grid.clean_up() {
				sub_grids_to_remove.push(sub_grid.id());
			}
		}
		if sub_grids_to_remove.len() == self.sub_grids.len() {
			return true;
		}
		for id in sub_grids_to_remove {
			self.remove_sub_grid(id);
		}
		return false;
	}

	pub fn add_voxel(&mut self, pos: IVec3, voxel: voxels::Voxel, resource_manager: &mut ResourceManager) {
		self.mass += voxel.mass as u64;
		self.voxel_center_of_mass_times_mass += voxel.mass as i64 * pos.as_i64vec3();
		self.inertia_tensor_at_zero += InertiaTensor::get_inertia_tensor_for_cube_at_pos(voxel.mass as f64, 1.0, &(pos.as_dvec3() + 0.5));
		let (sub_grid, sub_grid_pos) = self.get_sub_grid_mut_create(&pos, resource_manager);
		if let Some(old_voxel) = sub_grid.add_voxel(sub_grid_pos, voxel) {
			self.mass -= old_voxel.mass as u64;
			self.voxel_center_of_mass_times_mass -= old_voxel.mass as i64 * pos.as_i64vec3();
			self.inertia_tensor_at_zero -= InertiaTensor::get_inertia_tensor_for_cube_at_pos(old_voxel.mass as f64, 1.0, &(pos.as_dvec3() + 0.5));
		}
	}

	pub fn remove_voxel(&mut self, pos: &IVec3) {
		if let Some((sub_grid, sub_grid_pos)) = self.get_sub_grid_mut(&pos) {
			if let Some(voxel) = sub_grid.remove_voxel(&sub_grid_pos) {
				self.mass -= voxel.mass as u64;
				self.voxel_center_of_mass_times_mass -= voxel.mass as i64 * pos.as_i64vec3();
				self.inertia_tensor_at_zero -= InertiaTensor::get_inertia_tensor_for_cube_at_pos(voxel.mass as f64, 1.0, &(pos.as_dvec3() + 0.5));
			}
		}
	}

	pub fn get_voxel(&self, pos: IVec3) -> Option<&voxels::Voxel> {
		let (sub_grid, sub_grid_pos) = self.get_sub_grid(&pos)?;
		sub_grid.get_voxel(sub_grid_pos)
	}
	// pub fn get_voxels(&self) -> &voxels::Voxels { &self.voxels }

	pub fn body_to_local(&self, other: &Pose) -> Pose { self.pose.inverse() * other }
	pub fn local_to_body(&self, other: &Pose) -> Pose { self.pose * other }
	pub fn body_to_local_vec(&self, pos: &Vec3) -> Vec3 { self.pose.inverse() * pos }
	pub fn local_to_body_vec(&self, pos: &Vec3) -> Vec3 { self.pose * pos }
	pub fn body_to_local_rot(&self, rot: &Quat) -> Quat { self.pose.inverse() * rot }
	pub fn local_to_body_rot(&self, rot: &Quat) -> Quat { self.pose * rot }

	pub fn get_inertia_tensor_at_body(&self) -> InertiaTensor {
		let com = self.voxel_center_of_mass_times_mass.as_dvec3() / self.mass as f64 + 0.5;
		self.inertia_tensor_at_zero.move_between_points(
			&com,
			&(self.pose.inverse().translation.as_dvec3() - com),
			self.mass as f64
		).get_rotated(self.pose.rotation.as_dquat())
	}

	pub fn get_inertia_tensor_at_zero(&self) -> &InertiaTensor {
		&self.inertia_tensor_at_zero
	}

	pub fn get_inertia_tensor_at_center_of_mass(&self) -> InertiaTensor {
		self.inertia_tensor_at_zero.move_to_center_of_mass(&(self.voxel_center_of_mass_times_mass.as_dvec3() / self.mass as f64), self.mass as f64)
	}
}
