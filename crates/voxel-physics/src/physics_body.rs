use glam::{Vec3, Quat};

use crate::world::physics_solver::inertia_tensor::InertiaTensor;

use super::resource_manager::ResourceUUID;
use super::{grid::GridId, pose::Pose};

#[derive(Copy, Clone, Hash, PartialEq, Eq, Debug)]
pub struct PhysicsBodyId(pub u32);

pub struct PhysicsBody {
	pub pose: Pose,
	pub velocity: Vec3,
	pub angular_velocity: Vec3,
	pub is_static: bool,
	grids: Vec<GridId>,
	id: PhysicsBodyId,
	uuid: ResourceUUID,
	pub mass: f32,
	pub center_of_mass: Vec3,
	pub rotational_inertia: InertiaTensor,
}

impl PhysicsBody {
	pub fn new(body_uuid: ResourceUUID, id: PhysicsBodyId) -> Self {
		Self {
			pose: Pose::ZERO,
			velocity: Vec3::ZERO,
			angular_velocity: Vec3::ZERO,
			is_static: false,
			grids: vec![],
			id: id,
			uuid: body_uuid,
			mass: 0.0,
			center_of_mass: Vec3::ZERO,
			rotational_inertia: InertiaTensor::ZERO,
		}
	}
	pub fn id(&self) -> PhysicsBodyId {
		self.id
	}
	pub fn uuid(&self) -> &ResourceUUID {
		&self.uuid
	}
	pub fn mass(&self) -> f32 {
		self.mass
	}
	pub fn local_center_of_mass(&self) -> Vec3 {
		self.center_of_mass
	}
	pub fn global_rotated_center_of_mass(&self) -> Vec3 {
		self.pose.rotation * self.center_of_mass
	}
	pub fn global_center_of_mass(&self) -> Vec3 {
		self.pose * self.center_of_mass
	}
	pub fn rotational_inertia(&self) -> InertiaTensor {
		self.rotational_inertia.get_rotated(self.pose.rotation.as_dquat())
	}
	pub fn global_rotational_inertia(&self) -> InertiaTensor {
		self.rotational_inertia.get_rotated(self.pose.rotation.as_dquat())
	}

	// return true is this should be deleted
	// pub fn clean_up(&mut self) -> bool {
	// 	let mut index = 0u32;
	// 	while index < self.grids.len() as u32 {
	// 		if self.grids[index as usize].clean_up() {
	// 			self.remove_grid_by_index(index);
	// 		} else {
	// 			index += 1;
	// 		}
	// 	}
	// 	return self.grids.is_empty();
	// }

	// pub fn add_grid(&mut self, uuid: ResourceUUID, grid_pose: Pose) -> GridId {
	// 	self.grid_id_to_index.insert(self.next_grid_id, self.grids.len() as u32);
	// 	self.grids.push(Grid::new(uuid, self.next_grid_id, &grid_pose));
	// 	self.next_grid_id.0 += 1;
	// 	GridId(self.next_grid_id.0 - 1)
	// }

	// pub fn remove_grid(&mut self, grid_id: GridId) {
	// 	let index = match self.grid_id_to_index.remove(&grid_id) {
	// 		Some(i) => i,
	// 		None => return,
	// 	};
	// 	self.grids.swap_remove(index as usize);
	// 	if index != self.grids.len() as u32 {
	// 		let other_id = self.grids[index as usize].id();
	// 		self.grid_id_to_index.insert(other_id, index);
	// 	}
	// }

	// pub fn remove_grid_by_index(&mut self, index: u32) {
	// 	if let Some(grid) = self.grids.get(index as usize) {
	// 		self.grid_id_to_index.remove(&grid.id());
	// 	} else { return; }
	// 	self.grids.swap_remove(index as usize);
	// 	if index != self.grids.len() as u32 {
	// 		let other_id = self.grids[index as usize].id();
	// 		self.grid_id_to_index.insert(other_id, index);
	// 	}
	// }

	// pub fn grid(&self, grid_id: GridId) -> Option<&Grid> {
	// 	let index = *self.grid_id_to_index.get(&grid_id)?;
	// 	self.grids.get(index as usize)
	// }

	// pub fn grid_by_index(&self, grid_index: u32) -> Option<&Grid> {
	// 	self.grids.get(grid_index as usize)
	// }

	// pub fn grid_mut(&mut self, grid_id: GridId) -> Option<&mut Grid> {
	// 	let index = *self.grid_id_to_index.get(&grid_id)?;
	// 	self.grids.get_mut(index as usize)
	// }

	// pub fn grid_by_index_mut(&mut self, grid_index: u32) -> Option<&mut Grid> {
	// 	self.grids.get_mut(grid_index as usize)

	pub fn render_debug_inertia_box(&self) {
		self.global_rotational_inertia().render_debug_box(self.mass(), self.global_center_of_mass());
	}

	pub fn world_to_local(&self, other: &Pose) -> Pose { self.pose.inverse() * other }
	pub fn local_to_world(&self, other: &Pose) -> Pose { self.pose * other }
	pub fn world_to_local_vec(&self, vec: &Vec3) -> Vec3 { self.pose.inverse() * vec }
	pub fn local_to_world_vec(&self, vec: &Vec3) -> Vec3 { self.pose * vec }
	pub fn world_to_local_rot(&self, rot: &Quat) -> Quat { self.pose.inverse() * rot }
	pub fn local_to_world_rot(&self, rot: &Quat) -> Quat { self.pose * rot }
}
