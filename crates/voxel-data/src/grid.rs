use std::{collections::{HashMap}, sync::atomic::{AtomicBool, Ordering}};

use bevy::{ecs::{component::Component, storage::{SparseSet, SparseSetIndex}}, transform::components::Transform};
use glam::{I16Vec3, IVec3, Quat, Vec3};

use crate::sub_grid_gpu_state::{SubGridGpuState};
use crate::voxels::voxels::{Voxels, Voxel};
use crate::transform_ext::TransformExt;

#[derive(Copy, Clone, Hash, PartialEq, Eq, Debug)]
pub struct SubGridId(pub u32);

impl SparseSetIndex for SubGridId {
	fn sparse_set_index(&self) -> usize { self.0 as usize }
	fn get_sparse_set_index(index: usize) -> Self { Self(index as u32) }
}

#[derive(Debug)]
pub struct SubGrid {
	voxels: Voxels,
	sub_grid_pos: IVec3,
	reupload_gpu_grid: AtomicBool,
	sub_grid_gpu_state: SubGridGpuState,
}

impl SubGrid {
	pub fn new(sub_grid_pos: IVec3) -> Self {
		Self {
			voxels: Voxels::new(),
			sub_grid_pos,
			reupload_gpu_grid: AtomicBool::new(false),
			sub_grid_gpu_state: SubGridGpuState::new(),
		}
	}
	pub fn sub_grid_pos(&self) -> IVec3 {
		self.sub_grid_pos
	}
	pub fn add_voxel(&mut self, pos: I16Vec3, voxel: Voxel) -> Option<Voxel> {
		self.reupload_gpu_grid.store(true, Ordering::Release);
		self.voxels.add_voxel(pos, voxel)
	}
	pub fn remove_voxel(&mut self, pos: &I16Vec3) -> Option<Voxel> {
		self.reupload_gpu_grid.store(true, Ordering::Release);
		self.voxels.remove_voxel(pos)
	}
	pub fn get_voxel(&self, pos: &I16Vec3) -> Option<&Voxel> { self.voxels.get_voxel(pos) }
	pub fn get_voxels(&self) -> &Voxels { &self.voxels }

	pub fn gpu_state(&self) -> &SubGridGpuState {
		&self.sub_grid_gpu_state
	}
	pub fn gpu_state_mut(&mut self) -> &mut SubGridGpuState {
		&mut self.sub_grid_gpu_state
	}
	pub fn reupload_gpu_grid(&self) -> bool {
		self.reupload_gpu_grid.load(Ordering::Acquire)
	}
	pub fn clear_reupload_flag(&self) {
		self.reupload_gpu_grid.store(false, Ordering::Release);
	}

	pub fn aabb(&self, transform: &Transform) -> Option<(Vec3, Vec3)> {
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
		let rotated_corners = corners.map(|c| *transform * c);
		Some((
			rotated_corners.iter().fold(Vec3::splat(f32::MAX), |acc, c| acc.min(*c)),
			rotated_corners.iter().fold(Vec3::splat(f32::MIN), |acc, c| acc.max(*c))
		))
	}
}

#[derive(Copy, Clone, Hash, PartialEq, Eq, Debug)]
pub struct GridId(pub usize);

// impl SparseSetIndex for GridId {
// 	fn sparse_set_index(&self) -> usize { self.0 as usize }
// 	fn get_sparse_set_index(index: usize) -> Self { Self(index as u32) }
// }

#[derive(Debug, Component)]
pub struct Grid {
	transform: Transform,
	global_transform: Option<Transform>,
	sub_grids: bevy::ecs::storage::SparseSet<SubGridId, SubGrid>,
	next_sub_grid_id: SubGridId,
	position_mapping: HashMap<IVec3, SubGridId>,
}

impl Grid {
	const CHUNK_SIZE: i32 = 64;

	pub fn new(transform: &Transform) -> Self {
		Self {
			transform: *transform,
			global_transform: None,
			sub_grids: SparseSet::new(),
			next_sub_grid_id: SubGridId(0),
			position_mapping: HashMap::new(),
		}
	}

	pub fn transform(&self) -> &Transform {
		&self.transform
	}

	// sub grids
	fn add_sub_grid(&mut self, sub_grid_pos: &IVec3) -> SubGridId {
		let sub_grid_id = self.next_sub_grid_id;
		self.next_sub_grid_id.0 += 1;
		self.sub_grids.insert(sub_grid_id, SubGrid::new(*sub_grid_pos));
		sub_grid_id
	}
	fn remove_sub_grid(&mut self, sub_grid_id: SubGridId) -> bool {
		self.sub_grids.remove(sub_grid_id).is_some()
	}
	fn map_position_to_sub_grid_id(&self, pos: &IVec3) -> Option<SubGridId> {
		self.position_mapping.get(&(pos.div_euclid(IVec3::splat(Self::CHUNK_SIZE)))).copied()
	}
	fn map_position_to_sub_grid_id_create(&mut self, pos: &IVec3) -> SubGridId {
		*self.position_mapping.entry(pos.div_euclid(IVec3::splat(Self::CHUNK_SIZE))).or_insert_with(|| {
			let sub_grid_id = self.next_sub_grid_id;
			self.next_sub_grid_id.0 += 1;
			self.sub_grids.insert(sub_grid_id, SubGrid::new((pos.div_euclid(IVec3::splat(Self::CHUNK_SIZE))) * Self::CHUNK_SIZE));
			sub_grid_id
		})
	}
	pub fn sub_grid(&self, sub_grid_id: SubGridId) -> Option<&SubGrid> {
		self.sub_grids.get(sub_grid_id)
	}
	pub fn sub_grid_mut(&mut self, sub_grid_id: SubGridId) -> Option<&mut SubGrid> {
		self.sub_grids.get_mut(sub_grid_id)
	}
	pub fn sub_grids(&self) -> &SparseSet<SubGridId, SubGrid> {
		&self.sub_grids
	}

	// voxels
	pub fn add_voxel(&mut self, voxel_pos: &IVec3, voxel: &Voxel) -> Option<Voxel> {
		let sub_grid_id = self.map_position_to_sub_grid_id_create(voxel_pos);
		let sub_grid = self.sub_grids.get_mut(sub_grid_id)?;
		let old_voxel = sub_grid.add_voxel(voxel_pos.rem_euclid(IVec3::splat(Self::CHUNK_SIZE)).as_i16vec3(), *voxel)?;
		Some(old_voxel)
	}
	pub fn remove_voxel(&mut self, voxel_pos: &IVec3) -> Option<Voxel> {
		let sub_grid_id = self.map_position_to_sub_grid_id(voxel_pos)?;
		let sub_grid = self.sub_grids.get_mut(sub_grid_id)?;
		let voxel = sub_grid.remove_voxel(&voxel_pos.rem_euclid(IVec3::splat(Self::CHUNK_SIZE)).as_i16vec3())?;
		if sub_grid.get_voxels().get_voxels().len() == 0 {
			self.sub_grids.remove(sub_grid_id);
		}
		Some(voxel)
	}
	pub fn get_voxel(&self, voxel_pos: &IVec3) -> Option<&Voxel> {
		let sub_grid_id = self.map_position_to_sub_grid_id(voxel_pos)?;
		let sub_grid = self.sub_grids.get(sub_grid_id)?;
		sub_grid.get_voxel(&voxel_pos.rem_euclid(IVec3::splat(Self::CHUNK_SIZE)).as_i16vec3())
	}

	pub fn global_transform(&self) -> &Option<Transform> {
		&self.global_transform
	}
	pub fn update_physics_body_transform(&mut self, physics_body_transform: &Transform) {
		self.global_transform = Some(*physics_body_transform * self.transform);
	}

	// reference frame
	pub fn physics_body_to_grid(&self, other: &Transform) -> Transform { self.transform.inverse() * *other }
	pub fn grid_to_physics_body(&self, other: &Transform) -> Transform { self.transform * *other }
	// pub fn physics_body_to_grid_vec(&self, pos: &Vec3) -> Vec3 { self.transform.inverse() * *pos }
	pub fn grid_to_physics_body_vec(&self, pos: &Vec3) -> Vec3 { self.transform * *pos }
	pub fn physics_body_to_grid_rot(&self, rot: &Quat) -> Quat { self.transform.rotation.inverse() * *rot }
	pub fn grid_to_physics_body_rot(&self, rot: &Quat) -> Quat { self.transform.rotation * *rot }
}
