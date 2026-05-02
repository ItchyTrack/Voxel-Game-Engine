use std::{collections::{HashMap, HashSet}, sync::atomic::{AtomicBool, Ordering}};

use crate::player::camera::ViewFrustum;

use super::{pose::Pose, gpu::sub_grid_gpu_state::{SubGridGpuState, SubGridGpuUploadingState}, world::World};
use super::{physics_body::PhysicsBodyId, physics_solver::inertia_tensor::InertiaTensor, sparse_set::SparseSet, resource_manager::ResourceUUID, voxels::{Voxels, Voxel}};
use glam::{I16Vec3, IVec3, Quat, Vec3, I64Vec3};

#[derive(Copy, Clone, Hash, PartialEq, Eq, Debug)]
pub struct SubGridId(pub u32);
pub struct SubGridVersionId(pub u64);

pub struct SubGrid {
	voxels: Voxels,
	sub_grid_pos: IVec3,
	version_id: SubGridVersionId,
	reupload_gpu_grid: AtomicBool,
	sub_grid_gpu_state: SubGridGpuState,
}

impl SubGrid {
	pub fn new(sub_grid_pos: IVec3) -> Self {
		Self {
			voxels: Voxels::new(),
			sub_grid_pos,
			version_id: SubGridVersionId(0),
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

	// pub fn set_updated_gpu_grid_tree(&self, tree_id: u32, voxel_id: u32, lod: f32) {
		// self.gpu_grid_tree_id.set(Some((tree_id, voxel_id, lod)));
	// }
	pub fn gpu_state(&self) -> &SubGridGpuState {
		&self.sub_grid_gpu_state
	}
	pub fn gpu_state_mut(&mut self) -> &mut SubGridGpuState {
		&mut self.sub_grid_gpu_state
	}
	pub fn reupload_gpu_grid(&self) -> bool {
		self.reupload_gpu_grid.load(Ordering::Acquire)
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

#[derive(Copy, Clone, Hash, PartialEq, Eq, Debug)]
pub struct GridId(pub u32);

pub struct Grid {
	pose: Pose,
	global_pose: Option<Pose>,
	physics_body_id: PhysicsBodyId,
	sub_grids: SparseSet<SubGridId, SubGrid>,
	next_sub_grid_id: SubGridId,
	position_mapping: HashMap<IVec3, SubGridId>,
	resource_uuid: ResourceUUID,
	mass: u64,
	voxel_center_of_mass_times_mass: I64Vec3,
	inertia_tensor_at_zero: InertiaTensor,
}

impl Grid {
	const CHUNK_SIZE: i32 = 64;

	pub fn new(physics_body_id: PhysicsBodyId, pose: &Pose, resource_uuid: ResourceUUID) -> Self {
		Self {
			pose: *pose,
			global_pose: None,
			physics_body_id: physics_body_id,
			sub_grids: SparseSet::new(),
			next_sub_grid_id: SubGridId(0),
			position_mapping: HashMap::new(),
			resource_uuid: resource_uuid,
			mass: 0,
			voxel_center_of_mass_times_mass: I64Vec3::ZERO,
			inertia_tensor_at_zero: InertiaTensor::ZERO,
		}
	}

	// info
	pub fn pose(&self) -> &Pose {
		&self.pose
	}
	pub fn physics_body_id(&self) -> PhysicsBodyId {
		self.physics_body_id
	}

	// sub grids
	fn add_sub_grid(&mut self, sub_grid_pos: &IVec3) -> SubGridId {
		let sub_grid_id = self.next_sub_grid_id;
		self.next_sub_grid_id.0 += 1;
		self.sub_grids.insert(sub_grid_id, SubGrid::new(*sub_grid_pos));
		sub_grid_id
	}
	fn remove_sub_grid(&mut self, sub_grid_id: SubGridId) -> bool {
		self.sub_grids.remove(&sub_grid_id).is_some()
	}
	fn map_position_to_sub_grid_id(&self, pos: &IVec3) -> Option<SubGridId> {
		self.position_mapping.get(&(pos / Self::CHUNK_SIZE)).copied()
	}
	fn map_position_to_sub_grid_id_create(&mut self, pos: &IVec3) -> SubGridId {
		*self.position_mapping.entry(pos / Self::CHUNK_SIZE).or_insert_with(|| {
			let sub_grid_id = self.next_sub_grid_id;
			self.next_sub_grid_id.0 += 1;
			self.sub_grids.insert(sub_grid_id, SubGrid::new((pos / Self::CHUNK_SIZE) * Self::CHUNK_SIZE));
			sub_grid_id
		})
	}
	pub fn sub_grid(&self, sub_grid_id: SubGridId) -> Option<&SubGrid> {
		self.sub_grids.get(&sub_grid_id)
	}
	pub fn sub_grid_mut(&mut self, sub_grid_id: SubGridId) -> Option<&mut SubGrid> {
		self.sub_grids.get_mut(&sub_grid_id)
	}
	pub fn sub_grids(&self) -> &SparseSet<SubGridId, SubGrid> {
		&self.sub_grids
	}

	// voxels
	pub fn add_voxel(&mut self, voxel_pos: &IVec3, voxel: &Voxel) -> Option<Voxel> {
		self.mass += voxel.mass as u64;
		self.voxel_center_of_mass_times_mass += voxel.mass as i64 * voxel_pos.as_i64vec3();
		self.inertia_tensor_at_zero += InertiaTensor::get_inertia_tensor_for_cube_at_pos(voxel.mass as f64, 1.0, &(voxel_pos.as_dvec3() + 0.5));
		let sub_grid_id = self.map_position_to_sub_grid_id_create(voxel_pos);
		let sub_grid = self.sub_grids.get_mut(&sub_grid_id)?;
		let old_voxel = sub_grid.add_voxel(voxel_pos.rem_euclid(IVec3::splat( Self::CHUNK_SIZE)).as_i16vec3(), *voxel)?;
		self.mass -= old_voxel.mass as u64;
		self.voxel_center_of_mass_times_mass -= old_voxel.mass as i64 * voxel_pos.as_i64vec3();
		self.inertia_tensor_at_zero -= InertiaTensor::get_inertia_tensor_for_cube_at_pos(old_voxel.mass as f64, 1.0, &(voxel_pos.as_dvec3() + 0.5));
		Some(old_voxel)
	}
	pub fn remove_voxel(&mut self, voxel_pos: &IVec3) -> Option<Voxel> {
		let sub_grid_id = self.map_position_to_sub_grid_id(voxel_pos)?;
		let sub_grid = self.sub_grids.get_mut(&sub_grid_id)?;
		let voxel = sub_grid.remove_voxel(&voxel_pos.rem_euclid(IVec3::splat( Self::CHUNK_SIZE)).as_i16vec3())?;
		self.mass -= voxel.mass as u64;
		self.voxel_center_of_mass_times_mass -= voxel.mass as i64 * voxel_pos.as_i64vec3();
		self.inertia_tensor_at_zero -= InertiaTensor::get_inertia_tensor_for_cube_at_pos(voxel.mass as f64, 1.0, &(voxel_pos.as_dvec3() + 0.5));
		if sub_grid.get_voxels().get_voxels().len() == 0 {
			self.sub_grids.remove(&sub_grid_id);
		}
		Some(voxel)
	}
	pub fn get_voxel(&self, voxel_pos: &IVec3) -> Option<&Voxel> {
		let sub_grid_id = self.map_position_to_sub_grid_id(voxel_pos)?;
		let sub_grid = self.sub_grids.get(&sub_grid_id)?;
		sub_grid.get_voxel(&voxel_pos.rem_euclid(IVec3::splat(Self::CHUNK_SIZE)).as_i16vec3())
	}

	// physics
	pub fn mass(&self) -> f32 { self.mass as f32 }

	pub fn grid_center_of_mass(&self) -> Vec3 {
		if self.mass == 0 {
			Vec3::ZERO
		} else {
			self.voxel_center_of_mass_times_mass.as_vec3() / self.mass() + 0.5
		}
	}
	pub fn physics_body_center_of_mass(&self) -> Vec3 {
		self.grid_to_physics_body_vec(&self.grid_center_of_mass())
	}

	pub fn grid_inertia_tensor_at_zero(&self) -> &InertiaTensor {
		&self.inertia_tensor_at_zero
	}
	pub fn grid_inertia_tensor_at_centered_of_mass(&self) -> InertiaTensor {
		let com = self.voxel_center_of_mass_times_mass.as_dvec3() / self.mass as f64 + 0.5;
		self.inertia_tensor_at_zero.move_to_center_of_mass(
			&com,
			self.mass as f64
		)
	}
	pub fn inertia_tensor(&self) -> InertiaTensor {
		let com = self.voxel_center_of_mass_times_mass.as_dvec3() / self.mass as f64 + 0.5;
		self.inertia_tensor_at_zero.move_between_points(
			&com,
			&(self.pose.inverse().translation.as_dvec3() - com),
			self.mass as f64
		).get_rotated(self.pose.rotation.as_dquat())
	}

	pub fn global_pose(&self) -> &Option<Pose> {
		&self.global_pose
	}
	pub fn update_physics_body_pose(&mut self, physics_body_pose: &Pose) {
		self.global_pose = Some(physics_body_pose * self.pose);
	}

	pub fn get_inertia_tensor_at_center_of_mass(&self) -> InertiaTensor {
		self.inertia_tensor_at_zero.move_to_center_of_mass(&(self.voxel_center_of_mass_times_mass.as_dvec3() / self.mass as f64), self.mass as f64)
	}

	// reference frame
	pub fn physics_body_to_grid(&self, other: &Pose) -> Pose { self.pose.inverse() * other }
	pub fn grid_to_physics_body(&self, other: &Pose) -> Pose { self.pose * other }
	pub fn physics_body_to_grid_vec(&self, pos: &Vec3) -> Vec3 { self.pose.inverse() * pos }
	pub fn grid_to_physics_body_vec(&self, pos: &Vec3) -> Vec3 { self.pose * pos }
	pub fn physics_body_to_grid_rot(&self, rot: &Quat) -> Quat { self.pose.inverse() * rot }
	pub fn grid_to_physics_body_rot(&self, rot: &Quat) -> Quat { self.pose * rot }
}

pub struct GridManager {
	grids: SparseSet<GridId, Grid>,
	body_to_grids: SparseSet<PhysicsBodyId, HashSet<GridId>>,
	next_grid_id: GridId,
}

impl GridManager {
	pub fn new() -> Self {
		Self {
			grids: SparseSet::new(),
			body_to_grids: HashMap::new(),
			next_grid_id: GridId(0),
		}
	}
	// grids
	pub fn add_grid(&mut self, physics_body_id: PhysicsBodyId, pose: &Pose, resource_uuid: ResourceUUID) -> GridId {
		let grid_id = self.next_grid_id;
		self.next_grid_id.0 += 1;
		self.grids.insert(grid_id, Grid::new(physics_body_id, pose, resource_uuid));
		self.body_to_grids.entry(physics_body_id).or_default().insert(grid_id);
		grid_id
	}
	pub fn remove_grid(&mut self, grid_id: GridId) -> bool {
		if let Some(grid) = self.grids.remove(&grid_id) {
			let grids = &mut self.body_to_grids.get_mut(&grid.physics_body_id()).unwrap();
			if grids.len() == 1 {
				assert!(grids.contains(&grid_id));
				self.body_to_grids.remove(&grid.physics_body_id);
			} else {
				grids.remove(&grid_id);
			}
			true
		} else { false }
	}
	pub fn grid(&self, grid_id: GridId) -> Option<&Grid> {
		self.grids.get(&grid_id)
	}
	pub fn grid_mut(&mut self, grid_id: GridId) -> Option<&mut Grid> {
		self.grids.get_mut(&grid_id)
	}
	pub fn grids(&self) -> &SparseSet<GridId, Grid> {
		&self.grids
	}
	pub fn physics_body_grid_ids(&self, physics_body_id: PhysicsBodyId) -> &HashSet<GridId> {
		self.body_to_grids.get(&physics_body_id).unwrap()
	}
	pub fn update_gpu_grid_tree(
		&mut self,
		world: &World,
		id_to_hit_count: &HashMap<(PhysicsBodyId, GridId, SubGridId), u32>,
		view_frustum: &ViewFrustum,
		_camera_pose: &Pose,
	) -> HashMap<(PhysicsBodyId, GridId, SubGridId), (u32, u32, Pose)> {
		let mut mapping: HashMap<(PhysicsBodyId, GridId, SubGridId), (u32, u32, Pose)> = HashMap::new();
		for (physics_body_id, physics_body) in world.physics_bodies.read().iter() {
			for grid_id in self.physics_body_grid_ids(*physics_body_id).clone() {
				let grid = self.grid_mut(grid_id).unwrap();
				let grid_pose = *grid.pose();
				for (sub_grid_id, sub_grid) in grid.sub_grids.iter_mut() {
					let hit_count = id_to_hit_count.get(&(*physics_body_id, grid_id, *sub_grid_id)).unwrap_or(&1);
					let grid_pose = physics_body.pose * grid_pose * Pose::from_translation(sub_grid.sub_grid_pos().as_vec3());
					mapping.insert(
						(*physics_body_id, grid_id, *sub_grid_id),
						{
							let priority = 0.0;
							let lod_level = 0.0;
							let aabb = if let Some(aabb) = sub_grid.local_aabb(&grid_pose.rotation) { aabb } else { continue; };
							let radius = aabb.0.distance(aabb.1) / 2.0 + 1.0;
							let center = (aabb.0 + aabb.1) / 2.0;
							let in_view = view_frustum.compare_sphere(center, radius);
							// if !in_view { return None; }
							let priority = if in_view { priority + 5.0 } else { priority };
							let lod_level = if *hit_count > 0 { lod_level } else { lod_level + 3.0 };
							if
								sub_grid.reupload_gpu_grid() ||
								(lod_level != sub_grid.gpu_state().lod_level() && lod_level == 0.0) ||
								((sub_grid.gpu_state().lod_level() - lod_level).abs() > 0.25)
							{
								sub_grid.reupload_gpu_grid.store(false, Ordering::Release);
								sub_grid.sub_grid_gpu_state.request_gpu_state(
									world,
									SubGridGpuUploadingState{ lod_level: lod_level },
									priority,
									grid_id,
									*sub_grid_id,
									&sub_grid.voxels
								);
								if !sub_grid.gpu_state().on_gpu() { continue; }
							}
							let world_gpu_data = world.world_gpu_data.read();
							let packed_64_tree_dynamic_buffer = world_gpu_data.packed_64_tree_dynamic_buffer.read();
							let info_64_tree = packed_64_tree_dynamic_buffer.get_held_buffer(sub_grid.gpu_state().tree_id());
							let info_64_tree = if let Some(info_64_tree) = info_64_tree { info_64_tree } else { continue; };
							let packed_voxel_data_dynamic_buffer = world_gpu_data.packed_voxel_data_dynamic_buffer.read();
							let info_voxel_data = packed_voxel_data_dynamic_buffer.get_held_buffer(sub_grid.gpu_state().voxels_id());
							let info_voxel_data = if let Some(info_voxel_data) = info_voxel_data { info_voxel_data } else { continue; };
							(
								info_64_tree.offset(),
								info_voxel_data.offset(),
								grid_pose * Pose::from_translation(sub_grid.get_voxels().get_voxels().get_internals().1.as_vec3())
							)
						}
					);
				}
			}
		}
		mapping
	}
}
