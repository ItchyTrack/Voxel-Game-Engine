use std::{cell::Cell, collections::HashMap};

use glam::{I16Vec3, I64Vec3, IVec3, Quat, Vec3};
use tracy_client::span;
use crate::{gpu_objects::{gpu_grid_tree, packed_dynamic_buffer::PackedDynamicBuffer}, player::camera, pose::Pose, voxels};

use super::inertia_tensor::InertiaTensor;

pub struct SubGrid {
	voxels: voxels::Voxels,
	reupload_gpu_grid: Cell<bool>,
	gpu_grid_tree_id: Cell<Option<(u32, u32, f32)>>, // tree id, tree id, lod it was taken at
}

impl SubGrid {
	pub fn new() -> Self {
		Self {
			voxels: voxels::Voxels::new(),
			reupload_gpu_grid: Cell::new(false),
			gpu_grid_tree_id: Cell::new(None),
		}
	}
	pub fn add_voxel(&mut self, pos: I16Vec3, voxel: voxels::Voxel) -> Option<voxels::Voxel> {
		self.reupload_gpu_grid.set(true);
		self.voxels.add_voxel(pos, voxel)
	}
	pub fn remove_voxel(&mut self, pos: &I16Vec3) -> Option<voxels::Voxel> {
		self.reupload_gpu_grid.set(true);
		self.voxels.remove_voxel(pos)
	}
	pub fn get_voxel(&self, pos: I16Vec3) -> Option<&voxels::Voxel> { self.voxels.get_voxel(pos) }
	pub fn get_voxels(&self) -> &voxels::Voxels { &self.voxels }

	// pub fn get_voxels(&self) -> &voxels::Voxels { &self.voxels }

	pub fn update_gpu_grid_tree(
		&self,
		device: &wgpu::Device,
		queue: &wgpu::Queue,
		packed_64_tree_dynamic_buffer: &mut PackedDynamicBuffer,
		packed_voxel_data_dynamic_buffer: &mut PackedDynamicBuffer,
		view_frustum: &camera::ViewFrustum,
		lod_level: f32,
		pose: Pose
	) -> Option<(u32, u32, Pose)> {
		let aabb = self.voxels.get_bounding_box()?;
		let aabb = (pose * aabb.0.as_vec3(), pose * aabb.1.as_vec3());
		let radius = aabb.0.distance(aabb.1) / 2.0;
		let center = (aabb.0 + aabb.1) / 2.0;
		let in_view = view_frustum.compare_sphere(center, radius);
		let gpu_grid_tree_id_val = self.gpu_grid_tree_id.get();
		if in_view {
			if let Some((grid_id, voxel_id, old_lod_level)) = gpu_grid_tree_id_val {
				if self.reupload_gpu_grid.get() || lod_level != old_lod_level && (lod_level == 0.0 || (old_lod_level - lod_level).abs() > 0.25) {
					let _zone = span!("Recreate GPU grid tree");
					self.gpu_grid_tree_id.set({
						let (tree_buffer, voxel_buffer) = gpu_grid_tree::make_gpu_grid_tree(self.voxels.get_voxels(), self.voxels.get_palette(), lod_level);
						match packed_64_tree_dynamic_buffer.replace_buffer(device, queue, grid_id, &tree_buffer) {
							Ok(gpu_grid_tree_id) => {
								match packed_voxel_data_dynamic_buffer.replace_buffer(device, queue, voxel_id, &voxel_buffer) {
									Ok(gpu_voxel_data_id) => {
										self.reupload_gpu_grid.set(false);
										tracy_client::plot!("64 tree bytes", packed_64_tree_dynamic_buffer.held_bytes() as f64);
										tracy_client::plot!("voxel data bytes", packed_voxel_data_dynamic_buffer.held_bytes() as f64);
										Some((gpu_grid_tree_id, gpu_voxel_data_id, lod_level))
									},
									Err(err) => {
										println!("{}", err);
										if let Err(err) = packed_64_tree_dynamic_buffer.remove_buffer(gpu_grid_tree_id) {
											println!("{}", err);
										}
										self.gpu_grid_tree_id.set(None);
										tracy_client::plot!("64 tree bytes", packed_64_tree_dynamic_buffer.held_bytes() as f64);
										tracy_client::plot!("voxel data bytes", packed_voxel_data_dynamic_buffer.held_bytes() as f64);
										return None;
									},
								}
							},
							Err(err) => {
								println!("{}", err);
								if let Err(err) = packed_voxel_data_dynamic_buffer.remove_buffer(voxel_id) {
									println!("{}", err);
								}
								self.gpu_grid_tree_id.set(None);
								tracy_client::plot!("64 tree bytes", packed_64_tree_dynamic_buffer.held_bytes() as f64);
								tracy_client::plot!("voxel data bytes", packed_voxel_data_dynamic_buffer.held_bytes() as f64);
								return None;
							},
						}
					});
					return Some((
						packed_64_tree_dynamic_buffer.get_held_buffer(self.gpu_grid_tree_id.get()?.0)?.offset(),
						packed_voxel_data_dynamic_buffer.get_held_buffer(self.gpu_grid_tree_id.get()?.1)?.offset(),
						pose * Pose::from_translation(self.voxels.get_voxels().get_internals().1.as_vec3()
					)));
				}
				return Some((
					packed_64_tree_dynamic_buffer.get_held_buffer(grid_id)?.offset(),
					packed_voxel_data_dynamic_buffer.get_held_buffer(voxel_id)?.offset(),
					pose * Pose::from_translation(self.voxels.get_voxels().get_internals().1.as_vec3())
				));
			}
			let _zone = span!("Create GPU grid tree");
			self.gpu_grid_tree_id.set({
				let (tree_buffer, voxel_buffer) = gpu_grid_tree::make_gpu_grid_tree(self.voxels.get_voxels(), self.voxels.get_palette(), lod_level);
				match packed_64_tree_dynamic_buffer.add_buffer(device, queue, &tree_buffer) {
					Ok(gpu_grid_tree_id) => {
						match packed_voxel_data_dynamic_buffer.add_buffer(device, queue, &voxel_buffer) {
							Ok(gpu_voxel_data_id) => {
								self.reupload_gpu_grid.set(false);
								tracy_client::plot!("64 tree bytes", packed_64_tree_dynamic_buffer.held_bytes() as f64);
								tracy_client::plot!("voxel data bytes", packed_voxel_data_dynamic_buffer.held_bytes() as f64);
								Some((gpu_grid_tree_id, gpu_voxel_data_id, lod_level))
							},
							Err(err) => {
								println!("{}", err);
								if let Err(err) = packed_64_tree_dynamic_buffer.remove_buffer(gpu_grid_tree_id) {
									println!("{}", err);
								}
								self.gpu_grid_tree_id.set(None);
								tracy_client::plot!("64 tree bytes", packed_64_tree_dynamic_buffer.held_bytes() as f64);
								tracy_client::plot!("voxel data bytes", packed_voxel_data_dynamic_buffer.held_bytes() as f64);
								return None;
							},
						}
					},
					Err(err) => {
						println!("{}", err);
						self.gpu_grid_tree_id.set(None);
						tracy_client::plot!("64 tree bytes", packed_64_tree_dynamic_buffer.held_bytes() as f64);
						tracy_client::plot!("voxel data bytes", packed_voxel_data_dynamic_buffer.held_bytes() as f64);
						return None;
					},
				}
			});
			Some((
				packed_64_tree_dynamic_buffer.get_held_buffer(self.gpu_grid_tree_id.get()?.0)?.offset(),
				packed_voxel_data_dynamic_buffer.get_held_buffer(self.gpu_grid_tree_id.get()?.1)?.offset(),
				pose * Pose::from_translation(self.voxels.get_voxels().get_internals().1.as_vec3()
			)))
		} else {
			if let Some((grid_id, voxel_id, _)) = gpu_grid_tree_id_val {
				self.gpu_grid_tree_id.set(None);
				if let Err(err) = packed_64_tree_dynamic_buffer.remove_buffer(grid_id) {
					println!("{}", err);
				}
				if let Err(err) = packed_voxel_data_dynamic_buffer.remove_buffer(voxel_id) {
					println!("{}", err);
				}
				tracy_client::plot!("64 tree bytes", packed_64_tree_dynamic_buffer.held_bytes() as f64);
				tracy_client::plot!("voxel data bytes", packed_voxel_data_dynamic_buffer.held_bytes() as f64);
			}
			None
		}
	}

	// return true is this should be deleted
	pub fn clean_up(&mut self) -> bool {
		self.get_voxels().get_voxels().is_empty()
	}
}

pub struct PhysicsBodyGrid {
	pub pose: Pose,
	sub_grids: HashMap<IVec3, SubGrid>,
	mass: u64,
	voxel_center_of_mass_times_mass: I64Vec3,
	inertia_tensor_at_zero: InertiaTensor,
	id: u32,
}

const SUB_GRID_SIZE: u16 = 64;

impl PhysicsBodyGrid {
	pub fn new(grid_id: u32, pose: &Pose) -> Self {
		Self {
			pose: *pose,
			sub_grids: HashMap::new(),
			mass: 0,
			voxel_center_of_mass_times_mass: I64Vec3::ZERO,
			inertia_tensor_at_zero: InertiaTensor::ZERO,
			id: grid_id,
		}
	}

	pub fn get_sub_grids(&self) -> &HashMap<IVec3, SubGrid> {
		&self.sub_grids
	}

	pub fn get_sub_grid(&self, pos: &IVec3) -> Option<(&SubGrid, I16Vec3)> {
		Some((self.sub_grids.get(&(pos.div_euclid(IVec3::splat(SUB_GRID_SIZE as i32))))?, pos.rem_euclid(IVec3::splat(SUB_GRID_SIZE as i32)).as_i16vec3()))
	}

	pub fn get_sub_grid_from_sub_grid_pos(&self, sub_grid_pos: &IVec3) -> Option<&SubGrid> {
		Some(self.sub_grids.get(&sub_grid_pos)?)
	}

	fn get_sub_grid_mut(&mut self, pos: &IVec3) -> Option<(&mut SubGrid, I16Vec3)> {
		Some((self.sub_grids.get_mut(&(pos.div_euclid(IVec3::splat(SUB_GRID_SIZE as i32))))?, pos.rem_euclid(IVec3::splat(SUB_GRID_SIZE as i32)).as_i16vec3()))
	}

	fn get_sub_grid_mut_create(&mut self, pos: &IVec3) -> (&mut SubGrid, I16Vec3) {
		(self.sub_grids.entry(pos.div_euclid(IVec3::splat(SUB_GRID_SIZE as i32))).or_insert_with(|| SubGrid::new()), pos.rem_euclid(IVec3::splat(SUB_GRID_SIZE as i32)).as_i16vec3())
	}

	pub fn sub_grid_pos_to_grid_pos(&self, sub_grid_pos: &IVec3) -> IVec3 {
		sub_grid_pos * SUB_GRID_SIZE as i32
	}

	pub fn id(&self) -> u32 {
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

	pub fn update_gpu_grid_tree(
		&self,
		device: &wgpu::Device,
		queue: &wgpu::Queue,
		packed_64_tree_dynamic_buffer: &mut PackedDynamicBuffer,
		packed_voxel_data_dynamic_buffer: &mut PackedDynamicBuffer,
		view_frustum: &camera::ViewFrustum,
		camera_pose: Pose,
		pose: &Pose
	) -> Vec<(IVec3, (u32, u32, Pose))> {
		self.sub_grids.iter().filter_map(|(sub_grid_pos, sub_grid)| {
			let grid_pose = pose * self.pose * Pose::from_translation(self.sub_grid_pos_to_grid_pos(sub_grid_pos).as_vec3());
			Some((
				*sub_grid_pos,
				(sub_grid.update_gpu_grid_tree(
					device,
					queue,
					packed_64_tree_dynamic_buffer,
					packed_voxel_data_dynamic_buffer,
					view_frustum,
					f32::max(camera_pose.translation.distance(grid_pose.translation) - 1000.0, 0.0) / 1000.0,
					grid_pose
				)?)
			))}
		).collect()
	}

	// return true is this should be deleted
	pub fn clean_up(&mut self) -> bool {
		let mut sub_grids_to_remove = vec![];
		for (id, sub_grid) in &mut self.sub_grids {
			if sub_grid.clean_up() {
				sub_grids_to_remove.push(*id);
			}
		}
		if sub_grids_to_remove.len() == self.sub_grids.len() {
			return true;
		}
		for id in sub_grids_to_remove {
			self.sub_grids.remove(&id);
		}
		return false;
	}

	pub fn add_voxel(&mut self, pos: IVec3, voxel: voxels::Voxel) {
		self.mass += voxel.mass as u64;
		self.voxel_center_of_mass_times_mass += voxel.mass as i64 * pos.as_i64vec3();
		self.inertia_tensor_at_zero += InertiaTensor::get_inertia_tensor_for_cube_at_pos(voxel.mass as f64, 1.0, &(pos.as_dvec3() + 0.5));
		let (sub_grid, sub_grid_pos) = self.get_sub_grid_mut_create(&pos);
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

pub struct PhysicsBody {
	pub pose: Pose,
	pub velocity: Vec3,
	pub angular_velocity: Vec3,
	pub is_static: bool,
	grids: Vec<PhysicsBodyGrid>,
	grids_id_to_index: HashMap<u32, u32>,
	next_grid_id: u32,
	id: u32,
}

impl PhysicsBody {
	pub fn new(id: u32) -> Self {
		Self {
			pose: Pose::ZERO,
			velocity: Vec3::ZERO,
			angular_velocity: Vec3::ZERO,
			is_static: false,
			grids: vec![],
			grids_id_to_index: HashMap::new(),
			next_grid_id: 0,
			id: id,
		}
	}
	pub fn id(&self) -> u32 {
		self.id
	}
	pub fn mass(&self) -> f32 {
		let mut mass_sum = 0.0;
		for grid in self.grids.iter() {
			mass_sum += grid.mass();
		}
		mass_sum
	}
	pub fn get_local_center_of_mass(&self) -> Vec3 {
		let mut center_of_mass_times_mass = Vec3::ZERO;
		let mut mass_sum = 0.0;
		for grid in self.grids.iter() {
			center_of_mass_times_mass += grid.get_body_center_of_mass() * grid.mass();
			mass_sum += grid.mass();
		}
		if mass_sum == 0.0 { return Vec3::ZERO; }
		center_of_mass_times_mass / mass_sum
	}
	pub fn get_local_center_of_mass_and_mass(&self) -> (Vec3, f32) {
		let mut center_of_mass_times_mass = Vec3::ZERO;
		let mut mass_sum = 0.0;
		for grid in self.grids.iter() {
			center_of_mass_times_mass += grid.get_body_center_of_mass() * grid.mass();
			mass_sum += grid.mass();
		}
		if mass_sum == 0.0 { return (Vec3::ZERO, 0.0); }
		(center_of_mass_times_mass / mass_sum, mass_sum)
	}
	pub fn get_global_rotated_center_of_mass(&self) -> Vec3 {
		let mut center_of_mass_times_mass = Vec3::ZERO;
		let mut mass_sum = 0.0;
		for grid in self.grids.iter() {
			center_of_mass_times_mass += grid.get_body_center_of_mass() * grid.mass();
			mass_sum += grid.mass();
		}
		if mass_sum == 0.0 { return Vec3::ZERO; }
		self.pose.rotation * (center_of_mass_times_mass / mass_sum)
	}
	pub fn get_global_center_of_mass(&self) -> Vec3 {
		let mut center_of_mass_times_mass = Vec3::ZERO;
		let mut mass_sum = 0.0;
		for grid in self.grids.iter() {
			center_of_mass_times_mass += grid.get_body_center_of_mass() * grid.mass();
			mass_sum += grid.mass();
		}
		if mass_sum == 0.0 { return Vec3::ZERO; }
		self.pose * (center_of_mass_times_mass / mass_sum)
	}
	pub fn get_global_center_of_mass_and_mass(&self) -> (Vec3, f32) {
		let mut center_of_mass_times_mass = Vec3::ZERO;
		let mut mass_sum = 0.0;
		for grid in self.grids.iter() {
			center_of_mass_times_mass += grid.get_body_center_of_mass() * grid.mass();
			mass_sum += grid.mass();
		}
		if mass_sum == 0.0 { return (Vec3::ZERO, 0.0); }
		(self.pose * (center_of_mass_times_mass / mass_sum), mass_sum)
	}
	pub fn rotational_inertia(&self) -> InertiaTensor {
		let (com, mass) = self.get_local_center_of_mass_and_mass();
		let mut inertia_tensor_at_zero: InertiaTensor = InertiaTensor::ZERO;
		for grid in self.grids.iter() {
			inertia_tensor_at_zero += grid.get_inertia_tensor_at_body();
		}
		inertia_tensor_at_zero.move_to_center_of_mass(
			&com.as_dvec3(),
			mass as f64
		).get_rotated(self.pose.rotation.as_dquat())
	}
	pub fn rotational_inertia_at_zero(&self) -> InertiaTensor {
		let mut inertia_tensor_at_zero: InertiaTensor = InertiaTensor::ZERO;
		for grid in self.grids.iter() {
			inertia_tensor_at_zero += grid.get_inertia_tensor_at_body();
		}
		inertia_tensor_at_zero.get_rotated(self.pose.rotation.as_dquat())
	}

	pub fn update_gpu_grid_tree(
		&self,
		device: &wgpu::Device,
		queue: &wgpu::Queue,
		packed_64_tree_dynamic_buffer: &mut PackedDynamicBuffer,
		packed_voxel_data_dynamic_buffer: &mut PackedDynamicBuffer,
		view_frustum: &camera::ViewFrustum,
		camera_pose: Pose,
	) -> Vec<((u32, IVec3), (u32, u32, Pose))> {
		let mut gpu_grid_tree_id_to_id_poses = vec![];
		for (grid_index, grid) in self.grids.iter().enumerate() {
			for (sub_grid_pos, data) in grid.update_gpu_grid_tree(device, queue, packed_64_tree_dynamic_buffer, packed_voxel_data_dynamic_buffer, view_frustum, camera_pose, &self.pose) {
				gpu_grid_tree_id_to_id_poses.push(((grid_index as u32, sub_grid_pos), data));
			}
		}
		gpu_grid_tree_id_to_id_poses
	}

	// return true is this should be deleted
	pub fn clean_up(&mut self) -> bool {
		let mut index = 0u32;
		while index < self.grids.len() as u32 {
			if self.grids[index as usize].clean_up() {
				self.remove_grid_by_index(index);
			} else {
				index += 1;
			}
		}
		return self.grids.is_empty();
	}

	pub fn add_grid(&mut self, grid_pose: Pose) -> u32 {
		self.grids_id_to_index.insert(self.next_grid_id, self.grids.len() as u32);
		self.grids.push(PhysicsBodyGrid::new(self.next_grid_id, &grid_pose));
		self.next_grid_id += 1;
		self.next_grid_id - 1
	}

	pub fn remove_grid(&mut self, grid_id: u32) {
		let index = match self.grids_id_to_index.remove(&grid_id) {
			Some(i) => i,
			None => return,
		};
		self.grids.swap_remove(index as usize);
		if index != self.grids.len() as u32 {
			let other_id = self.grids[index as usize].id();
			self.grids_id_to_index.insert(other_id, index);
		}
	}

	pub fn remove_grid_by_index(&mut self, index: u32) {
		if let Some(grid) = self.grids.get(index as usize) {
			self.grids_id_to_index.remove(&grid.id());
		} else { return; }
		self.grids.swap_remove(index as usize);
		if index != self.grids.len() as u32 {
			let other_id = self.grids[index as usize].id();
			self.grids_id_to_index.insert(other_id, index);
		}
	}

	pub fn grid(&self, grid_id: u32) -> Option<&PhysicsBodyGrid> {
		let index = *self.grids_id_to_index.get(&grid_id)?;
		self.grids.get(index as usize)
	}

	pub fn grid_by_index(&self, grid_index: u32) -> Option<&PhysicsBodyGrid> {
		self.grids.get(grid_index as usize)
	}

	pub fn grid_mut(&mut self, grid_id: u32) -> Option<&mut PhysicsBodyGrid> {
		let index = *self.grids_id_to_index.get(&grid_id)?;
		self.grids.get_mut(index as usize)
	}

	pub fn grid_by_index_mut(&mut self, grid_index: u32) -> Option<&mut PhysicsBodyGrid> {
		self.grids.get_mut(grid_index as usize)
	}

	pub fn grids(&self) -> &[PhysicsBodyGrid] {
		&self.grids
	}

	pub fn aabb(&self) -> Option<(Vec3, Vec3)> {
		let mut aabb_min = Vec3::splat(f32::MAX);
		let mut aabb_max = Vec3::splat(f32::MIN);
		for grid in &self.grids {
			for (sub_grid_pos, sub_grid) in grid.get_sub_grids() {
				if let Some((min, max)) = sub_grid.get_voxels().get_bounding_box() {
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
					let sub_grid_grid_pos = grid.sub_grid_pos_to_grid_pos(&sub_grid_pos).as_vec3();
					let rotated_corners = corners.map(|c| self.pose * grid.pose * (c + sub_grid_grid_pos));
					aabb_min = aabb_min.min(rotated_corners.iter().fold(Vec3::splat(f32::MAX), |acc, c| acc.min(*c)));
					aabb_max = aabb_max.max(rotated_corners.iter().fold(Vec3::splat(f32::MIN), |acc, c| acc.max(*c)));
				}
			}
		}
		if aabb_min.x == f32::MAX { return None; }
		Some((aabb_min, aabb_max))
	}

	pub fn grid_aabb(&self, grid_index: u32) -> Option<(Vec3, Vec3)> {
		let mut aabb_min = Vec3::splat(f32::MAX);
		let mut aabb_max = Vec3::splat(f32::MIN);
		let grid = self.grids.get(grid_index as usize)?;
		for (sub_grid_pos, sub_grid) in grid.get_sub_grids() {
			if let Some((min, max)) = sub_grid.get_voxels().get_bounding_box() {
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
				let sub_grid_grid_pos = grid.sub_grid_pos_to_grid_pos(&sub_grid_pos).as_vec3();
				let rotated_corners = corners.map(|c| self.pose * grid.pose * (c + sub_grid_grid_pos));
				aabb_min = aabb_min.min(rotated_corners.iter().fold(Vec3::splat(f32::MAX), |acc, c| acc.min(*c)));
				aabb_max = aabb_max.max(rotated_corners.iter().fold(Vec3::splat(f32::MIN), |acc, c| acc.max(*c)));
			}
		}
		Some((aabb_min, aabb_max))
	}

	pub fn sub_grid_aabb_by_index(&self, grid_index: u32, sub_grid_pos: &IVec3) -> Option<(Vec3, Vec3)> {
		let grid = self.grids.get(grid_index as usize)?;
		let sub_grid = grid.get_sub_grid_from_sub_grid_pos(sub_grid_pos)?;
		let (min, max) = sub_grid.get_voxels().get_bounding_box()?;
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
		let sub_grid_grid_pos = grid.sub_grid_pos_to_grid_pos(&sub_grid_pos).as_vec3();
		let rotated_corners = corners.map(|c| self.pose * grid.pose * (c + sub_grid_grid_pos));
		Some((
			rotated_corners.iter().fold(Vec3::splat(f32::MAX), |acc, c| acc.min(*c)),
			rotated_corners.iter().fold(Vec3::splat(f32::MIN), |acc, c| acc.max(*c))
		))
	}

	pub fn render_debug_inertia_box(&self) {
		self.rotational_inertia().render_debug_box(self.mass(), self.get_global_center_of_mass());
	}

	pub fn world_to_local(&self, other: &Pose) -> Pose { self.pose.inverse() * other }
	pub fn local_to_world(&self, other: &Pose) -> Pose { self.pose * other }
	pub fn world_to_local_vec(&self, vec: &Vec3) -> Vec3 { self.pose.inverse() * vec }
	pub fn local_to_world_vec(&self, vec: &Vec3) -> Vec3 { self.pose * vec }
	pub fn world_to_local_rot(&self, rot: &Quat) -> Quat { self.pose.inverse() * rot }
	pub fn local_to_world_rot(&self, rot: &Quat) -> Quat { self.pose * rot }

	pub fn world_to_grid(&self, grid_index: u32, other: &Pose) -> Pose { self.grids[grid_index as usize].body_to_local(&self.world_to_local(other)) }
	pub fn grid_to_world(&self, grid_index: u32, other: &Pose) -> Pose { self.local_to_world(&self.grids[grid_index as usize].local_to_body(other)) }
	pub fn world_to_grid_vec(&self, grid_index: u32, pos: &Vec3) -> Vec3 { self.grids[grid_index as usize].body_to_local_vec(&self.world_to_local_vec(pos)) }
	pub fn grid_to_world_vec(&self, grid_index: u32, pos: &Vec3) -> Vec3 { self.local_to_world_vec(&self.grids[grid_index as usize].local_to_body_vec(pos)) }
	pub fn world_to_grid_rot(&self, grid_index: u32, rot: &Quat) -> Quat { self.grids[grid_index as usize].body_to_local_rot(&self.world_to_local_rot(rot)) }
	pub fn grid_to_world_rot(&self, grid_index: u32, rot: &Quat) -> Quat { self.local_to_world_rot(&self.grids[grid_index as usize].local_to_body_rot(rot)) }
}
