use std::{collections::HashMap, sync::Arc};

use glam::{I16Vec3, I64Vec3, Mat4, Quat, Vec3};
use tracy_client::span;
use crate::{gpu_objects::mesh::{self, GetMesh}, player::camera, pose::Pose, voxels};

use super::inertia_tensor::InertiaTensor;

pub struct PhysicsBodyGrid {
	pub pose: Pose,
	voxels: voxels::Voxels,
	mesh: clone_cell::cell::Cell<Option<Arc<mesh::Mesh>>>,
	mass: u64,
	voxel_center_of_mass_times_mass: I64Vec3,
	inertia_tensor_at_zero: InertiaTensor,
	id: u32,
}

impl PhysicsBodyGrid {
	pub fn new(grid_id: u32, pose: &Pose) -> Self {
		Self {
			pose: *pose,
			voxels: voxels::Voxels::new(),
			mesh: clone_cell::cell::Cell::new(None),
			mass: 0,
			voxel_center_of_mass_times_mass: I64Vec3::ZERO,
			inertia_tensor_at_zero: InertiaTensor::ZERO,
			id: grid_id,
		}
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

	pub fn get_rendering_meshes(&self, device: &wgpu::Device, _camera: &camera::Camera) -> Vec<Arc<mesh::Mesh>> {
		let mesh = self.mesh.get();
		if mesh.is_none() {
			let _zone = span!("Recreate Mesh");
			self.mesh.set(match self.voxels.get_mesh(device) {
				Some(mesh) => Some(Arc::new(mesh)),
				None => None,
			});
			return vec![self.mesh.get().unwrap()];
		}
		vec![mesh.unwrap()]
	}

	pub fn add_voxel(&mut self, pos: I16Vec3, voxel: voxels::Voxel) {
		self.mesh.set(None);
		self.mass += voxel.mass as u64;
		self.voxel_center_of_mass_times_mass += voxel.mass as i64 * pos.as_i64vec3();
		self.inertia_tensor_at_zero += InertiaTensor::get_inertia_tensor_for_cube_at_pos(voxel.mass as f64, 1.0, &(pos.as_dvec3() + 0.5));
		if let Some(old_voxel) = self.voxels.add_voxel(pos, voxel) {
			self.mass -= old_voxel.mass as u64;
			self.voxel_center_of_mass_times_mass -= old_voxel.mass as i64 * pos.as_i64vec3();
			self.inertia_tensor_at_zero -= InertiaTensor::get_inertia_tensor_for_cube_at_pos(old_voxel.mass as f64, 1.0, &(pos.as_dvec3() + 0.5));
		}
	}

	pub fn remove_voxel(&mut self, pos: &I16Vec3) {
		if let Some(voxel) = self.voxels.remove_voxel(pos) {
			self.mesh.set(None);
			self.mass -= voxel.mass as u64;
			self.voxel_center_of_mass_times_mass -= voxel.mass as i64 * pos.as_i64vec3();
			self.inertia_tensor_at_zero -= InertiaTensor::get_inertia_tensor_for_cube_at_pos(voxel.mass as f64, 1.0, &(pos.as_dvec3() + 0.5));
		}
	}

	pub fn get_voxel(&self, pos: I16Vec3) -> Option<&voxels::Voxel> { self.voxels.get_voxel(pos) }
	pub fn get_voxels(&self) -> &voxels::Voxels { &self.voxels }

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

	pub fn get_rendering_meshes(&self, device: &wgpu::Device, camera: &camera::Camera) -> Vec<(Arc<mesh::Mesh>, Mat4)> {
		let mut meshes: Vec<(Arc<mesh::Mesh>, Mat4)> = vec![];
		for grid in self.grids.iter() {
			let pose = self.pose * grid.pose;
			let matrix = Mat4::from_rotation_translation(pose.rotation, pose.translation);
			for mesh in grid.get_rendering_meshes(device, camera) {
				meshes.push((mesh, matrix));
			}
		}
		meshes
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

	pub fn aabb(&self) -> (Vec3, Vec3) {
		let mut aabb_min = Vec3::splat(f32::MAX);
		let mut aabb_max = Vec3::splat(f32::MIN);
		for grid in &self.grids {
			let (min, max) = grid.get_voxels().get_bounding_box().unwrap();
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
			let rotated_corners = corners.map(|c| self.pose * grid.pose * c);
			aabb_min = aabb_min.min(rotated_corners.iter().fold(Vec3::splat(f32::MAX), |acc, c| acc.min(*c)));
			aabb_max = aabb_max.max(rotated_corners.iter().fold(Vec3::splat(f32::MIN), |acc, c| acc.max(*c)));
		}
		(aabb_min, aabb_max)
	}

	pub fn grid_aabb(&self, grid_id: u32) -> Option<(Vec3, Vec3)> {
		let grid = self.grids.get(grid_id as usize)?;
		let (min, max) = grid.get_voxels().get_bounding_box()?;
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
		let rotated_corners = corners.map(|c| self.pose * grid.pose * c);
		Some((
			rotated_corners.iter().fold(Vec3::splat(f32::MAX), |acc, c| acc.min(*c)),
			rotated_corners.iter().fold(Vec3::splat(f32::MIN), |acc, c| acc.max(*c))
		))
	}

	pub fn render_debug_inertia_box(&self) {
		self.rotational_inertia().render_debug_box(self.mass(), self.get_global_center_of_mass());
	}

	pub fn apply_central_impulse(&mut self, impluse: &Vec3) {
		self.velocity += impluse / self.mass();
	}
	pub fn apply_rotational_impulse(&mut self, rotational_impluse: &Vec3) {
		self.angular_velocity += self.rotational_inertia().mat.as_mat3().inverse() * rotational_impluse;
	}
	pub fn apply_impulse(&mut self, impluse_pos: &Vec3, impluse: &Vec3) {
		self.velocity += impluse / self.mass();
		self.angular_velocity += self.rotational_inertia().mat.as_mat3().inverse() * (impluse_pos - self.get_global_center_of_mass()).cross(*impluse);
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
