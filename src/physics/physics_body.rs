use std::{cell::Cell, collections::HashMap, sync::Arc};

use glam::{DMat3, DVec3, IVec3, Mat3, Mat4, Quat, Vec3};
use crate::{camera, gpu_objects::mesh::{self, GetMesh}, pose::Pose, voxels};

pub struct PhysicsBodySubGrid {
	pub pose: Pose,
	voxels: voxels::Voxels,
	mesh: Cell<Option<Arc<mesh::Mesh>>>,
	mass: f64,
	voxel_center_of_mass_times_mass: DVec3,
	inertia_tensor_at_zero: DMat3,
	id: u32,

}

// According to https://en.wikipedia.org/wiki/Moment_of_inertia#:~:text=in%20the%20body.-,Inertia%20tensor
fn get_inertia_tensor_for_cube(pos: &IVec3, mass: f32) -> Mat3 {
	let center_pos = pos.as_vec3() + Vec3::new(0.5, 0.5, 0.5);
	let i_center = mass / 6.0;
	let r2 = center_pos.length_squared();

	let outer = Mat3::from_cols(
		center_pos * center_pos.x,
		center_pos * center_pos.y,
		center_pos * center_pos.z,
	);

	let parallel_axis = mass * (r2 * Mat3::IDENTITY - outer);
	Mat3::IDENTITY * i_center + parallel_axis
}

fn move_inertia_tensor_to_center_of_mass(inertia_tensor: DMat3, center_of_mass: DVec3, mass: f64) -> DMat3 {
	inertia_tensor - ((DMat3::IDENTITY * center_of_mass.length_squared()) - DMat3::from_cols_array(&[
	center_of_mass.x * center_of_mass.x, center_of_mass.x * center_of_mass.y, center_of_mass.x * center_of_mass.z,
	center_of_mass.y * center_of_mass.x, center_of_mass.y * center_of_mass.y, center_of_mass.y * center_of_mass.z,
	center_of_mass.z * center_of_mass.x, center_of_mass.z * center_of_mass.y, center_of_mass.z * center_of_mass.z])) * mass
}

fn combine_inertia_tensors(inertia_tensor_a: DMat3, inertia_tensor_b: DMat3) -> DMat3 {
	inertia_tensor_a + inertia_tensor_b
}

impl PhysicsBodySubGrid {
	pub fn new(sub_grid_id: u32, pose: &Pose) -> Self {
		Self {
			pose: *pose,
			voxels: voxels::Voxels::new(),
			mesh: Cell::new(None),
			mass: 0.0,
			voxel_center_of_mass_times_mass: DVec3::ZERO,
			inertia_tensor_at_zero: DMat3::ZERO,
			id: sub_grid_id,
		}
	}

	pub fn id(&self) -> u32 {
		self.id
	}

	pub fn get_local_center_of_mass(&self) -> Vec3 {
		if self.mass == 0.0 {
			Vec3::ZERO
		} else {
			(self.voxel_center_of_mass_times_mass / self.mass).as_vec3()
		}
	}
	pub fn get_center_of_mass_time_mass(&self) -> Vec3 {
		if self.mass == 0.0 {
			Vec3::ZERO
		} else {
			self.voxel_center_of_mass_times_mass.as_vec3()
		}
	}
	pub fn mass(&self) -> f32 { self.mass as f32 }

	pub fn get_rendering_meshes(&self, device: &wgpu::Device, _camera: &camera::Camera) -> Vec<Arc<mesh::Mesh>> {
		let mesh = self.mesh.take();
		if mesh.is_none() {
			self.mesh.set(match self.voxels.get_mesh(device) {
				Some(mesh) => Some(Arc::new(mesh)),
				None => None,
			});
			return vec![self.mesh.take().unwrap()];
		}
		vec![mesh.unwrap()]
	}

	pub fn add_voxel(&mut self, pos: IVec3, voxel: voxels::Voxel) {
		self.mesh.set(None);
		let mass = voxel.mass as f64;
		self.inertia_tensor_at_zero = combine_inertia_tensors(self.inertia_tensor_at_zero, get_inertia_tensor_for_cube(&pos, voxel.mass).as_dmat3());
		self.voxel_center_of_mass_times_mass += mass * (pos.as_dvec3() + 0.5);
		self.mass += mass;
		if let Some(old_voxel) = self.voxels.add_voxel(pos, voxel) {
			let old_mass = old_voxel.mass as f64;
			self.mass -= old_mass;
			self.voxel_center_of_mass_times_mass -= old_mass * (pos.as_dvec3() + 0.5);
			self.inertia_tensor_at_zero = combine_inertia_tensors(self.inertia_tensor_at_zero, -get_inertia_tensor_for_cube(&pos, old_voxel.mass).as_dmat3());
		}
	}

	pub fn remove_voxel(&mut self, pos: &IVec3) {
		if let Some(voxel) = self.voxels.remove_voxel(pos) {
			self.mesh.set(None);
			let mass = voxel.mass as f64;
			self.mass -= mass;
			self.voxel_center_of_mass_times_mass -= mass * (pos.as_dvec3() + 0.5);
			self.inertia_tensor_at_zero = combine_inertia_tensors(self.inertia_tensor_at_zero, -get_inertia_tensor_for_cube(&pos, voxel.mass).as_dmat3());
		}
	}

	pub fn get_voxel(&self, pos: IVec3) -> Option<&voxels::Voxel> { self.voxels.get_voxel(pos) }
	pub fn get_voxels(&self) -> &voxels::Voxels { &self.voxels }

	pub fn body_to_local(&self, other: &Pose) -> Pose { self.pose.inverse() * other }
	pub fn local_to_body(&self, other: &Pose) -> Pose { self.pose * other }
	pub fn body_to_local_vec(&self, pos: &Vec3) -> Vec3 { self.pose.inverse() * pos }
	pub fn local_to_body_vec(&self, pos: &Vec3) -> Vec3 { self.pose * pos }
	pub fn body_to_local_rot(&self, rot: &Quat) -> Quat { self.pose.inverse() * rot }
	pub fn local_to_body_rot(&self, rot: &Quat) -> Quat { self.pose * rot }

	pub fn get_inertia_tensor_at_center_of_mass(&self) -> Mat3 {
		move_inertia_tensor_to_center_of_mass(self.inertia_tensor_at_zero, self.voxel_center_of_mass_times_mass / self.mass, self.mass).as_mat3()
	}
}

pub struct PhysicsBody {
	pub pose: Pose,
	pub velocity: Vec3,
	pub angular_velocity: Vec3,
	pub is_static: bool,
	sub_grids: Vec<PhysicsBodySubGrid>,
	sub_grids_id_to_index: HashMap<u32, u32>,
	next_sub_grid_id: u32,
	id: u32,
}

impl PhysicsBody {
	pub fn new(id: u32) -> Self {
		Self {
			pose: Pose::ZERO,
			velocity: Vec3::ZERO,
			angular_velocity: Vec3::ZERO,
			is_static: false,
			sub_grids: vec![],
			sub_grids_id_to_index: HashMap::new(),
			next_sub_grid_id: 0,
			id: id,
		}
	}
	pub fn id(&self) -> u32 {
		self.id
	}
	pub fn mass(&self) -> f32 { self.sub_grids.first().unwrap().mass as f32 }
	pub fn get_local_center_of_mass(&self) -> Vec3 {
		let mut center_of_mass = Vec3::ZERO;
		let mut mass_sum = 0.0;
		for sub_grid in self.sub_grids.iter() {
			center_of_mass += sub_grid.get_center_of_mass_time_mass();
			mass_sum += sub_grid.mass();
		}
		center_of_mass / mass_sum
	}
	pub fn get_center_of_mass(&self) -> Vec3 {
		let mut center_of_mass = Vec3::ZERO;
		let mut mass_sum = 0.0;
		for sub_grid in self.sub_grids.iter() {
			center_of_mass += sub_grid.get_center_of_mass_time_mass();
			mass_sum += sub_grid.mass();
		}
		self.pose.rotation * (center_of_mass / mass_sum)
	}
	pub fn rotational_inertia(&self) -> Mat3 {
		let mat = self.sub_grids.first().unwrap().get_inertia_tensor_at_center_of_mass();
		mat
	}

	pub fn get_rendering_meshes(&self, device: &wgpu::Device, camera: &camera::Camera) -> Vec<(Arc<mesh::Mesh>, Mat4)> {
		let mut meshes: Vec<(Arc<mesh::Mesh>, Mat4)> = vec![];
		for sub_grid in self.sub_grids.iter() {
			let pose = self.pose * sub_grid.pose;
			let matrix = Mat4::from_rotation_translation(pose.rotation, pose.translation);
			for mesh in sub_grid.get_rendering_meshes(device, camera) {
				meshes.push((mesh, matrix));
			}
		}
		meshes
	}

	pub fn add_sub_grid(&mut self, sub_grid_pose: Pose) -> u32 {
		self.sub_grids_id_to_index.insert(self.next_sub_grid_id, self.sub_grids.len() as u32);
		self.sub_grids.push(PhysicsBodySubGrid::new(self.next_sub_grid_id, &sub_grid_pose));
		self.next_sub_grid_id += 1;
		self.next_sub_grid_id - 1
	}

	pub fn remove_sub_grid(&mut self, physics_body_id: u32) {
		let index = match self.sub_grids_id_to_index.remove(&physics_body_id) {
			Some(i) => i,
			None => return,
		};
		self.sub_grids.swap_remove(index as usize);
		if index != self.sub_grids.len() as u32 {
			let other_id = self.sub_grids[index as usize].id();
			self.sub_grids_id_to_index.insert(other_id, index);
		}
	}

	pub fn sub_grid(&self, physics_body_id: u32) -> Option<&PhysicsBodySubGrid> {
		let index = *self.sub_grids_id_to_index.get(&physics_body_id)?;
		self.sub_grids.get(index as usize)
	}

	pub fn sub_grid_mut(&mut self, physics_body_id: u32) -> Option<&mut PhysicsBodySubGrid> {
		let index = *self.sub_grids_id_to_index.get(&physics_body_id)?;
		self.sub_grids.get_mut(index as usize)
	}

	pub fn sub_grids(&self) -> &[PhysicsBodySubGrid] {
		&self.sub_grids
	}

	pub fn aabb(&self) -> (Vec3, Vec3) {
		let mut aabb_min = Vec3::splat(f32::MAX);
		let mut aabb_max = Vec3::splat(f32::MIN);
		for sub_grid in &self.sub_grids {
			let (min, max) = sub_grid.get_voxels().get_bounding_box().unwrap();
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
			let rotated_corners = corners.map(|c| self.pose * sub_grid.pose * c);
			aabb_min = aabb_min.min(rotated_corners.iter().fold(Vec3::splat(f32::MAX), |acc, c| acc.min(*c)));
			aabb_max = aabb_max.max(rotated_corners.iter().fold(Vec3::splat(f32::MIN), |acc, c| acc.max(*c)));
		}
		(aabb_min, aabb_max)
	}

	pub fn world_to_local(&self, other: &Pose) -> Pose { self.pose.inverse() * other }
	pub fn local_to_world(&self, other: &Pose) -> Pose { self.pose * other }
	pub fn world_to_local_vec(&self, vec: &Vec3) -> Vec3 { self.pose.inverse() * vec }
	pub fn local_to_world_vec(&self, vec: &Vec3) -> Vec3 { self.pose * vec }
	pub fn world_to_local_rot(&self, rot: &Quat) -> Quat { self.pose.inverse() * rot }
	pub fn local_to_world_rot(&self, rot: &Quat) -> Quat { self.pose * rot }

	pub fn world_to_sub_grid(&self, sub_grid_index: u32, other: &Pose) -> Pose { self.sub_grids[sub_grid_index as usize].body_to_local(&self.world_to_local(other)) }
	pub fn sub_grid_to_world(&self, sub_grid_index: u32, other: &Pose) -> Pose { self.local_to_world(&self.sub_grids[sub_grid_index as usize].local_to_body(other)) }
	pub fn world_to_sub_grid_vec(&self, sub_grid_index: u32, pos: &Vec3) -> Vec3 { self.sub_grids[sub_grid_index as usize].body_to_local_vec(&self.world_to_local_vec(pos)) }
	pub fn sub_grid_to_world_vec(&self, sub_grid_index: u32, pos: &Vec3) -> Vec3 { self.local_to_world_vec(&self.sub_grids[sub_grid_index as usize].local_to_body_vec(pos)) }
	pub fn world_to_sub_grid_rot(&self, sub_grid_index: u32, rot: &Quat) -> Quat { self.sub_grids[sub_grid_index as usize].body_to_local_rot(&self.world_to_local_rot(rot)) }
	pub fn sub_grid_to_world_rot(&self, sub_grid_index: u32, rot: &Quat) -> Quat { self.local_to_world_rot(&self.sub_grids[sub_grid_index as usize].local_to_body_rot(rot)) }
}
