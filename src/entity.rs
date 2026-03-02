use std::collections::HashMap;

use glam::{DMat3, DVec3, IVec3, Mat3, Mat4, Quat, Vec3};
use crate::{camera, gpu_objects::mesh::{self, GetMesh}, voxels};

pub struct Entity {
	pub position: Vec3,
	pub orientation: Quat,
	pub momentum: Vec3,
	pub angular_momentum: Vec3,
	center_of_mass_times_mass: DVec3,
	mass: f64,
	inertia_tensor_at_origin: DMat3,
	voxels: voxels::Voxels,
	mesh: Option<mesh::Mesh>,
	update_mesh: bool,
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

impl Entity {
	pub fn new() -> Self {
		Self {
			position: Vec3::ZERO,
			orientation: Quat::IDENTITY,
			momentum: Vec3::ZERO,
			angular_momentum: Vec3::ZERO,
			center_of_mass_times_mass: DVec3::ZERO,
			mass: 0.0,
			inertia_tensor_at_origin: DMat3::ZERO,
			voxels: voxels::Voxels::new(),
			mesh: None,
			update_mesh: true,
		}
	}
	pub fn center_of_mass(&self) -> Vec3 {
		if self.mass == 0.0 {
			Vec3::ZERO
		} else {
			(self.center_of_mass_times_mass / self.mass).as_vec3()
		}
	}
	pub fn mass(&self) -> f32 { self.mass as f32 }
	pub fn rotational_inertia(&self) -> Mat3 {
		// apply Inertia tensor of translation
		let center_of_mass = self.center_of_mass();
		self.inertia_tensor_at_origin.as_mat3() - ((Mat3::IDENTITY * center_of_mass.length_squared()) - Mat3::from_cols_array(&[
			center_of_mass.x * center_of_mass.x, center_of_mass.x * center_of_mass.y, center_of_mass.x * center_of_mass.z,
			center_of_mass.y * center_of_mass.x, center_of_mass.y * center_of_mass.y, center_of_mass.y * center_of_mass.z,
			center_of_mass.z * center_of_mass.x, center_of_mass.z * center_of_mass.y, center_of_mass.z * center_of_mass.z
		])) * (self.mass as f32)
	}
	pub fn add_voxel(&mut self, pos: IVec3, voxel: voxels::Voxel) {
		self.update_mesh = true;
		let mass = voxel.mass as f64;
		self.mass += mass;
		self.center_of_mass_times_mass += mass * pos.as_dvec3();
		self.inertia_tensor_at_origin += get_inertia_tensor_for_cube(&pos, voxel.mass).as_dmat3();
		if let Some(old_voxel) = self.voxels.add_voxel(pos, voxel) {
			let old_mass = old_voxel.mass as f64;
			self.mass -= old_mass;
			self.center_of_mass_times_mass -= old_mass * pos.as_dvec3();
			self.inertia_tensor_at_origin -= get_inertia_tensor_for_cube(&pos, old_voxel.mass).as_dmat3();
		}
	}

	pub fn remove_voxel(&mut self, pos: &IVec3) {
		if let Some(voxel) = self.voxels.remove_voxel(pos) {
			self.update_mesh = true;
			let mass = voxel.mass as f64;
			self.mass -= mass;
			self.center_of_mass_times_mass -= mass * pos.as_dvec3();
			self.inertia_tensor_at_origin -= get_inertia_tensor_for_cube(&pos, voxel.mass).as_dmat3();
		}
	}

	pub fn get_voxel(&self, pos: IVec3) -> Option<&voxels::Voxel> { self.voxels.get_voxel(pos) }
	pub fn get_voxels(&self) -> &voxels::Voxels { &self.voxels }

	pub fn get_rendering_meshes(&mut self, device: &wgpu::Device, camera: &camera::Camera) -> Vec<(&mesh::Mesh, Mat4)> {
		if self.update_mesh {
			self.mesh = self.voxels.get_mesh(device);
			self.update_mesh = false;
		}
		if self.mesh.is_some() {
			let matrix = Mat4::from_translation(self.position) * Mat4::from_quat(self.orientation);
			return vec![( self.mesh.as_ref().unwrap(), matrix )]
		}
		vec![]
	}
}
