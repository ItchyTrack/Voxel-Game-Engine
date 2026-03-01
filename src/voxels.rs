use glam::{IVec3, Mat3, Vec3, DVec3, DMat3};
use wgpu::util::DeviceExt;
use std::{collections::HashMap};

use crate::gpu_objects::mesh;
use crate::gpu_objects::matrix;

#[derive(Clone, Copy)]
pub struct Voxel {
	pub color: [f32; 4],
	pub mass: f32,
}

pub struct Voxels {
	voxels: HashMap<IVec3, Voxel>,
	center_of_mass_times_mass: DVec3,
	mass: f64,
	inertia_tensor_at_origin: DMat3,
}

// According to https://en.wikipedia.org/wiki/Moment_of_inertia#:~:text=in%20the%20body.-,Inertia%20tensor
fn get_inertia_tensor_at_point(pos: Vec3, mass: f32) -> Mat3 {
	let x_sqr = pos.x * pos.x;
	let y_sqr = pos.y * pos.y;
	let z_sqr = pos.z * pos.z;
	Mat3::from_cols_array(&[
		y_sqr + z_sqr, -mass * pos.x * pos.y, -mass * pos.x * pos.z,
		-mass * pos.x * pos.y, z_sqr + x_sqr, -mass * pos.y * pos.z,
		-mass * pos.x * pos.z, -mass * pos.y * pos.z, x_sqr + y_sqr,
	])
}

impl Voxels {
	pub fn new() -> Self {
		Self {
			voxels: HashMap::new(),
			center_of_mass_times_mass: DVec3::ZERO,
			mass: 0.0,
			inertia_tensor_at_origin: DMat3::ZERO,
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

	pub fn set_voxel(&mut self, pos: IVec3, voxel: Voxel) {
		let mass = voxel.mass as f64;
		self.mass += mass;
		self.center_of_mass_times_mass += mass * pos.as_dvec3();
		self.inertia_tensor_at_origin += get_inertia_tensor_at_point(pos.as_vec3(), voxel.mass).as_dmat3();
		if let Some(old_voxel) = self.voxels.insert(pos, voxel) {
			let old_mass = old_voxel.mass as f64;
			self.mass -= old_mass;
			self.center_of_mass_times_mass -= old_mass * pos.as_dvec3();
			self.inertia_tensor_at_origin -= get_inertia_tensor_at_point(pos.as_vec3(), old_voxel.mass).as_dmat3();
		}
	}

	pub fn delete_voxel(&mut self, pos: IVec3) {
		if let Some(voxel) = self.voxels.remove(&pos) {
			let mass = voxel.mass as f64;
			self.mass -= mass;
			self.center_of_mass_times_mass -= mass * pos.as_dvec3();
			self.inertia_tensor_at_origin -= get_inertia_tensor_at_point(pos.as_vec3(), voxel.mass).as_dmat3();
		}
	}

	pub fn get_voxel(&self, pos: IVec3) -> Option<&Voxel> { self.voxels.get(&pos) }
	pub fn get_voxels(&self) -> &HashMap<IVec3, Voxel> { &self.voxels }
}

impl mesh::GetMesh for Voxels {
	fn get_mesh(&self, device: &wgpu::Device) -> Option<mesh::Mesh> {
		if self.voxels.is_empty() {
			return None;
		}
		let mut verties: Vec<mesh::MeshVertex> = vec![];
		let mut indexes: Vec<u32> = vec![];
		for (pos, voxel) in &self.voxels {
			let fpos: Vec3 = pos.as_vec3();
			let start_verties = verties.len() as u32;
			verties.push(mesh::MeshVertex { // 0
				position: fpos.to_array(),
				color: voxel.color,
				normal: [0.0, 0.0, 0.0]
			});
			verties.push(mesh::MeshVertex { // 1
				position: (fpos + Vec3::new(1.0, 0.0, 0.0)).to_array(),
				color: voxel.color,
				normal: [0.0, 0.0, 0.0]
			});
			verties.push(mesh::MeshVertex { // 2
				position: (fpos + Vec3::new(0.0, 1.0, 0.0)).to_array(),
				color: voxel.color,
				normal: [0.0, 0.0, 0.0]
			});
			verties.push(mesh::MeshVertex { // 3
				position: (fpos + Vec3::new(0.0, 0.0, 1.0)).to_array(),
				color: voxel.color,
				normal: [0.0, 0.0, 0.0]
			});
			verties.push(mesh::MeshVertex { // 4
				position: (fpos + Vec3::new(1.0, 1.0, 0.0)).to_array(),
				color: voxel.color,
				normal: [0.0, 0.0, 0.0]
			});
			verties.push(mesh::MeshVertex { // 5
				position: (fpos + Vec3::new(1.0, 0.0, 1.0)).to_array(),
				color: voxel.color,
				normal: [0.0, 0.0, 0.0]
			});
			verties.push(mesh::MeshVertex { // 6
				position: (fpos + Vec3::new(0.0, 1.0, 1.0)).to_array(),
				color: voxel.color,
				normal: [0.0, 0.0, 0.0]
			});
			verties.push(mesh::MeshVertex { // 7
				position: (fpos + Vec3::new(1.0, 1.0, 1.0)).to_array(),
				color: voxel.color,
				normal: [0.0, 0.0, 0.0]
			});
			if !self.voxels.contains_key(&(pos + IVec3::X)) {
				indexes.push(start_verties + 5); // +x
				indexes.push(start_verties + 4);
				indexes.push(start_verties + 7);
				indexes.push(start_verties + 5);
				indexes.push(start_verties + 1);
				indexes.push(start_verties + 4);
			}
			if !self.voxels.contains_key(&(pos - IVec3::X)) {
				indexes.push(start_verties + 0); // -x
				indexes.push(start_verties + 3);
				indexes.push(start_verties + 2);
				indexes.push(start_verties + 2);
				indexes.push(start_verties + 3);
				indexes.push(start_verties + 6);
			}
			if !self.voxels.contains_key(&(pos + IVec3::Y)) {
				indexes.push(start_verties + 7); // +y
				indexes.push(start_verties + 4);
				indexes.push(start_verties + 6);
				indexes.push(start_verties + 4);
				indexes.push(start_verties + 2);
				indexes.push(start_verties + 6);
			}
			if !self.voxels.contains_key(&(pos - IVec3::Y)) {
				indexes.push(start_verties + 0); // -y
				indexes.push(start_verties + 1);
				indexes.push(start_verties + 3);
				indexes.push(start_verties + 1);
				indexes.push(start_verties + 5);
				indexes.push(start_verties + 3);
			}
			if !self.voxels.contains_key(&(pos + IVec3::Z)) {
				indexes.push(start_verties + 7); // +z
				indexes.push(start_verties + 6);
				indexes.push(start_verties + 5);
				indexes.push(start_verties + 6);
				indexes.push(start_verties + 3);
				indexes.push(start_verties + 5);
			}
			if !self.voxels.contains_key(&(pos - IVec3::Z)) {
				indexes.push(start_verties + 0); // -z
				indexes.push(start_verties + 2);
				indexes.push(start_verties + 1);
				indexes.push(start_verties + 2);
				indexes.push(start_verties + 4);
				indexes.push(start_verties + 1);
			}
		}
		let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label: Some("Vertex Buffer"),
			contents: bytemuck::cast_slice(&verties),
			usage: wgpu::BufferUsages::VERTEX,
		});
		let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label: Some("Index Buffer"),
			contents: bytemuck::cast_slice(&indexes),
			usage: wgpu::BufferUsages::INDEX,
		});


		let matrix_buffer = matrix::MatrixUniform::get_buffer(device, 0);
		Some(mesh::Mesh {
			vertex_buffer,
			index_buffer,
			num_elements: indexes.len() as u32,
			matrix_buffer: matrix_buffer.0,
			matrix_bind_group: matrix_buffer.1,
		})
	}
}
