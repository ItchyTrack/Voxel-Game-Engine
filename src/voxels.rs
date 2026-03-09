use glam::{IVec3, Vec3};
use wgpu::util::DeviceExt;
use std::cell::Cell;

use crate::gpu_objects::mesh;
use crate::gpu_objects::matrix;
use crate::grid_tree::GridTree;

#[derive(Clone, Copy, Debug)]
pub struct Voxel {
	pub color: [f32; 4],
	pub mass: f32,
}

pub struct Voxels {
	voxels: GridTree<3, Voxel>,
	bounding_box: Cell<Option<(IVec3, IVec3)>>,
	bounding_box_dirty: Cell<bool>,
}

impl Voxels {
	pub fn new() -> Self {
		Self { voxels: GridTree::new(), bounding_box: Cell::new(None), bounding_box_dirty: Cell::new(false) }
	}
	pub fn add_voxel(&mut self, pos: IVec3, voxel: Voxel) -> Option<Voxel> {
		match self.bounding_box.get() {
			Some((min, max)) => {
				self.bounding_box.set(Some((min.min(pos), max.max(pos))));
			},
			None => {
				self.bounding_box.set(Some((pos, pos)));
			}
		}
		self.voxels.insert(pos, voxel)
	}

	pub fn remove_voxel(&mut self, pos: &IVec3) -> Option<Voxel> {
		self.bounding_box_dirty.set(true);
		self.voxels.remove(pos)
	}

	pub fn get_voxel(&self, pos: IVec3) -> Option<&Voxel> { self.voxels.get(&pos) }
	pub fn get_voxels(&self) -> &GridTree<3, Voxel> { &self.voxels }

	pub fn get_bounding_box(&self) -> Option<(IVec3, IVec3)> {
		if self.bounding_box_dirty.get() {
			self.bounding_box_dirty.set(false);
			self.bounding_box.set(self.voxels.iter().fold(None, |bb, (p, _)| {
				match bb {
					Some((min, max)) => Some((min.min(p), max.max(p))),
					None => Some((p, p))
				}
			}));
		}
		self.bounding_box.get()
	}
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
