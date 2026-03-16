use glam::{U16Vec3};
use glam::{I16Vec3, Vec3};
use wgpu::util::DeviceExt;
use std::cell::Cell;
use bimap::BiHashMap;

use crate::gpu_objects::mesh;
use crate::gpu_objects::matrix;
use crate::grid_tree::GridTree;
use crate::pose::Pose;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Voxel {
	pub color: [u8; 4],
	pub mass: u32,
}

#[derive(Clone, Debug)]
pub struct VoxelPalette {
	palette: BiHashMap<u16, Voxel>,
	next_id: u16,
}

impl VoxelPalette {
	pub fn new() -> Self {
		Self {
			palette: BiHashMap::new(),
			next_id: 0,
		}
	}
	pub fn get_palette_id(&mut self, voxel: &Voxel) -> u16 {
		if let Err(_) = self.palette.insert_no_overwrite(self.next_id, *voxel) {
			let id = *self.palette.get_by_right(voxel).unwrap();
			assert!(self.next_id != id);
			return id;
		}
		let id = self.next_id;
		self.next_id += 1;
		id
	}
	pub fn get_voxel(&self, id: u16) -> Option<&Voxel> {
		self.palette.get_by_left(&id)
	}
}

pub struct Voxels {
	voxels: GridTree<u16>,
	voxel_palette: VoxelPalette,
	bounding_box: Cell<Option<(I16Vec3, I16Vec3)>>,
	bounding_box_dirty: Cell<bool>,
}

impl Voxels {
	pub fn new() -> Self {
		Self {
			voxels: GridTree::new(),
			voxel_palette: VoxelPalette::new(),
			bounding_box: Cell::new(None),
			bounding_box_dirty: Cell::new(false)
		}
	}
	pub fn add_voxel(&mut self, pos: I16Vec3, voxel: Voxel) -> Option<&Voxel> {
		match self.bounding_box.get() {
			Some((min, max)) => {
				self.bounding_box.set(Some((min.min(pos), max.max(pos))));
			},
			None => {
				self.bounding_box.set(Some((pos, pos)));
			}
		}
		let out = self.voxels.insert(pos, self.voxel_palette.get_palette_id(&voxel))?;
		self.voxel_palette.get_voxel(out)
	}

	pub fn remove_voxel(&mut self, pos: &I16Vec3) -> Option<&Voxel> {
		self.bounding_box_dirty.set(true);
		self.voxel_palette.get_voxel(self.voxels.remove(pos)?)
	}

	pub fn get_voxel(&self, pos: I16Vec3) -> Option<&Voxel> {
			self.voxel_palette.get_voxel(*self.voxels.get(&pos)?)
		}
	pub fn get_voxels(&self) -> &GridTree<u16> { &self.voxels }

	pub fn get_bounding_box(&self) -> Option<(I16Vec3, I16Vec3)> {
		if self.bounding_box_dirty.get() {
			self.bounding_box_dirty.set(false);
			self.bounding_box.set(self.voxels.iter().fold(None, |bb, (p, size, _)| {
				match bb {
					Some((min, max)) => Some((min.min(p), max.max(p + size as i16))),
					None => Some((p, p + size as i16))
				}
			}));
		}
		self.bounding_box.get()
	}

	pub fn render_debug(&self, pose: &Pose) {
		self.voxels.render_debug(pose);
	}
}

impl mesh::GetMesh for Voxels {
	fn get_mesh(&self, device: &wgpu::Device) -> Option<mesh::Mesh> {
		if self.voxels.is_empty() {
			return None;
		}
		let mut verties: Vec<mesh::MeshVertex> = vec![];
		let mut indexes: Vec<u32> = vec![];
		for (pos, size, voxel_id) in &self.voxels {
			let voxel = self.voxel_palette.get_voxel(*voxel_id).unwrap();
			let fpos: Vec3 = pos.as_vec3();
			// let start_verties = verties.len() as u32;
			let f32_color = [voxel.color[0] as f32 / 255.0, voxel.color[1] as f32 / 255.0, voxel.color[2] as f32 / 255.0, voxel.color[3] as f32 / 255.0];
			let mut verties_index = [8; 8];
			let mut get_index = |id: u8| -> u32 {
				let val = &mut verties_index[id as usize];
				if *val == 8 {
					*val = verties.len();
					match id {
						0 => {verties.push(mesh::MeshVertex { // 0
							position: fpos.to_array(),
							color: f32_color,
							normal: [0.0, 0.0, 0.0]
						});}
						1 => {verties.push(mesh::MeshVertex { // 1
							position: (fpos + Vec3::new(1.0, 0.0, 0.0) * size as f32).to_array(),
							color: f32_color,
							normal: [0.0, 0.0, 0.0]
						});}
						2 => {verties.push(mesh::MeshVertex { // 2
							position: (fpos + Vec3::new(0.0, 1.0, 0.0) * size as f32).to_array(),
							color: f32_color,
							normal: [0.0, 0.0, 0.0]
						});}
						3 => {verties.push(mesh::MeshVertex { // 3
							position: (fpos + Vec3::new(0.0, 0.0, 1.0) * size as f32).to_array(),
							color: f32_color,
							normal: [0.0, 0.0, 0.0]
						});}
						4 => {verties.push(mesh::MeshVertex { // 4
							position: (fpos + Vec3::new(1.0, 1.0, 0.0) * size as f32).to_array(),
							color: f32_color,
							normal: [0.0, 0.0, 0.0]
						});}
						5 => {verties.push(mesh::MeshVertex { // 5
							position: (fpos + Vec3::new(1.0, 0.0, 1.0) * size as f32).to_array(),
							color: f32_color,
							normal: [0.0, 0.0, 0.0]
						});}
						6 => {verties.push(mesh::MeshVertex { // 6
							position: (fpos + Vec3::new(0.0, 1.0, 1.0) * size as f32).to_array(),
							color: f32_color,
							normal: [0.0, 0.0, 0.0]
						});}
						7 => {verties.push(mesh::MeshVertex { // 7
							position: (fpos + Vec3::new(1.0, 1.0, 1.0) * size as f32).to_array(),
							color: f32_color,
							normal: [0.0, 0.0, 0.0]
						});}
						_ => unreachable!()
					}
				}
				return *val as u32;
			};
			if !self.voxels.is_area_filled(&(pos + I16Vec3::X * size as i16), &U16Vec3::new(1, size, size)) {
				indexes.push(get_index(5)); // +x
				indexes.push(get_index(4));
				indexes.push(get_index(7));
				indexes.push(get_index(5));
				indexes.push(get_index(1));
				indexes.push(get_index(4));
			}
			if !self.voxels.is_area_filled(&(pos - I16Vec3::X), &U16Vec3::new(1, size, size)) {
				indexes.push(get_index(0)); // -x
				indexes.push(get_index(3));
				indexes.push(get_index(2));
				indexes.push(get_index(2));
				indexes.push(get_index(3));
				indexes.push(get_index(6));
			}
			if !self.voxels.is_area_filled(&(pos + I16Vec3::Y * size as i16), &U16Vec3::new(size, 1, size)) {
				indexes.push(get_index(7)); // +y
				indexes.push(get_index(4));
				indexes.push(get_index(6));
				indexes.push(get_index(4));
				indexes.push(get_index(2));
				indexes.push(get_index(6));
			}
			if !self.voxels.is_area_filled(&(pos - I16Vec3::Y), &U16Vec3::new(size, 1, size)) {
				indexes.push(get_index(0)); // -y
				indexes.push(get_index(1));
				indexes.push(get_index(3));
				indexes.push(get_index(1));
				indexes.push(get_index(5));
				indexes.push(get_index(3));
			}
			if !self.voxels.is_area_filled(&(pos + I16Vec3::Z * size as i16), &U16Vec3::new(size, size, 1)) {
				indexes.push(get_index(7)); // +z
				indexes.push(get_index(6));
				indexes.push(get_index(5));
				indexes.push(get_index(6));
				indexes.push(get_index(3));
				indexes.push(get_index(5));
			}
			if !self.voxels.is_area_filled(&(pos - I16Vec3::Z), &U16Vec3::new(size, size, 1)) {
				indexes.push(get_index(0)); // -z
				indexes.push(get_index(2));
				indexes.push(get_index(1));
				indexes.push(get_index(2));
				indexes.push(get_index(4));
				indexes.push(get_index(1));
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
