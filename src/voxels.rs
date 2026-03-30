use glam::I16Vec3;
use tracy_client::span;
use std::cell::Cell;
use bimap::BiHashMap;

use crate::grid_tree::GridTree;

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
	voxels: GridTree,
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
	pub fn add_voxel(&mut self, pos: I16Vec3, voxel: Voxel) -> Option<Voxel> {
		match self.bounding_box.get() {
			Some((min, max)) => {
				self.bounding_box.set(Some((min.min(pos), max.max(pos))));
			},
			None => {
				self.bounding_box.set(Some((pos, pos)));
			}
		}
		let out = self.voxels.insert(&pos, self.voxel_palette.get_palette_id(&voxel))?;
		self.voxel_palette.get_voxel(out).cloned()
	}

	pub fn remove_voxel(&mut self, pos: &I16Vec3) -> Option<Voxel> {
		self.bounding_box_dirty.set(true);
		self.voxel_palette.get_voxel(self.voxels.remove(pos)?).cloned()
	}

	pub fn get_voxel(&self, pos: I16Vec3) -> Option<&Voxel> {
			self.voxel_palette.get_voxel(self.voxels.get(&pos)?)
		}
	pub fn get_voxels(&self) -> &GridTree { &self.voxels }

	pub fn get_bounding_box(&self) -> Option<(I16Vec3, I16Vec3)> {
		if self.bounding_box_dirty.get() {
			let _zone = span!("rebuild voxel bounding box");
			self.bounding_box_dirty.set(false);
			self.bounding_box.set(self.voxels.iter().fold(None, |bb, (p, size, _)| {
				match bb {
					Some((min, max)) => Some((min.min(p), max.max(p + size as i16 - 1))),
					None => Some((p, p + size as i16 - 1))
				}
			}));
		}
		self.bounding_box.get()
	}
}

// impl mesh::GetMesh for Voxels {
// 	fn get_mesh_buffers(&self) -> Option<Vec<mesh::MeshVertex>> {
// 		if self.voxels.is_empty() {
// 			return None;
// 		}
// 		let mut vertices: Vec<mesh::MeshVertex> = vec![];
// 		// let mut indexes: Vec<u32> = vec![];
// 		for (pos, size, voxel_id) in &self.voxels {
// 			let voxel = self.voxel_palette.get_voxel(voxel_id).unwrap();
// 			if !self.voxels.is_area_filled(&(pos + I16Vec3::X * size as i16), &U16Vec3::new(1, size, size)) {
// 				vertices.push(MeshVertex{ // +x
// 					position: pos.as_i8vec3().to_array(),
// 					orientation_size: 0 + size as u8 * 8,
// 					color: voxel.color,
// 				});
// 			}
// 			if !self.voxels.is_area_filled(&(pos - I16Vec3::X), &U16Vec3::new(1, size, size)) {
// 				vertices.push(MeshVertex{ // -x
// 					position: pos.as_i8vec3().to_array(),
// 					orientation_size: 1 + size as u8 * 8,
// 					color: voxel.color,
// 				});
// 			}
// 			if !self.voxels.is_area_filled(&(pos + I16Vec3::Y * size as i16), &U16Vec3::new(size, 1, size)) {
// 				vertices.push(MeshVertex{ // +y
// 					position: pos.as_i8vec3().to_array(),
// 					orientation_size: 2 + size as u8 * 8,
// 					color: voxel.color,
// 				});
// 			}
// 			if !self.voxels.is_area_filled(&(pos - I16Vec3::Y), &U16Vec3::new(size, 1, size)) {
// 				vertices.push(MeshVertex{ // -y
// 					position: pos.as_i8vec3().to_array(),
// 					orientation_size: 3 + size as u8 * 8,
// 					color: voxel.color,
// 				});
// 			}
// 			if !self.voxels.is_area_filled(&(pos + I16Vec3::Z * size as i16), &U16Vec3::new(size, size, 1)) {
// 				vertices.push(MeshVertex{ // +z
// 					position: pos.as_i8vec3().to_array(),
// 					orientation_size: 4 + size as u8 * 8,
// 					color: voxel.color,
// 				});
// 			}
// 			if !self.voxels.is_area_filled(&(pos - I16Vec3::Z), &U16Vec3::new(size, size, 1)) {
// 				vertices.push(MeshVertex{ // -z
// 					position: pos.as_i8vec3().to_array(),
// 					orientation_size: 5 + size as u8 * 8,
// 					color: voxel.color,
// 				});
// 			}
// 		}
// 		Some(vertices)
// 	}
// 	// fn get_mesh(&self, device: &wgpu::Device) -> Option<mesh::Mesh> {
// 	// 	let vertices = self.get_mesh_buffers()?;
// 	// 	let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
// 	// 		label: Some("Vertex Buffer"),
// 	// 		contents: bytemuck::cast_slice(&vertices),
// 	// 		usage: wgpu::BufferUsages::STORAGE,
// 	// 	});
// 	// 	let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
// 	// 		label: Some("Index Buffer"),
// 	// 		contents: bytemuck::cast_slice(&indexes),
// 	// 		usage: wgpu::BufferUsages::INDEX,
// 	// 	});

// 	// 	let matrix_buffer = matrix::MatrixUniform::get_buffer(device, 0);
// 	// 	Some(mesh::Mesh {
// 	// 		vertex_buffer,
// 	// 		index_buffer,
// 	// 		num_elements: indexes.len() as u32,
// 	// 		matrix_buffer: matrix_buffer.0,
// 	// 		matrix_bind_group: matrix_buffer.1,
// 	// 	})
// 	// }
// }
