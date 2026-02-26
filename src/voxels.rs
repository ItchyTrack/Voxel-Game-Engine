use crate::{MatrixUniform, get_matrix_buffer, mesh::{GetMesh, Mesh, MeshVertex}};
use wgpu::util::DeviceExt;
use std::collections::HashMap;

pub struct Voxel {
	pub color: [f32; 4],
}

pub struct Voxels {
    pub voxels: HashMap<[i32; 3], Voxel>,
}

impl Voxels {
    pub fn new() -> Self {
        Self { voxels: HashMap::new() }
    }
}

impl GetMesh for Voxels {
    fn get_mesh(&self, device: &wgpu::Device) -> Mesh {
		let mut verties: Vec<MeshVertex> = vec![];
		let mut indexes: Vec<u32> = vec![];
		for (pos, voxel) in &self.voxels {
			let fpos: [f32; 3] = [pos[0] as f32, pos[1] as f32, pos[2] as f32];
			let start_verties = verties.len() as u32;
			verties.push(MeshVertex { // 0
				position: fpos,
				color: voxel.color,
				normal: [0.0, 0.0, 0.0]
			});
			verties.push(MeshVertex { // 1
				position: [fpos[0] + 1.0, fpos[1], fpos[2]],
				color: voxel.color,
				normal: [0.0, 0.0, 0.0]
			});
			verties.push(MeshVertex { // 2
				position: [fpos[0], fpos[1] + 1.0, fpos[2]],
				color: voxel.color,
				normal: [0.0, 0.0, 0.0]
			});
			verties.push(MeshVertex { // 3
				position: [fpos[0], fpos[1], fpos[2] + 1.0],
				color: voxel.color,
				normal: [0.0, 0.0, 0.0]
			});
			verties.push(MeshVertex { // 4
				position: [fpos[0] + 1.0, fpos[1] + 1.0, fpos[2]],
				color: voxel.color,
				normal: [0.0, 0.0, 0.0]
			});
			verties.push(MeshVertex { // 5
				position: [fpos[0] + 1.0, fpos[1], fpos[2] + 1.0],
				color: voxel.color,
				normal: [0.0, 0.0, 0.0]
			});
			verties.push(MeshVertex { // 6
				position: [fpos[0], fpos[1] + 1.0, fpos[2] + 1.0],
				color: voxel.color,
				normal: [0.0, 0.0, 0.0]
			});
			verties.push(MeshVertex { // 7
				position: [fpos[0] + 1.0, fpos[1] + 1.0, fpos[2] + 1.0],
				color: voxel.color,
				normal: [0.0, 0.0, 0.0]
			});
			if !self.voxels.contains_key(&[pos[0] + 1, pos[1], pos[2]]) {
				indexes.push(start_verties + 5); // +x
				indexes.push(start_verties + 4);
				indexes.push(start_verties + 7);
				indexes.push(start_verties + 5);
				indexes.push(start_verties + 1);
				indexes.push(start_verties + 4);
			}
			if !self.voxels.contains_key(&[pos[0] - 1, pos[1], pos[2]]) {
				indexes.push(start_verties + 0); // -x
				indexes.push(start_verties + 3);
				indexes.push(start_verties + 2);
				indexes.push(start_verties + 2);
				indexes.push(start_verties + 3);
				indexes.push(start_verties + 6);
			}
			if !self.voxels.contains_key(&[pos[0], pos[1] + 1, pos[2]]) {
				indexes.push(start_verties + 7); // +y
				indexes.push(start_verties + 4);
				indexes.push(start_verties + 6);
				indexes.push(start_verties + 4);
				indexes.push(start_verties + 2);
				indexes.push(start_verties + 6);
			}
			if !self.voxels.contains_key(&[pos[0], pos[1] - 1, pos[2]]) {
				indexes.push(start_verties + 0); // -y
				indexes.push(start_verties + 1);
				indexes.push(start_verties + 3);
				indexes.push(start_verties + 1);
				indexes.push(start_verties + 5);
				indexes.push(start_verties + 3);
			}
			if !self.voxels.contains_key(&[pos[0], pos[1], pos[2] + 1]) {
				indexes.push(start_verties + 7); // +z
				indexes.push(start_verties + 6);
				indexes.push(start_verties + 5);
				indexes.push(start_verties + 6);
				indexes.push(start_verties + 3);
				indexes.push(start_verties + 5);
			}
			if !self.voxels.contains_key(&[pos[0], pos[1], pos[2] - 1]) {
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


		let matrix_buffer = get_matrix_buffer(device);
		Mesh {
			vertex_buffer,
			index_buffer,
			num_elements: indexes.len() as u32,
			matrix_buffer: matrix_buffer.0,
			matrix_bind_group: matrix_buffer.1,
		}
	}
}
