use std::collections::BTreeMap;

use glam::Mat4;
use num::Integer;
use tracy_client::span;
use wgpu::{Device, Queue, RenderPass};

use crate::{gpu_objects::{matrix::MatrixUniform, mesh::MeshVertex, packed_buffer::{PackedBufferGroup, PackedBufferGroupId}}, pose::Pose};

struct PackedMeshInfo {
	pub vertex_data_count: u32,
}

pub struct PackedMeshBuffer {
	packed_buffer: PackedBufferGroup,
	buffer_infos: BTreeMap<PackedBufferGroupId, PackedMeshInfo>,
	mat_bind_group_layout: wgpu::BindGroupLayout,
	mat_buffer: wgpu::Buffer,
	mat_bind_group: wgpu::BindGroup
}

impl PackedMeshBuffer {
	pub fn new(device: &wgpu::Device, sinlge_buffer_size: u32) -> Result<Self, &'static str> {
		let mat_bind_group_layout = MatrixUniform::get_dynamic_offset_bind_group_layout(device, 0);
		let mat_buffer = device.create_buffer(&wgpu::BufferDescriptor {
				label: Some("Matrix Buffer"),
				size: std::mem::size_of::<MatrixUniform>() as u64,
				usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
				mapped_at_creation: false,
			});
		let mat_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
				layout: &mat_bind_group_layout,
				entries: &[wgpu::BindGroupEntry {
					binding: 0,
					resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
						buffer: &mat_buffer,
						offset: 0,
						size: Some(std::num::NonZeroU64::new(size_of::<MatrixUniform>() as u64).unwrap()),
					}),
				}],
				label: Some("matrix_bind_group"),
			});
		Ok(Self {
			packed_buffer: PackedBufferGroup::new(
				sinlge_buffer_size,
				(size_of::<MeshVertex>() as u32).lcm(&(size_of::<u32>() as u32)),
				wgpu::BufferUsages::STORAGE
			)?,
			buffer_infos: BTreeMap::new(),
			mat_bind_group_layout,
			mat_buffer,
			mat_bind_group,
		})
	}

	pub fn add_mesh(&mut self, device: &Device, queue: &Queue, vertex_data: &[MeshVertex]) -> Result<PackedBufferGroupId, &str> {
		let vertices_raw = bytemuck::cast_slice::<_, u8>(vertex_data);
		// let indices_raw = bytemuck::cast_slice::<_, u8>(indices);
		// let mesh = [
		// 		vertices_raw,
		// 		&vec![0; vertices_raw.len().next_multiple_of((wgpu::VERTEX_ALIGNMENT as usize).lcm(&size_of::<u32>())) - vertices_raw.len()],
		// 		indices_raw
		// 	].concat();
		let id = self.packed_buffer.add_buffer(device, queue, &vertices_raw)?;
		self.buffer_infos.insert(id, PackedMeshInfo { vertex_data_count: vertex_data.len() as u32 });
		Ok(id)
	}

	pub fn remove_buffer(&mut self, id: PackedBufferGroupId) -> Result<(), &'static str> {
		self.packed_buffer.remove_buffer(id)?;
		self.buffer_infos.remove(&id);
		Ok(())
	}

	pub fn replace_buffer(&mut self, device: &Device, queue: &Queue, id: PackedBufferGroupId, vertex_data: &[MeshVertex]) -> Result<PackedBufferGroupId, &'static str> {
		let vertices_raw = bytemuck::cast_slice::<_, u8>(vertex_data);
		// let mesh = [
		// 		vertices_raw,
		// 		&vec![0; vertices_raw.len().next_multiple_of((wgpu::VERTEX_ALIGNMENT as usize).lcm(&size_of::<u32>())) - vertices_raw.len()],
		// 		indices_raw
		// 	].concat();
		let new_id = self.packed_buffer.replace_buffer(device, queue, id, &vertices_raw)?;
		if let Some(_) = self.buffer_infos.remove(&id) {
			self.buffer_infos.insert(new_id, PackedMeshInfo { vertex_data_count: vertex_data.len() as u32 });
		} else {
			println!("replace_buffer returned Ok but buffer_infos.remove failed");
		}
		Ok(new_id)
	}

	pub fn render(&mut self, device: &Device, queue: &Queue, render_pass: &mut RenderPass, camera_bind_group: &wgpu::BindGroup, to_render: &[(PackedBufferGroupId, Pose)]) {
		let _zone = span!("Render Packed Mesh Buffer");
		if to_render.is_empty() { return; }
		// bind camera once
		render_pass.set_bind_group(0, camera_bind_group, &[]);
		// MatrixUniform Buffer
		let needed_matrix_buffer_size = to_render.len() as u64 * (size_of::<MatrixUniform>() as u64).next_multiple_of(device.limits().min_uniform_buffer_offset_alignment as u64);
		if needed_matrix_buffer_size > self.mat_buffer.size() {
			self.mat_buffer = device.create_buffer(&wgpu::BufferDescriptor {
					label: Some("Matrix Buffer"),
					size: needed_matrix_buffer_size,
					usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
					mapped_at_creation: false,
				});
			self.mat_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
					layout: &self.mat_bind_group_layout,
					entries: &[wgpu::BindGroupEntry {
						binding: 0,
						resource:  wgpu::BindingResource::Buffer(wgpu::BufferBinding {
							buffer: &self.mat_buffer,
							offset: 0,
							size: Some(std::num::NonZeroU64::new(size_of::<MatrixUniform>() as u64).unwrap()),
						}),
					}],
					label: Some("matrix_bind_group"),
				});
		}
		let mut to_render = to_render.to_vec();
		to_render.sort_by(|a, b| a.0.buffer_index().cmp(&b.0.buffer_index()));
		let mat_alignment = size_of::<MatrixUniform>().next_multiple_of(device.limits().min_uniform_buffer_offset_alignment as usize);
		let mut mat_buffer_offset = 0;
		for chunk in to_render.chunk_by(|a, b| a.0.buffer_index() == b.0.buffer_index()) {
			if let Some(single_packed_buffer) = self.packed_buffer.get_packed_buffer(chunk.first().unwrap().0.buffer_index()) {
				let _zone = span!("Render Mesh Chunk");
				let mut mats: Vec<u8> = vec![];
				let held_buffers: Vec<_> = chunk.iter().filter_map(|(id, pose)| {
					let held_buffer = single_packed_buffer.get_held_buffer(id.internal_id())?;
					let buffer_info = self.buffer_infos.get(id)?;
					mats.extend_from_slice(bytemuck::cast_slice(&[MatrixUniform::from_mat4(&Mat4::from_rotation_translation(pose.rotation, pose.translation))]));
					mats.resize(mats.len().next_multiple_of(mat_alignment), 0);
					Some((held_buffer, buffer_info))
				}).collect();
				// write mats to burffer
				queue.write_buffer(&self.mat_buffer, mat_buffer_offset, &mats);
				// bind main buffer
				let vertex_data_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
					label: None,
					entries: &[wgpu::BindGroupLayoutEntry {
						binding: 0,
						visibility: wgpu::ShaderStages::VERTEX,
						ty: wgpu::BindingType::Buffer {
							ty: wgpu::BufferBindingType::Storage { read_only: true },
							has_dynamic_offset: false,
							min_binding_size: None,
						},
						count: None,
					}],
				});
				let vertex_data_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
					label: None,
					layout: &vertex_data_bind_group_layout,
					entries: &[wgpu::BindGroupEntry {
						binding: 0,
						resource: single_packed_buffer.get_buffer().as_entire_binding(),
					}],
				});
				render_pass.set_bind_group(2, &vertex_data_bind_group, &[]);
				// render_pass.set_vertex_buffer(0, single_packed_buffer.get_buffer().slice(..));
				// render_pass.set_index_buffer(single_packed_buffer.get_buffer().slice(..), wgpu::IndexFormat::Uint32);
				// render
				for (i, (held_buffer, packed_mesh_info)) in held_buffers.iter().enumerate() {
					assert!(held_buffer.offset().is_multiple_of((size_of::<MeshVertex>() as u32).lcm(&(size_of::<u32>() as u32))));
					let first_index = held_buffer.offset() as u32 / size_of::<MeshVertex>() as u32;
					// 	held_buffer.offset() as usize +
					// 	(packed_mesh_info.vertex_count as usize * size_of::<MeshVertex>()).next_multiple_of(*size_of::<u32>())
					// ) / size_of::<u32>();
					render_pass.set_bind_group(1, &self.mat_bind_group, &[mat_alignment as u32 * i as u32 + mat_buffer_offset as u32]);
					// render_pass.draw_indexed(first_index_index as u32..first_index_index as u32 + packed_mesh_info.index_count, (held_buffer.offset() as usize / size_of::<MeshVertex>()) as i32, 0..1);
					render_pass.draw(first_index * 6..(first_index + packed_mesh_info.vertex_data_count) * 6, 0..1);
				}
				mat_buffer_offset += mats.len() as u64;
			}
		}
	}
}
