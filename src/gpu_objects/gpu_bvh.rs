use glam::IVec3;
use wgpu::{Device, util::DeviceExt};

use crate::physics::bvh;

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct GpuBVHNode {
	min_corner: [f32; 3],
	max_corner: [f32; 3],
	data_1: u16, // start if is_leaf else sub1
	data_2: u16, // count if is_leaf else sub2
	is_leaf: u32,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct GpuBVHItem {
	min_corner: [f32; 3],
	max_corner: [f32; 3],
	item_index: u32,
	padding: u32
}

pub struct GpuBvh {
	pub bvh_buffer: wgpu::Buffer,
	pub items_buffer: wgpu::Buffer,
	pub bind_group: wgpu::BindGroup,
	pub bind_group_layout: wgpu::BindGroupLayout,
}

impl GpuBvh {
	pub fn from_bvh(device: &Device, bvh: &bvh::BVH<(u32, u32, IVec3)>) -> Self {
		let (nodes, items) = bvh.get_internals();
		let mut bvh_data: Vec<u8> = Vec::with_capacity(nodes.len() * size_of::<GpuBVHNode>());
		for node in nodes {
			match node.sub_nodes {
				bvh::BVHInternal::SubNodes { sub1, sub2 } => {
					bvh_data.extend_from_slice(bytemuck::bytes_of(&GpuBVHNode {
						min_corner: node.min_corner.to_array(),
						max_corner: node.max_corner.to_array(),
						data_1: sub1,
						data_2: sub2,
						is_leaf: 0,
					}));
				},
				bvh::BVHInternal::Leaf { start, count } => {
					bvh_data.extend_from_slice(bytemuck::bytes_of(&GpuBVHNode {
						min_corner: node.min_corner.to_array(),
						max_corner: node.max_corner.to_array(),
						data_1: start,
						data_2: count,
						is_leaf: 1,
					}));
				},
			}
		}
		let bvh_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label: Some("bvh_buffer"),
			contents: &bvh_data,
			usage: wgpu::BufferUsages::STORAGE,
		});
		let mut item_data: Vec<u8> = Vec::with_capacity(items.len() * size_of::<u16>());
		for item in items {
			item_data.extend_from_slice(bytemuck::bytes_of(&GpuBVHItem {
				min_corner: item.1.0.to_array(),
				max_corner: item.1.1.to_array(),
				item_index: {
					let mut index = item.0.0;
					index = index ^ (item.0.1 + 0x9e3779b9 + (index << 6) + (index >> 2));
					index = index ^ (item.0.2.x as u32 + 0x9e3779b9 + (index << 6) + (index >> 2));
					index = index ^ (item.0.2.y as u32 + 0x9e3779b9 + (index << 6) + (index >> 2));
					index = index ^ (item.0.2.z as u32 + 0x9e3779b9 + (index << 6) + (index >> 2));
					index
				},
				// item_index: item.0.0 as u32 + (item.0.1 as u32) << 8 + item.0.2.x << 16 + item.0.2.y << 16 + item.0.2.z << 16,
				padding: 0,
			}));
		}
		let items_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label: Some("bvh_buffer"),
			contents: &item_data,
			usage: wgpu::BufferUsages::STORAGE,
		});
		let bind_group_layout = Self::bind_group_layout(device);
		let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &bind_group_layout,
			entries: &[wgpu::BindGroupEntry {
				binding: 0,
				resource:  wgpu::BindingResource::Buffer(wgpu::BufferBinding {
					buffer: &bvh_buffer,
					offset: 0,
					size: None,
				}),
			}, wgpu::BindGroupEntry {
				binding: 1,
				resource:  wgpu::BindingResource::Buffer(wgpu::BufferBinding {
					buffer: &items_buffer,
					offset: 0,
					size: None,
				}),
			}],
			label: Some("bvh_bind_group"),
		});
		Self {
			bvh_buffer,
			items_buffer,
			bind_group,
			bind_group_layout,
		}
	}

	pub fn bind_group_layout(device: &Device) -> wgpu::BindGroupLayout {
		device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[wgpu::BindGroupLayoutEntry {
				binding: 0,
				visibility: wgpu::ShaderStages::FRAGMENT,
				ty: wgpu::BindingType::Buffer {
					ty: wgpu::BufferBindingType::Storage { read_only: true },
					has_dynamic_offset: false,
					min_binding_size: None,
				},
				count: None,
			}, wgpu::BindGroupLayoutEntry {
				binding: 1,
				visibility: wgpu::ShaderStages::FRAGMENT,
				ty: wgpu::BindingType::Buffer {
					ty: wgpu::BufferBindingType::Storage { read_only: true },
					has_dynamic_offset: false,
					min_binding_size: None,
				},
				count: None,
			}],
			label: Some("bvh_bind_group_layout"),
		})
	}
}
