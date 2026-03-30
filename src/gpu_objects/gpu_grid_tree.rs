// use wgpu::{Device, util::DeviceExt};

use crate::grid_tree;


#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuGridTeeNode {
	child_count: u16,
	parent: u16, // last bit is 1 if its none
	pos_x: u16,
	pos_y: u16,
	pos_z: u16,
	scale: u16,
	contents: [u32; grid_tree::SIZE_USIZE_CUBED], // if last bit is 1 then it points to a child else a id in the pallet
}

pub struct GpuGrid {
	pub buffer: wgpu::Buffer,
	pub bind_group: wgpu::BindGroup,
	pub bind_group_layout: wgpu::BindGroupLayout,
}

// impl GpuGrid {
// 	pub fn from_bvh(device: &Device, grid_tree: &grid_tree::GridTree<u32>) -> Self {
// 		let (nodes, _items) = grid_tree.get_internals();
// 		let mut data: Vec<u8> = Vec::with_capacity(nodes.len() * size_of::<GpuGridTeeNode>()/* + items.len() * size_of::<u16>()*/);
// 		for node in nodes {
// 			match node.sub_nodes {
// 				grid_tree::ChildCell::Data { sub1, sub2 } => {
// 					data.extend_from_slice(bytemuck::cast_slice(&[GpuGridTeeNode {
// 						min_corner: node.min_corner.to_array(),
// 						max_corner: node.max_corner.to_array(),
// 						data_1: sub1,
// 						data_2: sub2,
// 						is_leaf: 0,
// 					}]));
// 				},
// 				grid_tree::ChildCell::Node { start, count } => {
// 					data.extend_from_slice(bytemuck::cast_slice(&[GpuGridTeeNode {
// 						min_corner: node.min_corner.to_array(),
// 						max_corner: node.max_corner.to_array(),
// 						data_1: start,
// 						data_2: count,
// 						is_leaf: 1,
// 					}]));
// 				},
// 				grid_tree::ChildCell::None: {

// 				}
// 			}
// 		}
// 		// for item in items {
// 		// 	data.extend_from_slice(bytemuck::cast_slice(&[0]));
// 		// }
// 		let bind_group_layout = Self::bind_group_layout(device);
// 		let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
// 			label: Some("bvh_buffer"),
// 			contents: &data,
// 			usage: wgpu::BufferUsages::STORAGE,
// 		});
// 		let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
// 			layout: &bind_group_layout,
// 			entries: &[wgpu::BindGroupEntry {
// 				binding: 0,
// 				resource:  wgpu::BindingResource::Buffer(wgpu::BufferBinding {
// 					buffer: &buffer,
// 					offset: 0,
// 					size: None,
// 				}),
// 			}],
// 			label: Some("bvh_bind_group"),
// 		});
// 		Self {
// 			buffer,
// 			bind_group,
// 			bind_group_layout,
// 		}
// 	}

// 	pub fn bind_group_layout(device: &Device) -> wgpu::BindGroupLayout {
// 		device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
// 			entries: &[wgpu::BindGroupLayoutEntry {
// 				binding: 0,
// 				visibility: wgpu::ShaderStages::FRAGMENT,
// 				ty: wgpu::BindingType::Buffer {
// 					ty: wgpu::BufferBindingType::Storage { read_only: true },
// 					has_dynamic_offset: false,
// 					min_binding_size: None,
// 				},
// 				count: None,
// 			}],
// 			label: Some("bvh_bind_group_layout"),
// 		})
// 	}
// }
