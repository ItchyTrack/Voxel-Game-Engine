use std::{collections::HashMap, fmt::Debug, hash::Hash};

use bevy::math::Vec3;
use bevy::transform::components::Transform;
use bevy::render::renderer::WgpuWrapper;
use voxel_data::bvh;
use wgpu::{util::DeviceExt, Device};

type GpuBuffer = WgpuWrapper<wgpu::Buffer>;
type GpuBindGroup = WgpuWrapper<wgpu::BindGroup>;
type GpuBindGroupLayout = WgpuWrapper<wgpu::BindGroupLayout>;

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct GpuBVHNode {
	c0_min_and_ref:   [f32; 4],
	c0_max_and_ref2:  [f32; 4],
	c1_min_and_flags: [f32; 4],
	c1_max_pad:       [f32; 4],
}

impl GpuBVHNode {
	fn internal(c0_min: Vec3, c0_max: Vec3, c0_ref: u32, c1_min: Vec3, c1_max: Vec3, c1_ref: u32) -> Self {
		let [c0_min_x, c0_min_y, c0_min_z] = c0_min.to_array();
		let [c0_max_x, c0_max_y, c0_max_z] = c0_max.to_array();
		let [c1_min_x, c1_min_y, c1_min_z] = c1_min.to_array();
		let [c1_max_x, c1_max_y, c1_max_z] = c1_max.to_array();
		Self {
			c0_min_and_ref:   [c0_min_x, c0_min_y, c0_min_z, f32::from_bits(c0_ref)],
			c0_max_and_ref2:  [c0_max_x, c0_max_y, c0_max_z, f32::from_bits(c1_ref)],
			c1_min_and_flags: [c1_min_x, c1_min_y, c1_min_z, f32::from_bits(0)],
			c1_max_pad:       [c1_max_x, c1_max_y, c1_max_z, 0.0],
		}
	}

	fn leaf(min: Vec3, max: Vec3, base: u32, count: u32) -> Self {
		let [min_x, min_y, min_z] = min.to_array();
		let [max_x, max_y, max_z] = max.to_array();
		Self {
			c0_min_and_ref:   [min_x, min_y, min_z, f32::from_bits(base)],
			c0_max_and_ref2:  [max_x, max_y, max_z, f32::from_bits(count)],
			c1_min_and_flags: [0.0, 0.0, 0.0, f32::from_bits(1)],
			c1_max_pad:       [0.0; 4],
		}
	}

	fn sentinel(world_min: Vec3, world_max: Vec3, root_ref: u32) -> Self {
		let [min_x, min_y, min_z] = world_min.to_array();
		let [max_x, max_y, max_z] = world_max.to_array();
		Self {
			c0_min_and_ref:   [min_x, min_y, min_z, f32::from_bits(root_ref)],
			c0_max_and_ref2:  [max_x, max_y, max_z, 0.0],
			c1_min_and_flags: [0.0; 4],
			c1_max_pad:       [0.0; 4],
		}
	}
}

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct GpuBVHItem {
	min_corner:   [f32; 3],
	aabb_size:    [u8; 3],
	_padding:     u8,
	item_index:   u32,
	item_index_2: u32,
	pos:          [f32; 3],
	quat:         [f32; 4],
	scale:        f32,
}

pub struct GpuBvh<Id> {
	pub bvh_buffer: GpuBuffer,
	pub items_buffer: GpuBuffer,
	pub bind_group: GpuBindGroup,
	pub item_direction_mask_buffer: GpuBuffer,
	pub item_direction_mask_staging_buffer: GpuBuffer,
	pub item_count: usize,
	pub bind_group_layout: GpuBindGroupLayout,
	pub item_ids: Vec<Id>,
}

impl<Id: Copy + Debug + PartialEq + Eq + Hash> GpuBvh<Id> {
	pub fn from_bvh(device: &Device, bvh: &bvh::BVH<Id>, gpu_grid_tree_id_to_id_transforms: &HashMap<Id, (u32, u32, Transform)>) -> Self {
		let (nodes, items) = bvh.internals();

		let mut gpu_nodes: Vec<GpuBVHNode> = Vec::with_capacity(nodes.len() + 1);
		if let Some(root) = nodes.first() {
			gpu_nodes.push(GpuBVHNode::sentinel(root.min_corner, root.max_corner, 1));
		} else {
			gpu_nodes.push(GpuBVHNode::sentinel(Vec3::ZERO, Vec3::ZERO, 0));
		}

		for node in nodes.iter() {
			let gpu_node = match node.sub_nodes {
				bvh::BVHInternal::SubNodes { sub1, sub2 } => {
					let c0 = &nodes[sub1 as usize];
					let c1 = &nodes[sub2 as usize];
					GpuBVHNode::internal(c0.min_corner, c0.max_corner, sub1 as u32 + 1, c1.min_corner, c1.max_corner, sub2 as u32 + 1)
				}
				bvh::BVHInternal::Leaf { start, count } => GpuBVHNode::leaf(node.min_corner, node.max_corner, start as u32, count as u32),
			};
			gpu_nodes.push(gpu_node);
		}

		let bvh_buffer = WgpuWrapper::new(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label: Some("bvh_buffer"),
			contents: bytemuck::cast_slice(&gpu_nodes),
			usage: wgpu::BufferUsages::STORAGE,
		}));

		let mut item_data: Vec<u8> = Vec::with_capacity(items.len() * size_of::<GpuBVHItem>());
		let mut item_ids: Vec<_> = Vec::with_capacity(items.len());
		for item in items {
			if let Some((tree_offset, voxels_offset, transform)) = gpu_grid_tree_id_to_id_transforms.get(&item.0) {
				let scale = transform.scale.x.max(f32::MIN_POSITIVE);
				item_data.extend_from_slice(bytemuck::bytes_of(&GpuBVHItem {
					min_corner:   item.1.0.to_array(),
					aabb_size:    ((item.1.1 - item.1.0) / scale).ceil().as_u8vec3().to_array(),
					_padding:     0,
					item_index:   *tree_offset,
					item_index_2: *voxels_offset,
					pos:          transform.translation.to_array(),
					quat:         transform.rotation.to_array(),
					scale:        transform.scale.x,
				}));
				item_ids.push(item.0);
			} else {
				log::warn!("BVH item missing from id_to_offsets map; inserting zero node");
				item_data.resize(item_data.len() + size_of::<GpuBVHItem>(), 0);
			}
		}

		if item_data.is_empty() {
			item_data.extend_from_slice(&[0; size_of::<GpuBVHItem>()]);
		}

		let items_buffer = WgpuWrapper::new(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label: Some("bvh_items_buffer"),
			contents: &item_data,
			usage: wgpu::BufferUsages::STORAGE,
		}));

		let item_count = items.len().max(1);
		let direction_mask_size = (item_count.div_ceil(4) * std::mem::size_of::<u32>()) as u64;

		let item_direction_mask_buffer = WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
			label:              Some("bvh_item_direction_mask_buffer"),
			size:               direction_mask_size,
			usage:              wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
			mapped_at_creation: false,
		}));
		let item_direction_mask_staging_buffer = WgpuWrapper::new(device.create_buffer(&wgpu::BufferDescriptor {
			label:              Some("bvh_item_direction_mask_staging_buffer"),
			size:               direction_mask_size,
			usage:              wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		}));

		let bind_group_layout = Self::bind_group_layout(device);
		let bind_group = WgpuWrapper::new(device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout: &bind_group_layout,
			entries: &[
				wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding { buffer: &bvh_buffer, offset: 0, size: None }) },
				wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding { buffer: &items_buffer, offset: 0, size: None }) },
				wgpu::BindGroupEntry { binding: 2, resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding { buffer: &item_direction_mask_buffer, offset: 0, size: None }) },
			],
			label: Some("bvh_bind_group"),
		}));

		Self { bvh_buffer, items_buffer, item_direction_mask_buffer, item_direction_mask_staging_buffer, item_count, bind_group, bind_group_layout, item_ids }
	}

	pub fn bind_group_layout(device: &Device) -> GpuBindGroupLayout {
		WgpuWrapper::new(device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[
				wgpu::BindGroupLayoutEntry {
					binding:    0,
					visibility: wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::COMPUTE,
					ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Storage { read_only: true }, has_dynamic_offset: false, min_binding_size: None },
					count: None,
				},
				wgpu::BindGroupLayoutEntry {
					binding:    1,
					visibility: wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::COMPUTE,
					ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Storage { read_only: true }, has_dynamic_offset: false, min_binding_size: None },
					count: None,
				},
				wgpu::BindGroupLayoutEntry {
					binding:    2,
					visibility: wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::COMPUTE,
					ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Storage { read_only: false }, has_dynamic_offset: false, min_binding_size: None },
					count: None,
				},
			],
			label: Some("bvh_bind_group_layout"),
		}))
	}
}
