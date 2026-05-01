use std::collections::HashMap;

use glam::Vec3;
use wgpu::{Device, util::DeviceExt};

use crate::{world::{physics_solver::bvh, physics_body::PhysicsBodyId, grid::GridId, grid::SubGridId}, pose::Pose};


// -- GPU node layout -----------------------------------------------------------
//
// Each GpuBVHNode is exactly 64 bytes — one cache line.
//
// The key optimisation vs the previous layout: for an internal node, both
// children's AABBs are stored *inside the parent*. The shader reads one node
// and immediately has both AABBs to test, instead of needing three node reads
// (parent + child 0 + child 1) to make progress. This cuts BVH memory
// bandwidth for internal nodes by 3*.
//
// Refs and flags are bit-cast into the .w component of the vec4 fields so
// that every field stays naturally 16-byte aligned for GPU consumption.
//
// +- internal node --------------------------------------------------------+
// │  c0_min_and_ref  .xyz = child-0 AABB min,  .w = bits(child-0 idx)      │
// │  c0_max_and_ref2 .xyz = child-0 AABB max,  .w = bits(child-1 idx)      │
// │  c1_min_and_flags.xyz = child-1 AABB min,  .w = bits(flags) = 0        │
// │  c1_max_pad      .xyz = child-1 AABB max,  .w = 0.0                    │
// +------------------------------------------------------------------------+
// +- leaf node ------------------------------------------------------------+
// │  c0_min_and_ref  .xyz = own AABB min,      .w = bits(item base)        │
// │  c0_max_and_ref2 .xyz = own AABB max,      .w = bits(item count)       │
// │  c1_min_and_flags       zeroed,            .w = bits(flags) = 1        │
// │  c1_max_pad             zeroed                                         │
// +------------------------------------------------------------------------+
//
// GPU node array layout:
//   [0]  sentinel  – c0 stores the world (root) AABB; c0_ref = 1 (true root).
//                    bvh_iter_new reads only this entry for the initial test.
//   [1…] actual BVH nodes, CPU indices shifted up by 1.
//
#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct GpuBVHNode {
	c0_min_and_ref:   [f32; 4], // xyz = child-0 (or own) AABB min, w = bitcast(c0_ref/base)
	c0_max_and_ref2:  [f32; 4], // xyz = child-0 (or own) AABB max, w = bitcast(c1_ref/count)
	c1_min_and_flags: [f32; 4], // xyz = child-1 AABB min (or 0),   w = bitcast(flags)
	c1_max_pad:       [f32; 4], // xyz = child-1 AABB max (or 0),   w = 0
	// Total: 64 bytes = 1 cache line
}

impl GpuBVHNode {
	/// Internal node: both child AABBs + their shifted node indices.
	fn internal(
		c0_min: Vec3, c0_max: Vec3, c0_ref: u32,
		c1_min: Vec3, c1_max: Vec3, c1_ref: u32,
	) -> Self {
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

	/// Leaf node: own AABB + item range.
	fn leaf(min: Vec3, max: Vec3, base: u32, count: u32) -> Self {
		let [min_x, min_y, min_z] = min.to_array();
		let [max_x, max_y, max_z] = max.to_array();
		Self {
			c0_min_and_ref:   [min_x, min_y, min_z, f32::from_bits(base)],
			c0_max_and_ref2:  [max_x, max_y, max_z, f32::from_bits(count)],
			c1_min_and_flags: [0.0, 0.0, 0.0,       f32::from_bits(1)], // flags bit 0 = is_leaf
			c1_max_pad:       [0.0; 4],
		}
	}

	/// Sentinel at index 0: stores the world (root) AABB in the c0 slot and a
	/// direct reference to the true root node (always at index 1).
	/// bvh_iter_new reads this entry for the initial ray–scene AABB test.
	fn sentinel(world_min: Vec3, world_max: Vec3, root_ref: u32) -> Self {
		let [min_x, min_y, min_z] = world_min.to_array();
		let [max_x, max_y, max_z] = world_max.to_array();
		Self {
			c0_min_and_ref:  [min_x, min_y, min_z, f32::from_bits(root_ref)],
			c0_max_and_ref2: [max_x, max_y, max_z, 0.0],
			c1_min_and_flags: [0.0; 4],
			c1_max_pad:       [0.0; 4],
		}
	}
}

// -- Item layout (unchanged) ---------------------------------------------------
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
}

pub struct GpuBvh {
	pub bvh_buffer: wgpu::Buffer,
	pub items_buffer: wgpu::Buffer,
	pub bind_group: wgpu::BindGroup,
	pub item_hit_count_buffer: wgpu::Buffer,
	pub item_hit_count_staging_buffer: wgpu::Buffer,
	pub item_count: usize,
	pub bind_group_layout: wgpu::BindGroupLayout,
	pub item_ids: Vec<(PhysicsBodyId, GridId, SubGridId)>,
}

impl GpuBvh {
	pub fn from_bvh(
		device: &Device,
		bvh: &bvh::BVH<(PhysicsBodyId, GridId, SubGridId)>,
		gpu_grid_tree_id_to_id_poses: &HashMap<(PhysicsBodyId, GridId, SubGridId), (u32, u32, Pose, )>,
	) -> Self {
		let (nodes, items) = bvh.get_internals();

		// -- Build GPU node buffer ---------------------------------------------
		//
		// GPU index = CPU index + 1 (slot 0 is the sentinel).
		// All sub1/sub2 child refs are incremented by 1 accordingly.
		//
		let mut gpu_nodes: Vec<GpuBVHNode> = Vec::with_capacity(nodes.len() + 1);

		// Sentinel at index 0
		if let Some(root) = nodes.first() {
			// root_ref = 1: the actual root node lives at GPU index 1.
			gpu_nodes.push(GpuBVHNode::sentinel(root.min_corner, root.max_corner, 1));
		} else {
			// Empty BVH: sentinel with ref 0 (bvh_iter_new treats 0 as no-root).
			gpu_nodes.push(GpuBVHNode::sentinel(Vec3::ZERO, Vec3::ZERO, 0));
		}

		// Actual BVH nodes (shifted to indices 1…n)
		for node in nodes.iter() {
			let gpu_node = match node.sub_nodes {
				bvh::BVHInternal::SubNodes { sub1, sub2 } => {
					// Embed both children's AABBs directly into the parent node.
					// The shader reads this node once and has both AABBs immediately.
					let c0 = &nodes[sub1 as usize];
					let c1 = &nodes[sub2 as usize];
					GpuBVHNode::internal(
						c0.min_corner, c0.max_corner, sub1 as u32 + 1,
						c1.min_corner, c1.max_corner, sub2 as u32 + 1,
					)
				}
				bvh::BVHInternal::Leaf { start, count } => {
					GpuBVHNode::leaf(node.min_corner, node.max_corner, start as u32, count as u32)
				}
			};
			gpu_nodes.push(gpu_node);
		}

		let bvh_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label:    Some("bvh_buffer"),
			contents: bytemuck::cast_slice(&gpu_nodes),
			usage:    wgpu::BufferUsages::STORAGE,
		});

		// -- Build GPU item buffer ---------------------------------------------
		let mut item_data: Vec<u8> = Vec::with_capacity(items.len() * size_of::<GpuBVHItem>());
		let mut item_ids: Vec<_> = Vec::with_capacity(items.len());
		for item in items {
			if let Some((tree_offset, voxels_offset, pose)) = gpu_grid_tree_id_to_id_poses.get(&item.0) {
				item_data.extend_from_slice(bytemuck::bytes_of(&GpuBVHItem {
					min_corner:   item.1.0.to_array(),
					aabb_size:    (item.1.1 - item.1.0).ceil().as_u8vec3().to_array(),
					_padding:     0,
					item_index:   *tree_offset,
					item_index_2: *voxels_offset,
					pos:          pose.translation.to_array(),
					quat:         pose.rotation.to_array(),
				}));
				item_ids.push(item.0);
			} else {
				println!("BVH item not found. Inserting 0 node into gpu bvh!");
				item_data.resize(item_data.len() + size_of::<GpuBVHItem>(), 0);
			}
		}

		if item_data.is_empty() { item_data.extend_from_slice(&[0; size_of::<GpuBVHItem>()]); }

		let items_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label:    Some("bvh_items_buffer"),
			contents: &item_data,
			usage:    wgpu::BufferUsages::STORAGE,
		});

		// -- Build hit-count buffers -------------------------------------------
		let item_count     = items.len().max(1); // match the ≥1 guarantee on items_buffer
		let hit_count_size = (item_count * std::mem::size_of::<u32>()) as u64;

		let item_hit_count_buffer = device.create_buffer(&wgpu::BufferDescriptor {
			label:              Some("bvh_item_hit_count_buffer"),
			size:               hit_count_size,
			// COPY_SRC so we can blit into the staging buffer each frame
			usage:              wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
			mapped_at_creation: false, // zero-initialised by wgpu
		});
		let item_hit_count_staging_buffer = device.create_buffer(&wgpu::BufferDescriptor {
			label:              Some("bvh_item_hit_count_staging_buffer"),
			size:               hit_count_size,
			usage:              wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		});

		let bind_group_layout = Self::bind_group_layout(device);
		let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
			layout:  &bind_group_layout,
			entries: &[
				wgpu::BindGroupEntry {
					binding:  0,
					resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
						buffer: &bvh_buffer, offset: 0, size: None,
					}),
				},
				wgpu::BindGroupEntry {
					binding:  1,
					resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
						buffer: &items_buffer, offset: 0, size: None,
					}),
				},
				wgpu::BindGroupEntry {
					binding:  2,
					resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
						buffer: &item_hit_count_buffer, offset: 0, size: None,
					}),
				},
			],
			label: Some("bvh_bind_group"),
		});

		Self { bvh_buffer, items_buffer, item_hit_count_buffer, item_hit_count_staging_buffer, item_count, bind_group, bind_group_layout, item_ids }
	}

	pub fn bind_group_layout(device: &Device) -> wgpu::BindGroupLayout {
		device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[
				wgpu::BindGroupLayoutEntry {
					binding:    0,
					visibility: wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::COMPUTE,
					ty: wgpu::BindingType::Buffer {
						ty:                 wgpu::BufferBindingType::Storage { read_only: true },
						has_dynamic_offset: false,
						min_binding_size:   None,
					},
					count: None,
				},
				wgpu::BindGroupLayoutEntry {
					binding:    1,
					visibility: wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::COMPUTE,
					ty: wgpu::BindingType::Buffer {
						ty:                 wgpu::BufferBindingType::Storage { read_only: true },
						has_dynamic_offset: false,
						min_binding_size:   None,
					},
					count: None,
				},
				wgpu::BindGroupLayoutEntry {
					binding:    2,
					visibility: wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::COMPUTE,
					ty: wgpu::BindingType::Buffer {
						ty:                 wgpu::BufferBindingType::Storage { read_only: false },
						has_dynamic_offset: false,
						min_binding_size:   None,
					},
					count: None,
				},
			],
			label: Some("bvh_bind_group_layout"),
		})
	}
}
