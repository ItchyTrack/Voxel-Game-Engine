use std::collections::HashMap;

use glam::IVec3;
use wgpu::{Device, util::DeviceExt};

use crate::{physics::bvh, pose::Pose};

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct GpuBVHNode {
    min_corner: [f32; 3],
    max_corner: [f32; 3],
    data_1: u16, // sub1 or start
    data_2: u16, // sub2 or count
    is_leaf: u16,
	parent: u16,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct GpuBVHItem {
    min_corner: [f32; 3],
    aabb_size:  [u8; 3],
    _padding:   u8,
    item_index: u32,
    pos:        [f32; 3],
    quat:       [f32; 4],
}

pub struct GpuBvh {
    pub bvh_buffer:       wgpu::Buffer,
    pub items_buffer:     wgpu::Buffer,
    pub bind_group:       wgpu::BindGroup,
    pub bind_group_layout: wgpu::BindGroupLayout,
}

impl GpuBvh {
    pub fn from_bvh(
        device: &Device,
        bvh: &bvh::BVH<(u32, u32, IVec3)>,
        gpu_grid_tree_id_to_id_poses: &HashMap<(u32, u32, IVec3), (u32, Pose)>,
    ) -> Self {
        let (nodes, items) = bvh.get_internals();

        // ── Pass 1: build nodes, parent = 0xFFFF (unknown) ───────────────
        let mut gpu_nodes: Vec<GpuBVHNode> = nodes.iter().map(|node| {
            match node.sub_nodes {
                bvh::BVHInternal::SubNodes { sub1, sub2 } => GpuBVHNode {
                    min_corner:         node.min_corner.to_array(),
                    max_corner:         node.max_corner.to_array(),
                    data_1:             sub1,
                    data_2:             sub2,
                    is_leaf: 			0, // not a leaf
                    parent:				0xFFFF, // no parent
                },
                bvh::BVHInternal::Leaf { start, count } => GpuBVHNode {
                    min_corner:         node.min_corner.to_array(),
                    max_corner:         node.max_corner.to_array(),
                    data_1:             start,
                    data_2:             count,
					is_leaf: 			1, // is a leaf
                    parent:				0xFFFF, // no parent
                },
            }
        }).collect();

        // ── Pass 2: stamp each internal node's index into its children ────
        for i in 0..gpu_nodes.len() {
            if gpu_nodes[i].is_leaf == 0 {
                let d1 = gpu_nodes[i].data_1 as usize;
                let d2 = gpu_nodes[i].data_2 as usize;
                let parent_bits = i as u16;
                gpu_nodes[d1].parent = parent_bits;
                gpu_nodes[d2].parent = parent_bits;
            }
        }

        let bvh_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label:    Some("bvh_buffer"),
            contents: bytemuck::cast_slice(&gpu_nodes),
            usage:    wgpu::BufferUsages::STORAGE,
        });

        let mut item_data: Vec<u8> =
            Vec::with_capacity(items.len() * size_of::<GpuBVHItem>());
        for item in items {
            if let Some((id, pose)) = gpu_grid_tree_id_to_id_poses.get(&item.0) {
                item_data.extend_from_slice(bytemuck::bytes_of(&GpuBVHItem {
                    min_corner: item.1.0.to_array(),
                    aabb_size:  (item.1.1 - item.1.0).ceil().as_u8vec3().to_array(),
                    _padding:   0,
                    item_index: *id,
                    pos:        pose.translation.to_array(),
                    quat:       pose.rotation.to_array(),
                }));
            } else {
                println!("BVH item not found. Inserting 0 node into gpu bvh!");
                item_data.resize(item_data.len() + size_of::<GpuBVHItem>(), 0);
            }
        }

        let items_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label:    Some("bvh_items_buffer"),
            contents: &item_data,
            usage:    wgpu::BufferUsages::STORAGE,
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
            ],
            label: Some("bvh_bind_group"),
        });

        Self { bvh_buffer, items_buffer, bind_group, bind_group_layout }
    }

    pub fn bind_group_layout(device: &Device) -> wgpu::BindGroupLayout {
        device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding:    0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty:                 wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size:   None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding:    1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty:                 wgpu::BufferBindingType::Storage { read_only: true },
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
