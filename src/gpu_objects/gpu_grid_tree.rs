use std::collections::HashMap;

use crate::{grid_tree::{self, GridTree}, voxels::VoxelPalette};

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuGridTreeNode {
    parent_offset: u16,
    used_cell_count: u8,
    depth: u8,
    // Encoding per cell (u16):
    // If value == 1<<16-1: NONE, Else if last bit is 0: DATA, Else last bit is 1: NODE.
	// NODE value != 1<<15-1 because then NONE)
	// Same as GridTree
    contents: [u16; grid_tree::SIZE_USIZE_CUBED],
}

pub fn make_gpu_grid_tree(grid_tree: &GridTree, palette: &VoxelPalette) -> Vec<u8> {
	let (nodes, root_pos) = grid_tree.get_internals();

	let mut gpu_nodes: Vec<GpuGridTreeNode> = Vec::with_capacity(nodes.len());

	let mut palette_vec = vec![];
	let mut palette_map = HashMap::new();
	for (id, voxel) in &palette.palette {
		palette_map.insert(id, palette_vec.len() as u16);
		palette_vec.push(voxel.color); // [u8; 4]
	}

	for node in nodes {
		let mut contents = [0u16; grid_tree::SIZE_USIZE_CUBED];
		for (i, cell) in node.contents.iter().enumerate() {
			match cell.value_type() {
				1 => contents[i] = palette_map[&cell.value()],
				_ => contents[i] = cell.value,
			}
		}
		gpu_nodes.push(GpuGridTreeNode {
			parent_offset: node.parent_offset,
			used_cell_count: node.used_cell_count,
			depth: node.depth,
			contents,
		});
	}

	let root_pos_bytes: [u8; 8] = bytemuck::cast([
		root_pos.x,
		root_pos.y,
		root_pos.z,
		0i16,
	]);

	let palette_offset: [u8; 4] = bytemuck::cast([palette_vec.len() as u32]);
	let palette_bytes = bytemuck::cast_vec(palette_vec);

	let node_bytes = bytemuck::cast_slice(&gpu_nodes);

	let mut buffer_data = Vec::with_capacity(root_pos_bytes.len() + palette_offset.len() + palette_bytes.len() + node_bytes.len());
	buffer_data.extend_from_slice(&root_pos_bytes);
	buffer_data.extend_from_slice(&palette_offset);
	buffer_data.extend_from_slice(&palette_bytes);
	buffer_data.extend_from_slice(node_bytes);
	buffer_data
}
