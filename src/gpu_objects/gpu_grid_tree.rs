use std::collections::HashMap;

use crate::{grid_tree::{self, GridTree}, voxels::VoxelPalette};

// Non-leaf node: 130 bytes (2 half-units).
// parent_offset and NODE cell offsets are in half-units.
// Cell encoding:
//   0xFFFF        → NONE
//   bit15 == 0    → DATA  (palette index)
//   bit15 == 1    → NODE  (child half-unit offset from this node)
#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuGridTreeNode {
    parent_offset: u16,
    contents: [u16; grid_tree::SIZE_USIZE_CUBED],
}

// Leaf node (depth == 0): 65 bytes (1 half-unit).
// parent_offset is in half-units.  No NODE cells possible.
// Cell encoding:
//   0xFF  → NONE
//   else  → DATA (palette index)
#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuGridTreeLeafNode {
    parent_offset: u8,
    contents: [u8; grid_tree::SIZE_USIZE_CUBED],
}

const _: () = assert!(
    std::mem::size_of::<GpuGridTreeLeafNode>() * 2 == std::mem::size_of::<GpuGridTreeNode>(),
    "Two GpuGridTreeLeafNodes must fit in one GpuGridTreeNode slot"
);

pub fn make_gpu_grid_tree(grid_tree: &GridTree, palette: &VoxelPalette) -> Vec<u8> {
    let (nodes, root_pos, root_depth) = grid_tree.get_internals();
    assert!(!nodes.is_empty(), "make_gpu_grid_tree: tree must have at least a root node");

    // ── Header ────────────────────────────────────────────────────────────────
    let root_pos_bytes: [u8; 6] = bytemuck::cast([root_pos.x, root_pos.y, root_pos.z]);
    let root_depth_bytes: [u8; 1] = [root_depth];

    // ── Palette ───────────────────────────────────────────────────────────────
    let mut palette_vec: Vec<[u8; 4]> = Vec::new();
    let mut palette_map: HashMap<u16, u8> = HashMap::new();
    for (id, voxel) in &palette.palette {
        if palette_vec.len() >= 254 {
            println!("ERROR: ran out of palette space");
            palette_map.insert(*id, 254);
            continue;
        }
        palette_map.insert(*id, palette_vec.len() as u8);
        palette_vec.push(voxel.color);
    }
    let palette_len_byte: [u8; 1] = [palette_vec.len() as u8];
    let palette_bytes: Vec<u8> = bytemuck::cast_vec(palette_vec);

    // ── Pass 1: DFS pre-order, leaf siblings before non-leaf siblings ─────────
    //
    // Clustering all leaf children of a parent together in memory means they
    // fill consecutive 65-byte half-units and can be packed two-per-slot.
    let mut cpu_to_gpu: Vec<u32> = vec![u32::MAX; nodes.len()];
    let mut gpu_order: Vec<(u32, u8)> = Vec::with_capacity(nodes.len()); // (cpu_idx, depth)

    let mut dfs_stack: Vec<(u32, u8)> = vec![(0, root_depth)];
    while let Some((cpu_idx, depth)) = dfs_stack.pop() {
        let gpu_idx = gpu_order.len() as u32;
        cpu_to_gpu[cpu_idx as usize] = gpu_idx;
        gpu_order.push((cpu_idx, depth));

        if depth > 0 {
            let node = &nodes[cpu_idx as usize];
            let mut leaf_children:     Vec<(u32, u8)> = Vec::new();
            let mut non_leaf_children: Vec<(u32, u8)> = Vec::new();
            for cell in &node.contents {
                if cell.value_type() == 2 {
                    let child_cpu   = cpu_idx + cell.value() as u32;
                    let child_depth = depth - 1;
                    if child_depth == 0 {
                        leaf_children.push((child_cpu, child_depth));
                    } else {
                        non_leaf_children.push((child_cpu, child_depth));
                    }
                }
            }
            // Push in reverse so cell-order is preserved on pop.
            // Non-leaves go deeper in the stack (processed after leaves).
            for &child in non_leaf_children.iter().rev() { dfs_stack.push(child); }
            for &child in leaf_children.iter().rev()     { dfs_stack.push(child); }
        }
    }

    // ── Pass 2: Assign half-indices (65-byte units) ───────────────────────────
    //
    // Leaf node    → 1 half-unit.
    // Non-leaf     → 2 half-units, must start at an even half-index (130-byte aligned).
    //
    // When a non-leaf follows an odd number of leaves, we insert a one-half-unit
    // pad (65 bytes of zeros) so the non-leaf lands on an even boundary.
    let mut half_indices = vec![0u32; gpu_order.len()];
    let mut cursor: u32 = 0;
    for (gpu_idx, &(_, depth)) in gpu_order.iter().enumerate() {
        if depth == 0 {
            half_indices[gpu_idx] = cursor;
            cursor += 1;
        } else {
            if cursor % 2 == 1 { cursor += 1; } // align to even (130-byte boundary)
            half_indices[gpu_idx] = cursor;
            cursor += 2;
        }
    }
    let total_half_units = cursor as usize;

    // ── Pass 3: Write node bytes ──────────────────────────────────────────────
    //
    // Pre-allocate the full node region; padding slots stay as the zero fill.
    let half_unit = std::mem::size_of::<GpuGridTreeLeafNode>(); // 65
    let full_unit = std::mem::size_of::<GpuGridTreeNode>();     // 130
    let mut gpu_bytes: Vec<u8> = vec![0u8; total_half_units * half_unit];

    for (gpu_idx, &(cpu_idx, depth)) in gpu_order.iter().enumerate() {
        let half_idx = half_indices[gpu_idx];
        let byte_off = half_idx as usize * half_unit;
        let node     = &nodes[cpu_idx as usize];

        // parent_offset in half-units; 0 means "no parent" (root)
        let parent_half_offset: u32 = if node.parent_offset == 0 {
            0
        } else {
            let parent_cpu  = cpu_idx - node.parent_offset as u32;
            let parent_half = half_indices[cpu_to_gpu[parent_cpu as usize] as usize];
            half_idx - parent_half
        };

        if depth == 0 {
            // ── Leaf (65 bytes, 1 half-unit) ──────────────────────────────────
            debug_assert!(parent_half_offset <= u8::MAX as u32,
                "leaf parent_offset {parent_half_offset} overflows u8");
            let mut contents = [0xFFu8; grid_tree::SIZE_USIZE_CUBED];
            for (i, cell) in node.contents.iter().enumerate() {
                contents[i] = match cell.value_type() {
                    0 => 0xFF,
                    1 => palette_map.get(&cell.value()).copied().unwrap_or(254),
                    _ => unreachable!("leaf node cannot contain NODE cells"),
                };
            }
            let leaf = GpuGridTreeLeafNode { parent_offset: parent_half_offset as u8, contents };
            gpu_bytes[byte_off..byte_off + half_unit].copy_from_slice(bytemuck::bytes_of(&leaf));
        } else {
            // ── Non-leaf (130 bytes, 2 half-units) ────────────────────────────
            debug_assert!(parent_half_offset <= u16::MAX as u32,
                "non-leaf parent_offset {parent_half_offset} overflows u16");
            let mut contents = [0xFFFFu16; grid_tree::SIZE_USIZE_CUBED];
            for (i, cell) in node.contents.iter().enumerate() {
                contents[i] = match cell.value_type() {
                    0 => 0xFFFF,
                    1 => {
                        // DATA: palette index, bit15 must be 0
                        let pal = palette_map.get(&cell.value()).copied().unwrap_or(254) as u16;
                        debug_assert!(pal < 0x8000);
                        pal
                    }
                    2 => {
                        // NODE: half-unit offset to child, bit15 set
                        let child_cpu  = cpu_idx + cell.value() as u32;
                        let child_half = half_indices[cpu_to_gpu[child_cpu as usize] as usize];
                        let offset     = child_half - half_idx;
                        debug_assert!(offset < 0x8000,
                            "NODE child half-offset {offset} overflows 15 bits");
                        (offset as u16) | (1u16 << 15)
                    }
                    _ => unreachable!(),
                };
            }
            let nl = GpuGridTreeNode { parent_offset: parent_half_offset as u16, contents };
            gpu_bytes[byte_off..byte_off + full_unit].copy_from_slice(bytemuck::bytes_of(&nl));
        }
    }

    // ── Assemble final buffer ─────────────────────────────────────────────────
    // Layout:
    //   [root_pos:    6 bytes  (3× i16 little-endian)]
    //   [root_depth:  1 byte]
    //   [palette_len: 1 byte]
    //   [palette:     palette_len × 4 bytes (RGBA u8)]
    //   [nodes:       total_half_units × 65 bytes]
    //   [pad:         to wgpu::COPY_BUFFER_ALIGNMENT]
    let data_len =
        root_pos_bytes.len()
        + root_depth_bytes.len()
        + palette_len_byte.len()
        + palette_bytes.len()
        + gpu_bytes.len();
    let padded_len = data_len.next_multiple_of(wgpu::COPY_BUFFER_ALIGNMENT as usize);

    let mut buffer_data = Vec::with_capacity(padded_len);
    buffer_data.extend_from_slice(&root_pos_bytes);
    buffer_data.extend_from_slice(&root_depth_bytes);
    buffer_data.extend_from_slice(&palette_len_byte);
    buffer_data.extend_from_slice(&palette_bytes);
    buffer_data.extend_from_slice(&gpu_bytes);
    buffer_data.resize(padded_len, 0);
    buffer_data
}
