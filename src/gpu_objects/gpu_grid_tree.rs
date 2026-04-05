use std::collections::HashMap;

use crate::{grid_tree::{self, GridTree}, voxels::VoxelPalette};

// New layout (16-byte slots):
//
//   Non-leaf node  (depth > 0):
//     bytes 0..7   : u64 presence bitmap (bit i set iff cell i has a child/voxel)
//     bytes 8..9   : u16 parent_offset  (in 16-byte slots; 0 = root / no parent)
//     bytes 10..   : u16 per present cell, packed, wrapping into subsequent slots
//
//   Leaf node  (depth == 0):
//     bytes 0..7   : u64 presence bitmap
//     byte  8      : u8  parent_offset  (in 16-byte slots)
//     bytes 9..    : u8  per present cell, packed, wrapping into subsequent slots
//
// To read cell i:
//   1. Check bit i of the bitmap.  If clear -> EMPTY.
//   2. data_index = popcount(bitmap & ((1 << i) - 1))
//   3. For non-leaf: u16 at (node_slot_base*16 + 10 + data_index*2)
//      For leaf:     u8  at (node_slot_base*16 +  9 + data_index)
//
// Non-leaf data values  (u16):
//   bit15 == 0  ->  DATA  (palette index, or collapsed-LOD representative)
//   bit15 == 1  ->  NODE  (child slot offset = value & 0x7FFF)
//
// Leaf data values  (u8):
//   palette index  (0xFF is never a valid palette index; palette is ≤ 254 entries)

const SLOT_BYTES: usize = 16;
const NONLEAF_HEADER_BYTES: usize = 10; // 8 bitmap + 2 parent_offset
const LEAF_HEADER_BYTES:    usize =  9; // 8 bitmap + 1 parent_offset

/// How many 16-byte slots does a node occupy?
fn slots_for_node(bitmap: u64, is_leaf: bool) -> usize {
    let count = bitmap.count_ones() as usize;
    let (header, item_bytes) = if is_leaf {
        (LEAF_HEADER_BYTES, 1usize)
    } else {
        (NONLEAF_HEADER_BYTES, 2usize)
    };
    let total_bytes = header + count * item_bytes;
    total_bytes.div_ceil(SLOT_BYTES)
}

//  LOD collapse (unchanged logic, same signatures)

fn compute_lod_collapse(
    nodes:       &[grid_tree::GridTreeNode],
    root_depth:  u8,
    palette_vec: &[[u8; 4]],
    palette_map: &HashMap<u16, u8>,
    lod_level:   f32,
) -> Vec<Option<u8>> {
    let mut collapse_rep = vec![None::<u8>; nodes.len()];
    if lod_level <= 0.0 || palette_vec.is_empty() {
        return collapse_rep;
    }

    let lod_min_depth:      u8  = lod_level.floor() as u8;
    let variance_threshold: f32 = lod_level.fract();

    let mut avg_color: Vec<[f32; 4]> = vec![[0.0; 4]; nodes.len()];
    let mut count:     Vec<u64>      = vec![0;         nodes.len()];
    let mut variance:  Vec<f32>      = vec![0.0;       nodes.len()];

    let mut stack: Vec<(u32, u8, bool)> = vec![(0, root_depth, false)];
    while let Some((idx, depth, children_done)) = stack.pop() {
        if !children_done {
            stack.push((idx, depth, true));
            if depth > 0 {
                for cell in &nodes[idx as usize].contents {
                    if cell.value_type() == 2 {
                        stack.push((idx + cell.value() as u32, depth - 1, false));
                    }
                }
            }
            continue;
        }

        let node = &nodes[idx as usize];
        let mut sum_color = [0f64; 4];
        let mut n = 0u64;

        for cell in &node.contents {
            match cell.value_type() {
                0 => {}
                1 => {
                    if let Some(&pi) = palette_map.get(&cell.value()) {
                        let c = palette_vec[pi as usize];
                        for j in 0..4 { sum_color[j] += c[j] as f64; }
                        n += 1;
                    }
                }
                2 => {
                    let ci = (idx + cell.value() as u32) as usize;
                    let w  = count[ci] as f64;
                    for j in 0..4 { sum_color[j] += avg_color[ci][j] as f64 * w; }
                    n += count[ci];
                }
                _ => unreachable!(),
            }
        }

        let avg: [f32; 4] = if n > 0 {
            core::array::from_fn(|j| (sum_color[j] / n as f64) as f32)
        } else {
            [0.0; 4]
        };

        let mut sum_sq = 0f64;
        for cell in &node.contents {
            match cell.value_type() {
                0 => {}
                1 => {
                    if let Some(&pi) = palette_map.get(&cell.value()) {
                        let c = palette_vec[pi as usize];
                        for j in 0..3 {
                            let d = c[j] as f64 / 255.0 - avg[j] as f64 / 255.0;
                            sum_sq += d * d;
                        }
                    }
                }
                2 => {
                    let ci = (idx + cell.value() as u32) as usize;
                    let w  = count[ci] as f64;
                    sum_sq += variance[ci] as f64 * w;
                    for j in 0..3 {
                        let d = avg_color[ci][j] as f64 / 255.0 - avg[j] as f64 / 255.0;
                        sum_sq += d * d * w;
                    }
                }
                _ => unreachable!(),
            }
        }

        let var = if n > 0 { (sum_sq / n as f64) as f32 } else { 0.0 };

        avg_color[idx as usize] = avg;
        count    [idx as usize] = n;
        variance [idx as usize] = var;

        let should_collapse = n > 0 && (
            depth < lod_min_depth
            || (variance_threshold > 0.0
                && depth == lod_min_depth
                && var < variance_threshold)
        );

        if should_collapse {
            let rep: [u8; 4] = avg.map(|v| v.round().clamp(0.0, 255.0) as u8);
            collapse_rep[idx as usize] = Some(closest_palette_entry(palette_vec, rep));
        }
    }

    collapse_rep
}

fn closest_palette_entry(palette_vec: &[[u8; 4]], target: [u8; 4]) -> u8 {
    palette_vec
        .iter()
        .enumerate()
        .map(|(i, c)| {
            let d: i32 = (0..3)
                .map(|j| { let x = c[j] as i32 - target[j] as i32; x * x })
                .sum();
            (d, i as u8)
        })
        .min_by_key(|&(d, _)| d)
        .map(|(_, i)| i)
        .unwrap_or(0)
}

//  Build the bitmap for a CPU node
//
// A bit is set for every cell that is DATA or NODE (i.e. not EMPTY).
// Collapsed NODE children are treated as DATA, so they still count.
fn build_bitmap(
    node:         &grid_tree::GridTreeNode,
    cpu_idx:      u32,
    depth:        u8,
    collapse_rep: &[Option<u8>],
) -> u64 {
    let mut bitmap = 0u64;
    for (i, cell) in node.contents.iter().enumerate() {
        let present = match cell.value_type() {
            0 => false,
            1 => true,
            2 => {
                // NODE child: present whether collapsed or not
                true
            }
            _ => unreachable!(),
        };
        if present {
            bitmap |= 1u64 << i;
        }
    }
    // Collapsed children that were NODE still appear as DATA bits above,
    // but we must clear bits for children that are pruned AND whose subtree
    // is represented by their parent encoding them directly.
    // Actually: collapsed children are emitted as DATA in the parent, so
    // their bit stays set.  Nothing to clear.
    let _ = (cpu_idx, depth, collapse_rep); // suppress unused warnings
    bitmap
}

pub fn make_gpu_grid_tree(grid_tree: &GridTree, palette: &VoxelPalette, lod_level: f32) -> Vec<u8> {
    let (nodes, root_pos, root_depth) = grid_tree.get_internals();
    assert!(!nodes.is_empty(), "make_gpu_grid_tree: tree must have at least a root node");

    //  Header
    let root_pos_bytes:   [u8; 6] = bytemuck::cast([root_pos.x, root_pos.y, root_pos.z]);
    let root_depth_bytes: [u8; 1] = [root_depth];

    //  Palette
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

    //  LOD collapse
    let mut collapse_rep = compute_lod_collapse(
        nodes, root_depth, &palette_vec, &palette_map, lod_level,
    );
    collapse_rep[0] = None;

    //  Pass 1: DFS pre-order (leaf siblings before non-leaf siblings)
    //
    // Same ordering rule as before: leaf children first so they cluster together.
    let mut cpu_to_gpu: Vec<u32>       = vec![u32::MAX; nodes.len()];
    let mut gpu_order:  Vec<(u32, u8)> = Vec::with_capacity(nodes.len());

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
                    if collapse_rep[child_cpu as usize].is_some() { continue; }
                    if child_depth == 0 {
                        leaf_children.push((child_cpu, child_depth));
                    } else {
                        non_leaf_children.push((child_cpu, child_depth));
                    }
                }
            }
            for &child in non_leaf_children.iter().rev() { dfs_stack.push(child); }
            for &child in leaf_children.iter().rev()     { dfs_stack.push(child); }
        }
    }

    //  Pass 2: Assign slot indices
    //
    // Each node occupies ceil((header + popcount*item_size) / 16) slots.
    let mut slot_indices = vec![0u32; gpu_order.len()];
    let mut cursor: u32 = 0;
    for (gpu_idx, &(cpu_idx, depth)) in gpu_order.iter().enumerate() {
        let node   = &nodes[cpu_idx as usize];
        let is_leaf = depth == 0;

        // Build the bitmap to know popcount (collapsed children count as DATA).
        let bitmap = build_bitmap(node, cpu_idx, depth, &collapse_rep);

        slot_indices[gpu_idx] = cursor;
        cursor += slots_for_node(bitmap, is_leaf) as u32;
    }
    let total_slots = cursor as usize;

    //  Pass 3: Write node bytes
    let mut gpu_bytes: Vec<u8> = vec![0u8; total_slots * SLOT_BYTES];

    for (gpu_idx, &(cpu_idx, depth)) in gpu_order.iter().enumerate() {
        let node    = &nodes[cpu_idx as usize];
        let is_leaf = depth == 0;
        let my_slot = slot_indices[gpu_idx];
        let byte_base = my_slot as usize * SLOT_BYTES;

        //  Bitmap
        let bitmap = build_bitmap(node, cpu_idx, depth, &collapse_rep);
        gpu_bytes[byte_base..byte_base + 8].copy_from_slice(&bitmap.to_le_bytes());

        //  Parent offset (in slots)
        let parent_slot_offset: u32 = if node.parent_offset == 0 {
            0
        } else {
            let parent_cpu  = cpu_idx - node.parent_offset as u32;
            let parent_slot = slot_indices[cpu_to_gpu[parent_cpu as usize] as usize];
            my_slot - parent_slot
        };

        if is_leaf {
            debug_assert!(parent_slot_offset <= u8::MAX as u32,
                "leaf parent_slot_offset {parent_slot_offset} overflows u8");
            gpu_bytes[byte_base + 8] = parent_slot_offset as u8;
        } else {
            debug_assert!(parent_slot_offset <= u16::MAX as u32,
                "non-leaf parent_slot_offset {parent_slot_offset} overflows u16");
            let bytes = (parent_slot_offset as u16).to_le_bytes();
            gpu_bytes[byte_base + 8] = bytes[0];
            gpu_bytes[byte_base + 9] = bytes[1];
        }

        //  Data entries (packed after header, may spill into next slot)
        let header_bytes = if is_leaf { LEAF_HEADER_BYTES } else { NONLEAF_HEADER_BYTES };
        let mut data_byte_cursor = byte_base + header_bytes;

        // Iterate cells in bitmap order (bit 0 first).
        for i in 0..grid_tree::SIZE_USIZE_CUBED {
            if bitmap & (1u64 << i) == 0 { continue; }

            let cell = &node.contents[i];

            if is_leaf {
                // Leaf: 1-byte palette index.
                let pal: u8 = match cell.value_type() {
                    1 => palette_map.get(&cell.value()).copied().unwrap_or(254),
                    _ => unreachable!("leaf cells must be DATA"),
                };
                gpu_bytes[data_byte_cursor] = pal;
                data_byte_cursor += 1;
            } else {
                // Non-leaf: 2-byte value.
                let val: u16 = match cell.value_type() {
                    1 => {
                        // DATA
                        let pal = palette_map.get(&cell.value()).copied().unwrap_or(254) as u16;
                        debug_assert!(pal < 0x8000);
                        pal
                    }
                    2 => {
                        let child_cpu = cpu_idx + cell.value() as u32;
                        if let Some(rep_idx) = collapse_rep[child_cpu as usize] {
                            // Collapsed subtree -> DATA with representative color.
                            rep_idx as u16
                        } else {
                            // NODE: slot offset with bit15 set.
                            let child_slot = slot_indices[cpu_to_gpu[child_cpu as usize] as usize];
                            let offset     = child_slot - my_slot;
                            debug_assert!(offset < 0x8000,
                                "NODE child slot offset {offset} overflows 15 bits");
                            (offset as u16) | (1u16 << 15)
                        }
                    }
                    _ => unreachable!(),
                };
                let bytes = val.to_le_bytes();
                gpu_bytes[data_byte_cursor]     = bytes[0];
                gpu_bytes[data_byte_cursor + 1] = bytes[1];
                data_byte_cursor += 2;
            }
        }
    }

    //  Assemble final buffer
    let palette_len_byte: [u8; 1] = [palette_vec.len() as u8];
    let palette_bytes: Vec<u8>    = bytemuck::cast_vec(palette_vec);

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
