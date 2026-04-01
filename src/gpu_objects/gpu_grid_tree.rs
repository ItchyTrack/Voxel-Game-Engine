use std::collections::HashMap;

use crate::{grid_tree::{self, GridTree}, voxels::VoxelPalette};

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuGridTreeNode {
    parent_offset: u16,
    contents: [u16; grid_tree::SIZE_USIZE_CUBED],
}

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

// For each node, computes the average RGBA and RGB variance (mean squared distance
// from the mean, normalized to [0, 1]) of all DATA descendants, then decides
// whether the node should be collapsed:
//
//   depth < floor(lod_level)                             -> always collapse
//   depth == floor(lod_level) && variance < fract(lod)  -> collapse if uniform enough
//
// Returns: per-cpu-node Option<u8>:
//   Some(palette_idx) -> collapse this subtree, represent it with that palette entry
//   None              -> emit this node normally
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

    let lod_min_depth:        u8  = lod_level.floor() as u8;
    let variance_threshold:   f32 = lod_level.fract();

    // Per-node running stats, filled bottom-up.
    let mut avg_color: Vec<[f32; 4]> = vec![[0.0; 4]; nodes.len()];
    let mut count:     Vec<u64>      = vec![0;         nodes.len()];
    let mut variance:  Vec<f32>      = vec![0.0;       nodes.len()];

    // Iterative post-order DFS: (node_idx, depth, children_done).
    let mut stack: Vec<(u32, u8, bool)> = vec![(0, root_depth, false)];
    while let Some((idx, depth, children_done)) = stack.pop() {
        if !children_done {
            // Re-push self for the post-processing pass…
            stack.push((idx, depth, true));
            // …then push all NODE children.
            if depth > 0 {
                for cell in &nodes[idx as usize].contents {
                    if cell.value_type() == 2 {
                        stack.push((idx + cell.value() as u32, depth - 1, false));
                    }
                }
            }
            continue;
        }

        // ── Post-order: all children already have stats ────────────────────────
        let node = &nodes[idx as usize];

        // Pass 1 of 2: accumulate weighted sum -> compute average.
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

        // Pass 2 of 2: accumulate variance via the parallel-axis theorem.
        //   Var(X) = E[(X - μ)²]
        //   For a child subtree with mean μ_c and variance V_c:
        //     contribution = V_c + (μ_c - μ)²   (per-component, per voxel)
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

        // ── Collapse decision ─────────────────────────────────────────────────
        // Never collapse a node with no DATA descendants (would create a hole).
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

/// Returns the index of the palette entry whose RGB is closest to `target`.
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

pub fn make_gpu_grid_tree(grid_tree: &GridTree, palette: &VoxelPalette, lod_level: f32) -> Vec<u8> {
    let (nodes, root_pos, root_depth) = grid_tree.get_internals();
    assert!(!nodes.is_empty(), "make_gpu_grid_tree: tree must have at least a root node");

    // ── Header ────────────────────────────────────────────────────────────────
    let root_pos_bytes:   [u8; 6] = bytemuck::cast([root_pos.x, root_pos.y, root_pos.z]);
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

    // ── LOD: decide which subtrees to collapse ────────────────────────────────
    //
    // collapse_rep[cpu_idx] = Some(palette_idx)
    //   -> don't emit this node; its parent writes DATA(palette_idx) instead.
    // collapse_rep[cpu_idx] = None
    //   -> emit this node normally.
    //
    // Root (index 0) is never collapsed because it has no parent to absorb it.
    let mut collapse_rep = compute_lod_collapse(
        nodes, root_depth, &palette_vec, &palette_map, lod_level,
    );
    collapse_rep[0] = None;

    // ── Pass 1: DFS pre-order, leaf siblings before non-leaf siblings ─────────
    //
    // Collapsed subtrees are pruned: their root is never pushed onto the stack,
    // so they don't appear in gpu_order.
    let mut cpu_to_gpu: Vec<u32>      = vec![u32::MAX; nodes.len()];
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
                    // Prune collapsed subtrees from the traversal.
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

    // ── Pass 2: Assign half-indices (65-byte units) ───────────────────────────
    let mut half_indices = vec![0u32; gpu_order.len()];
    let mut cursor: u32 = 0;
    for (gpu_idx, &(_, depth)) in gpu_order.iter().enumerate() {
        if depth == 0 {
            half_indices[gpu_idx] = cursor;
            cursor += 1;
        } else {
            if cursor % 2 == 1 { cursor += 1; }
            half_indices[gpu_idx] = cursor;
            cursor += 2;
        }
    }
    let total_half_units = cursor as usize;

    // ── Pass 3: Write node bytes ──────────────────────────────────────────────
    let half_unit = std::mem::size_of::<GpuGridTreeLeafNode>(); // 65
    let full_unit = std::mem::size_of::<GpuGridTreeNode>();     // 130
    let mut gpu_bytes: Vec<u8> = vec![0u8; total_half_units * half_unit];

    for (gpu_idx, &(cpu_idx, depth)) in gpu_order.iter().enumerate() {
        let half_idx = half_indices[gpu_idx];
        let byte_off = half_idx as usize * half_unit;
        let node     = &nodes[cpu_idx as usize];

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
                        let pal = palette_map.get(&cell.value()).copied().unwrap_or(254) as u16;
                        debug_assert!(pal < 0x8000);
                        pal
                    }
                    2 => {
                        let child_cpu = cpu_idx + cell.value() as u32;

                        if let Some(rep_idx) = collapse_rep[child_cpu as usize] {
                            // Collapsed subtree -> emit as DATA with representative color.
                            // bit15 == 0, so this is unambiguously DATA.
                            rep_idx as u16
                        } else {
                            // Normal NODE: encode child half-unit offset with bit15 set.
                            let child_half = half_indices[cpu_to_gpu[child_cpu as usize] as usize];
                            let offset     = child_half - half_idx;
                            debug_assert!(offset < 0x8000,
                                "NODE child half-offset {offset} overflows 15 bits");
                            (offset as u16) | (1u16 << 15)
                        }
                    }
                    _ => unreachable!(),
                };
            }
            let nl = GpuGridTreeNode { parent_offset: parent_half_offset as u16, contents };
            gpu_bytes[byte_off..byte_off + full_unit].copy_from_slice(bytemuck::bytes_of(&nl));
        }
    }

    // ── Assemble final buffer ─────────────────────────────────────────────────
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
