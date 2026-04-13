use std::collections::{HashMap};

use parry3d;

use crate::{grid_tree::{self, GridTree}, voxels::VoxelPalette};

use rand::{seq::IteratorRandom};

const SLOT_BYTES:   usize = 4;
const HEADER_BYTES: usize = 12;

fn slots_for_nonleaf(bitmap: u64, depth: u8) -> usize {
    let count = bitmap.count_ones() as usize;
    let entry_bytes = if depth == 1 { 1 } else { 2 };
    (HEADER_BYTES + count * entry_bytes).div_ceil(SLOT_BYTES)
}
// ── LOD collapse ──────────────────────────────────────────────────────────────

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
                        if let Some(c) = palette_vec.get(pi as usize) {
                            for j in 0..4 { sum_color[j] += c[j] as f64; }
                            n += 1;
                        }
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

// ── Bitmap helper ─────────────────────────────────────────────────────────────

fn build_bitmap(node: &grid_tree::GridTreeNode) -> u64 {
    let mut bitmap = 0u64;
    for (i, cell) in node.contents.iter().enumerate() {
        if cell.value_type() != 0 {
            bitmap |= 1u64 << i;
        }
    }
    bitmap
}

// ── Main function ─────────────────────────────────────────────────────────────

pub fn make_gpu_grid_tree(grid_tree: &GridTree, palette: &VoxelPalette, lod_level: f32) -> (Vec<u8>, Vec<u8>) {
    let (nodes, _, root_depth) = grid_tree.get_internals();
    assert!(!nodes.is_empty(), "ERROR: tree must have at least a root node.");

    // ── Palette ───────────────────────────────────────────────────────────────
    let mut palette_vec: Vec<[u8; 4]>   = Vec::new();
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
    let palette_len_bytes = (palette_vec.len() as u8).to_le_bytes();
    let palette_bytes: &[u8] = bytemuck::cast_slice(&palette_vec);

    // ── LOD collapse ──────────────────────────────────────────────────────────
    let mut collapse_rep = compute_lod_collapse(nodes, root_depth, &palette_vec, &palette_map, lod_level);
    collapse_rep[0] = None;

    // ── Pass 1: DFS pre-order → gpu_order ────────────────────────────────────
    let mut cpu_to_gpu: Vec<u32>       = vec![u32::MAX; nodes.len()];
    let mut gpu_order:  Vec<(u32, u8)> = Vec::with_capacity(nodes.len());

    let mut dfs_stack: Vec<(u32, u8)> = vec![(0, root_depth)];
    while let Some((cpu_idx, depth)) = dfs_stack.pop() {
        let gpu_idx = gpu_order.len() as u32;
        cpu_to_gpu[cpu_idx as usize] = gpu_idx;
        gpu_order.push((cpu_idx, depth));

        if depth > 0 {
            for cell in nodes[cpu_idx as usize].contents.iter() {
                if cell.value_type() == 2 {
                    let child_cpu = cpu_idx + cell.value() as u32;
                    if collapse_rep[child_cpu as usize].is_some() { continue; }
                    dfs_stack.push((child_cpu, depth - 1));
                }
            }
        }
    }

    // ── Find separating plane ─────────────────────────────────────────────────
    //
    // Encoding (bytes 1-3 of tree_buffer header):
    //   nx, ny, nz : 4-bit each  –  actual = encoded − 7.5
    //   m          : 8-bit       –  actual = encoded / 32.0 − 3.984375
    //
    // Solid halfspace:  dot(n, pos − root_centre) ≤ m_actual · root_size

    const TREE_LOG_SIZE: u32 = 2;
    let root_size_f = (1u32 << (TREE_LOG_SIZE * (root_depth as u32 + 1))) as f32;
    let root_half   = root_size_f / 2.0;

    // Collect leaf-level cells with their centres (root-relative) and half-extent.
    struct TaggedCell { center_x: f32, center_y: f32, center_z: f32, half: f32, solid: bool }
    let mut tagged_cells: Vec<TaggedCell> = Vec::new();
    {
        let mut stack: Vec<(u32, u8, [u32; 3])> = vec![(0u32, root_depth, [0u32; 3])];
        while let Some((cpu_idx, depth, node_origin)) = stack.pop() {
            let node_size = 1u32 << (TREE_LOG_SIZE * (depth as u32 + 1));
            let cell_size = node_size >> TREE_LOG_SIZE;
            let half      = cell_size as f32 * 0.5;
            let node      = &nodes[cpu_idx as usize];

            for (i, cell) in node.contents.iter().enumerate() {
                let cell_x = (i % grid_tree::SIZE_USIZE) as u32;
                let cell_y = ((i / grid_tree::SIZE_USIZE) % grid_tree::SIZE_USIZE) as u32;
                let cell_z = (i / (grid_tree::SIZE_USIZE * grid_tree::SIZE_USIZE)) as u32;
                let center_x = node_origin[0] as f32 + cell_x as f32 * cell_size as f32 + half - root_half;
                let center_y = node_origin[1] as f32 + cell_y as f32 * cell_size as f32 + half - root_half;
                let center_z = node_origin[2] as f32 + cell_z as f32 * cell_size as f32 + half - root_half;
                let solid = match cell.value_type() {
                    0 => false,
                    1 => true,
                    2 => {
                        let child_cpu = cpu_idx + cell.value() as u32;
                        if collapse_rep[child_cpu as usize].is_some() {
                            true
                        } else {
                            stack.push((child_cpu, depth - 1, [
                                node_origin[0] + cell_x * cell_size,
                                node_origin[1] + cell_y * cell_size,
                                node_origin[2] + cell_z * cell_size,
                            ]));
                            continue;
                        }
                    }
                    _ => unreachable!(),
                };
                tagged_cells.push(TaggedCell { center_x, center_y, center_z, half, solid });
            }
        }
    }

    // (ox, oy, oz, half)
    let mut solid_cells: Vec<parry3d::math::Vec3> = vec![];
	for cell in &tagged_cells {
		if cell.solid {
			solid_cells.push(parry3d::math::Vec3::new(cell.center_x - cell.half, cell.center_y - cell.half, cell.center_z - cell.half));
			solid_cells.push(parry3d::math::Vec3::new(cell.center_x + cell.half, cell.center_y - cell.half, cell.center_z - cell.half));
			solid_cells.push(parry3d::math::Vec3::new(cell.center_x - cell.half, cell.center_y + cell.half, cell.center_z - cell.half));
			solid_cells.push(parry3d::math::Vec3::new(cell.center_x - cell.half, cell.center_y - cell.half, cell.center_z + cell.half));
			solid_cells.push(parry3d::math::Vec3::new(cell.center_x + cell.half, cell.center_y + cell.half, cell.center_z - cell.half));
			solid_cells.push(parry3d::math::Vec3::new(cell.center_x + cell.half, cell.center_y - cell.half, cell.center_z + cell.half));
			solid_cells.push(parry3d::math::Vec3::new(cell.center_x - cell.half, cell.center_y + cell.half, cell.center_z + cell.half));
			solid_cells.push(parry3d::math::Vec3::new(cell.center_x + cell.half, cell.center_y + cell.half, cell.center_z + cell.half));
		}
	}
	let (solid_cells_reduced, _indices) = parry3d::transformation::convex_hull(&solid_cells);

	let mut rng = rand::rng();
    let empty_cells: Vec<(f32, f32, f32, f32)> = {
		let empty_cells = tagged_cells.into_iter().filter(|c| !c.solid).map(|c| (c.center_x, c.center_y, c.center_z, c.half)).collect::<Vec<_>>();
		let empty_cells_len = empty_cells.len();
		if empty_cells_len > 100 {
			empty_cells.into_iter().sample(&mut rng, 100)
		} else {
			empty_cells
		}
	};

    let best_plane = if solid_cells.is_empty() {
        None
    } else {
        // (wrong_empty_count, nxe, nye, nze, m_encoded)
        let mut best: (usize, u8, u8, u8, u8) = (empty_cells.len() + 1, 0, 0, 0, 0);

        for nxe in 0u8..=7 {
			let nx = (nxe * 2) as f32 - 7.5;
			for nye in 0u8..=7 {
				let ny = (nye * 2) as f32 - 7.5;
				for nze in 0u8..=7 {
					let nz = (nze * 2) as f32 - 7.5;
					let max_solid_dot = solid_cells_reduced.iter()
						.map(|o| nx * o.x + ny * o.y + nz * o.z)
						.fold(f32::NEG_INFINITY, f32::max);

					let actual_m = max_solid_dot / root_size_f;
					let m_enc_f  = (actual_m + 3.984375) * 32.0;
					if !(0.0..=255.0).contains(&m_enc_f) { continue; }

					let m_enc_ceiled     = m_enc_f.ceil();
					let ceiled_threshold = (m_enc_ceiled / 32.0 - 3.984375) * root_size_f;

					let mut wrong_none = 0usize;
					for &(ox, oy, oz, _) in empty_cells.iter() {
						if nx * ox + ny * oy + nz * oz <= ceiled_threshold {
							wrong_none += 1;
							if wrong_none >= best.0 {
								break;
							}
						}
					}

					if wrong_none < best.0 {
						best = (wrong_none, nxe * 2, nye * 2, nze * 2, m_enc_ceiled as u8);
					}
				}
			}
        }

        if best.0 < empty_cells.len() { Some((best.1, best.2, best.3, best.4)) } else { None }
    };

    // ── Pass 2: Assign slot indices and voxel buffer offsets ─────────────────
    let mut slot_indices:  Vec<u32> = vec![0; gpu_order.len()];
    let mut voxel_offsets: Vec<u16> = vec![0; gpu_order.len()];
    let mut tree_cursor:  u32 = 0;
    let mut voxel_cursor: u32 = 0;

    for (gpu_idx, &(cpu_idx, depth)) in gpu_order.iter().enumerate() {
        let node   = &nodes[cpu_idx as usize];
        let bitmap = build_bitmap(node);

        slot_indices[gpu_idx] = tree_cursor;
        tree_cursor += if depth == 0 { 3 } else { slots_for_nonleaf(bitmap, depth) as u32 };

        let mut offset_back_shift = 0;
        let mut voxel_data_size   = 0;
        let mut voxel_node_run    = 0;
        let mut hit_data          = false;

        for cell in node.contents {
            match cell.value_type() {
                0 => {}
                1 => {
                    if hit_data {
                        voxel_data_size += 1 + voxel_node_run;
                    } else {
                        offset_back_shift += voxel_node_run / 4;
                        voxel_data_size   += 1 + voxel_node_run % 4;
                        hit_data           = true;
                    }
                    voxel_node_run = 0;
                }
                2 => {
                    let child_cpu = cpu_idx + cell.value() as u32;
                    if collapse_rep[child_cpu as usize].is_some() {
                        if hit_data {
                            voxel_data_size += 1 + voxel_node_run;
                        } else {
                            offset_back_shift += voxel_node_run / 4;
                            voxel_data_size   += 1 + voxel_node_run % 4;
                            hit_data           = true;
                        }
                        voxel_node_run = 0;
                    } else {
                        voxel_node_run += 1;
                    }
                }
                _ => unreachable!(),
            }
        }

        if voxel_data_size > 0 {
            let aligned = voxel_cursor.next_multiple_of(4);
            debug_assert!(
                aligned / 4 <= u16::MAX as u32,
                "voxel_data_offset overflow at node {gpu_idx}: buffer 2 exceeds 256 KiB"
            );
            voxel_offsets[gpu_idx] = (aligned / 4) as u16 + offset_back_shift as u16;
            voxel_cursor = aligned + voxel_data_size;
        }
    }

    let total_tree_slots  = tree_cursor as usize;
    let total_voxel_bytes = voxel_cursor as usize;

    // ── Pass 3: Write bytes ───────────────────────────────────────────────────
    let mut tree_bytes: Vec<u8> = vec![0u8; total_tree_slots * SLOT_BYTES];
    let mut voxel_bytes: Vec<u8> = vec![0u8; total_voxel_bytes];

    for (gpu_idx, &(cpu_idx, depth)) in gpu_order.iter().enumerate() {
        let node      = &nodes[cpu_idx as usize];
        let my_slot   = slot_indices[gpu_idx];
        let byte_base = my_slot as usize * SLOT_BYTES;
        let bitmap    = build_bitmap(node);

        let parent_slot_offset: u16 = if node.parent_offset == 0 {
            0
        } else {
            let parent_cpu  = cpu_idx - node.parent_offset as u32;
            let parent_slot = slot_indices[cpu_to_gpu[parent_cpu as usize] as usize];
            (my_slot - parent_slot) as u16
        };

        tree_bytes[byte_base..byte_base + 2].copy_from_slice(&parent_slot_offset.to_le_bytes());
        tree_bytes[byte_base + 2..byte_base + 4].copy_from_slice(&voxel_offsets[gpu_idx].to_le_bytes());
        tree_bytes[byte_base + 4..byte_base + 12].copy_from_slice(&bitmap.to_le_bytes());

        let mut vox_write = voxel_offsets[gpu_idx] as usize * 4;

        if depth == 0 {
            for i in 0..grid_tree::SIZE_USIZE_CUBED {
                if bitmap & (1u64 << i) == 0 { continue; }
                let pal = match node.contents[i].value_type() {
                    1 => palette_map.get(&node.contents[i].value()).copied().unwrap_or(254),
                    _ => unreachable!("depth-0 cell must be DATA"),
                };
                voxel_bytes[vox_write] = pal;
                vox_write += 1;
            }
        } else {
            let mut entry_cursor   = byte_base + HEADER_BYTES;
            let mut voxel_node_run = 0;
            let mut hit_data       = false;

            for i in 0..grid_tree::SIZE_USIZE_CUBED {
                if bitmap & (1u64 << i) == 0 { continue; }
                let cell = &node.contents[i];

                let entry_val: u16 = match cell.value_type() {
                    1 => {
                        if hit_data { vox_write += voxel_node_run; }
                        else        { vox_write += voxel_node_run % 4; hit_data = true; }
                        voxel_node_run = 0;
                        let pal = palette_map.get(&cell.value()).copied().unwrap_or(254);
                        voxel_bytes[vox_write] = pal;
                        vox_write += 1;
                        0x00
                    }
                    2 => {
                        let child_cpu = cpu_idx + cell.value() as u32;
                        if let Some(rep) = collapse_rep[child_cpu as usize] {
                            if hit_data { vox_write += voxel_node_run; }
                            else        { vox_write += voxel_node_run % 4; hit_data = true; }
                            voxel_node_run = 0;
                            voxel_bytes[vox_write] = rep;
                            vox_write += 1;
                            0x00
                        } else {
                            let child_slot = slot_indices[cpu_to_gpu[child_cpu as usize] as usize];
                            let offset = child_slot - my_slot;
                            voxel_node_run += 1;
                            debug_assert!(
								offset >= 1 && offset <= if depth == 1 { u8::MAX as u32 } else { u16::MAX as u32 },
								"NODE child slot offset {offset} out of range"
							);
                            offset as u16
                        }
                    }
                    _ => unreachable!(),
                };

				if depth == 1 {
                    tree_bytes[entry_cursor] = entry_val as u8;
                    entry_cursor += 1;
                } else {
                    tree_bytes[entry_cursor..entry_cursor + 2].copy_from_slice(&entry_val.to_le_bytes());
                    entry_cursor += 2;
                }
            }
        }
    }

    // ── Assemble final buffers ────────────────────────────────────────────────

    let tree_raw_len = 1 + tree_bytes.len();
    let tree_padded_len = tree_raw_len.next_multiple_of(wgpu::COPY_BUFFER_ALIGNMENT as usize);
    let mut tree_buffer = Vec::with_capacity(tree_padded_len);
    tree_buffer.push(root_depth);
    if let Some((x, y, z, m)) = best_plane {
        tree_buffer.extend_from_slice(
            &(x as u32 + ((y as u32) << 4) + ((z as u32) << 8) + ((m as u32) << 12))
                .to_le_bytes()[0..3],
        );
    } else {
        tree_buffer.extend_from_slice(&[0u8; 3]);
    }
    tree_buffer.extend_from_slice(&tree_bytes);
    tree_buffer.resize(tree_padded_len, 0);

    let voxel_raw_len    = voxel_bytes.len() + palette_len_bytes.len() + palette_bytes.len();
    let voxel_padded_len = voxel_raw_len.next_multiple_of(wgpu::COPY_BUFFER_ALIGNMENT as usize);
    let mut voxel_buffer = Vec::with_capacity(voxel_padded_len);
    voxel_buffer.extend_from_slice(&palette_len_bytes);
    voxel_buffer.extend_from_slice(palette_bytes);
    voxel_buffer.extend_from_slice(&voxel_bytes);
    voxel_buffer.resize(voxel_padded_len, 0);

    (tree_buffer, voxel_buffer)
}
