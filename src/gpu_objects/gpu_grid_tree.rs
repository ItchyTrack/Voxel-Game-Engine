use std::collections::HashMap;

use crate::{grid_tree::{self, GridTree}, voxels::VoxelPalette};

// Buffer 1 – tree buffer
//
//  byte 0 : u8  root_depth
//  bytes 1..: node slots (SLOT_BYTES = 12 each)
//
//  Every node begins with a 12-byte header (= exactly 1 slot):
//    bytes 0..1  : u16 parent_offset      (slots from this node back to parent; 0 = root)
//    bytes 2..3  : u16 voxel_data_offset  (start of this node's voxels in buffer 2, *4 bytes)
//    bytes 4..11 : u64 bitmap             (bit i = 1 iff cell i is DATA or NODE, 0 = NONE)
//
//  Depth-0 nodes: exactly 1 slot (12 bytes), nothing after.
//    Every set bitmap bit is a DATA voxel.
//    Voxel index for bit i = popcount(bitmap & ((1 << i) - 1)).
//
//  Non-leaf nodes (depth > 0): header slot + packed u16 per set bitmap bit:
//    0x0000        -> DATA  (voxel; k-th DATA entry maps to buffer-2 index k)
//    0x0001..FFFF  -> NODE  (child slot offset from this node's slot index)
//
// Buffer 2 – voxel buffer
//
//  Palette size 8 bits
//	Palette 4 bytes per entry. total size (Palette size) * 4
//
//  Packed u8 palette indices.
//  Node N's voxels begin at byte  voxel_data_offset_N * 4.
//  Within that run, entry k = k-th DATA cell (bitmap order).
//
//  Each node's voxel run starts at the next 4-byte-aligned position.
//  Nodes with zero DATA cells do not advance the cursor.
//  Buffer ends immediately after the last written voxel byte.

const SLOT_BYTES:   usize = 12;
const HEADER_BYTES: usize = 12; // same for all nodes; depth-0 nodes have nothing after

fn slots_for_nonleaf(bitmap: u64) -> usize {
	let count = bitmap.count_ones() as usize;
	(HEADER_BYTES + count * 2).div_ceil(SLOT_BYTES)
}

// LOD collapse

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

// Helpers

fn build_bitmap(node: &grid_tree::GridTreeNode) -> u64 {
	let mut bitmap = 0u64;
	for (i, cell) in node.contents.iter().enumerate() {
		if cell.value_type() != 0 {
			bitmap |= 1u64 << i;
		}
	}
	bitmap
}

// Main entry point

pub fn make_gpu_grid_tree(grid_tree: &GridTree, palette: &VoxelPalette, lod_level: f32) -> (Vec<u8>, Vec<u8>) {
	let (nodes, _, root_depth) = grid_tree.get_internals();
	assert!(!nodes.is_empty(), "make_gpu_grid_tree: tree must have at least a root node");

	// Palette
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
	let palette_len_bytes = (palette_vec.len() as u8).to_le_bytes();
	let palette_bytes: &[u8] = bytemuck::cast_slice(&palette_vec);

	// LOD collapse
	let mut collapse_rep = compute_lod_collapse(nodes, root_depth, &palette_vec, &palette_map, lod_level);
	collapse_rep[0] = None;

	// Pass 1: DFS pre-order -> gpu_order
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

	// Pass 2: Assign slot indices and voxel buffer offsets
	let mut slot_indices:  Vec<u32> = vec![0; gpu_order.len()];
	let mut voxel_offsets: Vec<u16> = vec![0; gpu_order.len()];
	let mut tree_cursor:  u32 = 0;
	let mut voxel_cursor: u32 = 0; // next free byte in buffer 2

	for (gpu_idx, &(cpu_idx, depth)) in gpu_order.iter().enumerate() {
		let node   = &nodes[cpu_idx as usize];
		let bitmap = build_bitmap(node);

		slot_indices[gpu_idx] = tree_cursor;
		tree_cursor += if depth == 0 { 1 } else { slots_for_nonleaf(bitmap) as u32 };

		let mut offset_back_shift = 0;
		let mut voxel_data_size = 0;
		let mut voxel_node_run = 0;
		let mut hit_data = false;
		for cell in node.contents {
			match cell.value_type() {
				0 => { },
				1 => {
					if hit_data {
						voxel_data_size += 1 + voxel_node_run;
					} else {
						offset_back_shift += voxel_node_run / 4;
						voxel_data_size += 1 + voxel_node_run % 4;
						hit_data = true;
					}
					voxel_node_run = 0;
				},
				2 => {
					let child_cpu = cpu_idx + cell.value() as u32;
					if let Some(_) = collapse_rep[child_cpu as usize] {
						if hit_data {
							voxel_data_size += 1 + voxel_node_run;
						} else {
							offset_back_shift += voxel_node_run / 4;
							voxel_data_size += 1 + voxel_node_run % 4;
							hit_data = true;
						}
						voxel_node_run = 0;
					} else {
						voxel_node_run += 1;
					}
				},
				_ => unreachable!(),
			}
		}

		if voxel_data_size == 0 {
			// No voxels to write; voxel_offsets is never read.
		} else {
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
	let total_voxel_bytes = voxel_cursor as usize; // ends right after the last voxel byte

	// Pass 3: Write bytes
	let mut tree_bytes:  Vec<u8> = vec![0u8; total_tree_slots * SLOT_BYTES];
	let mut voxel_bytes: Vec<u8> = vec![0u8; total_voxel_bytes];

	for (gpu_idx, &(cpu_idx, depth)) in gpu_order.iter().enumerate() {
		let node      = &nodes[cpu_idx as usize];
		let my_slot   = slot_indices[gpu_idx];
		let byte_base = my_slot as usize * SLOT_BYTES;
		let bitmap    = build_bitmap(node);

		// Parent offset (slots)
		let parent_slot_offset: u16 = if node.parent_offset == 0 {
			0
		} else {
			let parent_cpu = cpu_idx - node.parent_offset as u32;
			let parent_slot = slot_indices[cpu_to_gpu[parent_cpu as usize] as usize];
			(my_slot - parent_slot) as u16
		};

		// Header
		tree_bytes[byte_base..byte_base + 2].copy_from_slice(&parent_slot_offset.to_le_bytes());
		tree_bytes[byte_base + 2..byte_base + 4].copy_from_slice(&voxel_offsets[gpu_idx].to_le_bytes());
		tree_bytes[byte_base + 4..byte_base + 12].copy_from_slice(&bitmap.to_le_bytes());

		let mut vox_write = voxel_offsets[gpu_idx] as usize * 4;

		if depth == 0 {
			// Depth-0: no u16 entries in tree buffer; every set bit is a voxel.
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
			// Non-leaf: write u16 entries after header; DATA cells also go to buffer 2.
			let mut entry_cursor = byte_base + HEADER_BYTES;

			let mut voxel_node_run = 0;
			let mut hit_data = false;
			for i in 0..grid_tree::SIZE_USIZE_CUBED {
				if bitmap & (1u64 << i) == 0 { continue; }
				let cell = &node.contents[i];

				let entry_val: u16 = match cell.value_type() {
					1 => {
						if hit_data {
							vox_write += voxel_node_run;
						} else {
							vox_write += voxel_node_run % 4;
							hit_data = true;
						}
						voxel_node_run = 0;
						let pal = palette_map.get(&cell.value()).copied().unwrap_or(254);
						voxel_bytes[vox_write] = pal;
						vox_write += 1;
						0x00
					}
					2 => {
						let child_cpu = cpu_idx + cell.value() as u32;
						if let Some(rep) = collapse_rep[child_cpu as usize] {
							// Collapsed NODE -> DATA
							if hit_data {
								vox_write += voxel_node_run;
							} else {
								vox_write += voxel_node_run % 4;
								hit_data = true;
							}
							voxel_node_run = 0;
							voxel_bytes[vox_write] = rep;
							vox_write += 1;
							0x00
						} else {
							let child_slot = slot_indices[cpu_to_gpu[child_cpu as usize] as usize];
							let offset = child_slot - my_slot;
							voxel_node_run += 1;
							debug_assert!(offset >= 1 && offset <= u16::MAX as u32, "NODE child slot offset {offset} out of u16 range");
							offset as u16
						}
					}
					_ => unreachable!(),
				};

				tree_bytes[entry_cursor..entry_cursor + 2].copy_from_slice(&entry_val.to_le_bytes());
				entry_cursor += 2;
			}
		}
	}

	// Assemble final buffers

	// Buffer 1: [root_depth(1)] [node slots...]
	let tree_raw_len    = 1 + tree_bytes.len();
	let tree_padded_len = tree_raw_len.next_multiple_of(wgpu::COPY_BUFFER_ALIGNMENT as usize);
	let mut tree_buffer = Vec::with_capacity(tree_padded_len);
	tree_buffer.push(root_depth);
	tree_buffer.extend_from_slice(&[0u8; 3]);
	tree_buffer.extend_from_slice(&tree_bytes);
	tree_buffer.resize(tree_padded_len, 0);

	// Buffer 2: [packed voxel palette indices]

	let voxel_raw_len = voxel_bytes.len() + palette_len_bytes.len() + palette_bytes.len();
	let voxel_padded_len = voxel_raw_len.next_multiple_of(wgpu::COPY_BUFFER_ALIGNMENT as usize);
	let mut voxel_buffer = Vec::with_capacity(tree_padded_len);
	voxel_buffer.extend_from_slice(&palette_len_bytes);
	voxel_buffer.extend_from_slice(&palette_bytes);
	voxel_buffer.extend_from_slice(&voxel_bytes);
	voxel_buffer.resize(voxel_padded_len, 0);

	(tree_buffer, voxel_buffer)
}
