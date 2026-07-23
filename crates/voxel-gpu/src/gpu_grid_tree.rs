use tracy_client::span;

use voxel_data::grid_tree::{self, CellKind};
use voxel_data::voxel_grid_tree::{VoxelGridTree, PackedNode};
use voxel_data::voxels::{VoxelRef, VoxelTypeInfo};

use crate::voxel_color::VoxelGpuDataReaders;

const SLOT_BYTES: usize = 4;
const VOXEL_OFFSET_UNIT_BYTES: u32 = 16;

fn get_header_bytes(bitmap: u64) -> u32 {
	4 * (1 + ((bitmap & 0xFFFFFFFF) != 0) as u32 + ((bitmap & 0xFFFFFFFF00000000) != 0) as u32)
}

fn slots_for_nonleaf(bitmap: u64, depth: u8) -> usize {
	let count = bitmap.count_ones() as usize;
	let entry_bytes = if depth == 1 { 1 } else { 2 };
	( get_header_bytes(bitmap) as usize + count * entry_bytes).div_ceil(SLOT_BYTES)
}

fn build_bitmap(node: &PackedNode<'_>) -> u64 {
	let mut bitmap = 0u64;
	for i in 0..grid_tree::SIZE_CUBED {
		if node.kind(i) != CellKind::Empty {
			bitmap |= 1u64 << i;
		}
	}
	bitmap
}

fn collect_voxel_refs<'a>(nodes: &[PackedNode<'a>], gpu_order: &[(u32, u8)], voxel_type: VoxelTypeInfo) -> Vec<VoxelRef<'a>> {
	let data_cell_count = gpu_order
		.iter()
		.map(|&(cpu_idx, _)| nodes[cpu_idx as usize].data_mask().count_ones() as usize)
		.sum();
	let mut voxels = Vec::with_capacity(data_cell_count);

	for &(cpu_idx, _) in gpu_order {
		let node = &nodes[cpu_idx as usize];
		for i in 0..grid_tree::SIZE_USIZE_CUBED {
			let child_i = i as u8;
			if node.kind(child_i) == CellKind::Data {
				voxels.push(VoxelRef::new(voxel_type.id, node.cell_bytes(child_i)));
			}
		}
	}

	voxels
}

pub fn make_gpu_grid_tree(grid_tree: &VoxelGridTree, voxel_type: VoxelTypeInfo, gpu_data_readers: &VoxelGpuDataReaders) -> (Vec<u8>, Vec<u8>) {
	let _zone = span!("make GPU grid tree");
	let view = grid_tree.view();
	let nodes = view.nodes();
	let root_depth = view.root_depth();
	assert!(!nodes.is_empty(), "ERROR: tree must have at least a root node.");
	let voxel_stride = gpu_data_readers.gpu_size_bytes(voxel_type.id).unwrap_or(voxel_type.size_bytes as usize);
	let voxel_stride_bytes = voxel_stride as u32;
	assert!(voxel_stride_bytes > 0, "voxel GPU data stride must be non-zero");

	// -- Pass 1: DFS pre-order → gpu_order ------------------------------------
	let mut cpu_to_gpu: Vec<u32>       = vec![u32::MAX; nodes.len()];
	let mut gpu_order:  Vec<(u32, u8)> = Vec::with_capacity(nodes.len());

	let mut dfs_stack: Vec<(u32, u8)> = vec![(0, root_depth)];
	while let Some((cpu_idx, depth)) = dfs_stack.pop() {
		let gpu_idx = gpu_order.len() as u32;
		cpu_to_gpu[cpu_idx as usize] = gpu_idx;
		gpu_order.push((cpu_idx, depth));
		if depth > 0 {
			let node = nodes[cpu_idx as usize];
			for i in 0..grid_tree::SIZE_CUBED {
				if node.kind(i) == CellKind::Node {
					let child_cpu = node.child_index(i).expect("node cell has child");
					dfs_stack.push((child_cpu, depth - 1));
				}
			}
		}
	}

	// -- Find separating plane -------------------------------------------------
	//
	// Encoding (bytes 1-3 of tree_buffer header):
	//   nx, ny, nz : 4-bit each  –  actual = encoded − 7.5
	//   m          : 8-bit       –  actual = encoded / 32.0 − 3.984375
	//
	// Solid halfspace:  dot(n, pos − root_centre) <= m_actual * root_size

	// const TREE_LOG_SIZE: u32 = 2;
	// let root_size_f = (1u32 << (TREE_LOG_SIZE * (root_depth as u32 + 1))) as f32;
	// let root_half   = root_size_f / 2.0;

	// Collect leaf-level cells with their centres (root-relative) and half-extent.
	// struct TaggedCell { center_x: f32, center_y: f32, center_z: f32, half: f32, solid: bool }
	// let mut tagged_cells: Vec<TaggedCell> = Vec::new();
	// {
	// 	let mut stack: Vec<(u32, u8, [u32; 3])> = vec![(0u32, root_depth, [0u32; 3])];
	// 	while let Some((cpu_idx, depth, node_origin)) = stack.pop() {
	// 		let node_size = 1u32 << (TREE_LOG_SIZE * (depth as u32 + 1));
	// 		let cell_size = node_size >> TREE_LOG_SIZE;
	// 		let half      = cell_size as f32 * 0.5;
	// 		let node      = &nodes[cpu_idx as usize];

	// 		for (i, cell) in node.contents.iter().enumerate() {
	// 			let cell_x = (i % grid_tree::SIZE_USIZE) as u32;
	// 			let cell_y = ((i / grid_tree::SIZE_USIZE) % grid_tree::SIZE_USIZE) as u32;
	// 			let cell_z = (i / (grid_tree::SIZE_USIZE * grid_tree::SIZE_USIZE)) as u32;
	// 			let center_x = node_origin[0] as f32 + cell_x as f32 * cell_size as f32 + half - root_half;
	// 			let center_y = node_origin[1] as f32 + cell_y as f32 * cell_size as f32 + half - root_half;
	// 			let center_z = node_origin[2] as f32 + cell_z as f32 * cell_size as f32 + half - root_half;
	// 			let solid = match cell.kind() {
	// 				CellKind::Empty => false,
	// 				CellKind::Data => true,
	// 				CellKind::Node => {
	// 					let child_cpu = cpu_idx + cell.node_index();
	// 					stack.push((child_cpu, depth - 1, [
	// 						node_origin[0] + cell_x * cell_size,
	// 						node_origin[1] + cell_y * cell_size,
	// 						node_origin[2] + cell_z * cell_size,
	// 					]));
	// 					continue;
	// 				}
	// 			};
	// 			tagged_cells.push(TaggedCell { center_x, center_y, center_z, half, solid });
	// 		}
	// 	}
	// }

	// (ox, oy, oz, half)
	// let mut solid_cells: Vec<parry3d::math::Vec3> = vec![];
	// for cell in &tagged_cells {
	// 	if cell.solid {
	// 		solid_cells.push(parry3d::math::Vec3::new(cell.center_x - cell.half, cell.center_y - cell.half, cell.center_z - cell.half));
	// 		solid_cells.push(parry3d::math::Vec3::new(cell.center_x + cell.half, cell.center_y - cell.half, cell.center_z - cell.half));
	// 		solid_cells.push(parry3d::math::Vec3::new(cell.center_x - cell.half, cell.center_y + cell.half, cell.center_z - cell.half));
	// 		solid_cells.push(parry3d::math::Vec3::new(cell.center_x - cell.half, cell.center_y - cell.half, cell.center_z + cell.half));
	// 		solid_cells.push(parry3d::math::Vec3::new(cell.center_x + cell.half, cell.center_y + cell.half, cell.center_z - cell.half));
	// 		solid_cells.push(parry3d::math::Vec3::new(cell.center_x + cell.half, cell.center_y - cell.half, cell.center_z + cell.half));
	// 		solid_cells.push(parry3d::math::Vec3::new(cell.center_x - cell.half, cell.center_y + cell.half, cell.center_z + cell.half));
	// 		solid_cells.push(parry3d::math::Vec3::new(cell.center_x + cell.half, cell.center_y + cell.half, cell.center_z + cell.half));
	// 	}
	// }
	// let (solid_cells_reduced, _indices) = parry3d::transformation::convex_hull(&solid_cells);

	// let mut rng = rand::rng();
	// let empty_cells: Vec<(f32, f32, f32, f32)> = {
	// 	let empty_cells = tagged_cells.into_iter().filter(|c| !c.solid).map(|c| (c.center_x, c.center_y, c.center_z, c.half)).collect::<Vec<_>>();
	// 	let empty_cells_len = empty_cells.len();
	// 	if empty_cells_len > 100 {
	// 		empty_cells.into_iter().sample(&mut rng, 100)
	// 	} else {
	// 		empty_cells
	// 	}
	// };

	let best_plane: Option<(u8, u8, u8, u8)> = None; /*if solid_cells.is_empty() {
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
					let m_enc_f = (actual_m + 3.984375) * 32.0;
					if !(0.0..=255.0).contains(&m_enc_f) { continue; }

					let m_enc_ceiled = m_enc_f.ceil();
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
	};*/

	// -- Pass 2: Assign slot indices and voxel buffer offsets -----------------
	let mut slot_indices:  Vec<u32> = vec![0; gpu_order.len()];
	let mut voxel_offsets: Vec<u16> = vec![0; gpu_order.len()];
	let mut tree_cursor:  u32 = 0;
	let mut voxel_cursor: u32 = 0;

	for (gpu_idx, &(cpu_idx, depth)) in gpu_order.iter().enumerate() {
		let node = &nodes[cpu_idx as usize];
		let bitmap = build_bitmap(node);
		assert!(bitmap != 0);

		slot_indices[gpu_idx] = tree_cursor;
		tree_cursor += if depth == 0 { get_header_bytes(bitmap) / 4 } else { slots_for_nonleaf(bitmap, depth) as u32 };

		let mut voxel_data_size = 0u32;
		let mut voxel_node_run = 0u32;

		for i in 0..grid_tree::SIZE_CUBED {
			match node.kind(i) {
				CellKind::Empty => {}
				CellKind::Data => {
					voxel_data_size += voxel_stride_bytes * (1 + voxel_node_run);
					voxel_node_run = 0;
				}
				CellKind::Node => {
					voxel_node_run += 1;
				}
			}
		}

		if voxel_data_size > 0 {
			let aligned = voxel_cursor.next_multiple_of(VOXEL_OFFSET_UNIT_BYTES);
			debug_assert!(
				aligned / VOXEL_OFFSET_UNIT_BYTES <= u16::MAX as u32,
				"voxel_data_offset overflow at node {gpu_idx}: buffer 2 exceeds 1 MiB"
			);
			voxel_offsets[gpu_idx] = (aligned / VOXEL_OFFSET_UNIT_BYTES) as u16;
			voxel_cursor = aligned + voxel_data_size;
		}
	}

	let total_tree_slots = tree_cursor as usize;
	let total_voxel_bytes = voxel_cursor as usize;

	let voxel_refs = collect_voxel_refs(&nodes, &gpu_order, voxel_type);
	let mut voxel_payload_bytes = vec![0u8; voxel_refs.len() * voxel_stride];
	if gpu_data_readers.write_bytes(voxel_type.id, &voxel_refs, &mut voxel_payload_bytes).is_none() {
		for (voxel, out) in voxel_refs.iter().zip(voxel_payload_bytes.chunks_exact_mut(voxel_stride)) {
			out.copy_from_slice(&voxel.bytes()[..voxel_stride]);
		}
	}
	let mut voxel_payloads = voxel_payload_bytes.chunks_exact(voxel_stride);

	// -- Pass 3: Write bytes ---------------------------------------------------
	let mut tree_bytes: Vec<u8> = vec![0u8; total_tree_slots * SLOT_BYTES];
	let mut voxel_bytes: Vec<u8> = vec![0u8; total_voxel_bytes];

	for (gpu_idx, &(cpu_idx, depth)) in gpu_order.iter().enumerate() {
		let node = &nodes[cpu_idx as usize];
		let my_slot = slot_indices[gpu_idx];
		let byte_base = my_slot as usize * SLOT_BYTES;
		let bitmap = build_bitmap(node);

		let parent_slot_offset: u16 = if node.parent_offset() == 0 {
			0
		} else {
			let parent_cpu = cpu_idx - node.parent_offset();
			let parent_slot = slot_indices[cpu_to_gpu[parent_cpu as usize] as usize];
			(my_slot - parent_slot) as u16
		};
		assert!(parent_slot_offset <= 0x3FFF);
		tree_bytes[byte_base..byte_base + 2].copy_from_slice(&(parent_slot_offset & 0x3FFF).to_le_bytes());
		tree_bytes[byte_base + 2..byte_base + 4].copy_from_slice(&voxel_offsets[gpu_idx].to_le_bytes());
		let has_lo = (bitmap & 0xFFFFFFFF) != 0;
		let has_hi = (bitmap & 0xFFFFFFFF00000000) != 0;
		assert!(has_hi || has_lo);
		let flags: u8 = (has_lo as u8) | ((has_hi as u8) << 1);
		tree_bytes[byte_base + 1] |= flags << 6;
		let mut bitmap_cursor = byte_base + 4;
		if has_lo {
			tree_bytes[bitmap_cursor..bitmap_cursor + 4].copy_from_slice(&(bitmap as u32).to_le_bytes());
			bitmap_cursor += 4;
		}
		if has_hi {
			tree_bytes[bitmap_cursor..bitmap_cursor + 4].copy_from_slice(&((bitmap >> 32) as u32).to_le_bytes());
		}

		let mut vox_write = voxel_offsets[gpu_idx] as usize * VOXEL_OFFSET_UNIT_BYTES as usize;

		if depth == 0 {
			for i in 0..grid_tree::SIZE_USIZE_CUBED {
				if bitmap & (1u64 << i) == 0 { continue; }
				let payload = match node.kind(i as u8) {
					CellKind::Data => voxel_payloads.next().expect("voxel payload count must match data cells"),
					_ => unreachable!("depth-0 cell must be DATA"),
				};
				voxel_bytes[vox_write..vox_write + voxel_stride].copy_from_slice(payload);
				vox_write += voxel_stride;
			}
		} else {
			let mut entry_cursor = byte_base + get_header_bytes(bitmap) as usize;
			let mut voxel_node_run = 0usize;

			for i in 0..grid_tree::SIZE_USIZE_CUBED {
				if bitmap & (1u64 << i) == 0 { continue; }
				let child_i = i as u8;

				let entry_val: u16 = match node.kind(child_i) {
					CellKind::Data => {
						vox_write += voxel_node_run * voxel_stride;
						voxel_node_run = 0;
						let payload = voxel_payloads.next().expect("voxel payload count must match data cells");
						voxel_bytes[vox_write..vox_write + voxel_stride].copy_from_slice(payload);
						vox_write += voxel_stride;
						0x00
					}
					CellKind::Node => {
						let child_cpu = node.child_index(child_i).expect("node cell has child");
						let child_slot = slot_indices[cpu_to_gpu[child_cpu as usize] as usize];
						let offset = child_slot - my_slot;
						voxel_node_run += 1;
						debug_assert!(
							offset >= 1 && offset <= if depth == 1 { u8::MAX as u32 } else { u16::MAX as u32 },
							"NODE child slot offset {offset} out of range"
						);
						offset as u16
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
	debug_assert!(voxel_payloads.next().is_none(), "unused voxel payloads after GPU grid tree write");

	// -- Assemble final buffers ------------------------------------------------

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

	let voxel_raw_len    = voxel_bytes.len();
	let voxel_padded_len = voxel_raw_len.next_multiple_of(wgpu::COPY_BUFFER_ALIGNMENT as usize);
	let mut voxel_buffer = Vec::with_capacity(voxel_padded_len);
	voxel_buffer.extend_from_slice(&voxel_bytes);
	voxel_buffer.resize(voxel_padded_len, 0);

	(tree_buffer, voxel_buffer)
}
