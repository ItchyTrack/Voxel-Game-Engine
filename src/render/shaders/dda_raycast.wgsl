const DDA_LOG_SIZE:   u32 = 2u;
const DDA_SIZE:       u32 = 4u;
const DDA_SLOT_BYTES: u32 = 12u;

// Buffer layout:
//  tree_buf:  [root_depth: u32 (1 byte + 3 pad)] [node slots: 12 bytes each]
//  voxel_buf: [packed u8 palette indices]
//
// Node slot (12 bytes):
//  bytes 0..1 : u16 parent_offset     (slots back to parent; 0 = root)
//  bytes 2..3 : u16 voxel_data_offset (byte offset into voxel_buf = value * 4)
//  bytes 4..11: u64 bitmap            (bit i = 1 iff cell i is DATA or NODE)
//
// Non-leaf nodes (depth > 0): u16 entries packed after the 12-byte header
//  0x0000        -> DATA  (voxel; index in voxel_buf via popcount)
//  0x0001..FFFF  -> NODE  (child slot offset from this node's slot index)
//
// Depth-0 nodes: no entries after header; every set bitmap bit is a voxel.

@group(2) @binding(0) var<storage, read> grid_tree_buf: array<u32>;

fn dda_u8(byte_off: u32) -> u32 {
	return (grid_tree_buf[byte_off / 4u] >> ((byte_off % 4u) * 8u)) & 0xFFu;
}
fn dda_u16(byte_off: u32) -> u32 {
	return (grid_tree_buf[byte_off / 4u] >> ((byte_off % 4u) * 8u)) & 0xFFFFu;
}
fn dda_u64_lo(byte_off: u32) -> u32 { return grid_tree_buf[byte_off / 4u]; }
fn dda_u64_hi(byte_off: u32) -> u32 { return grid_tree_buf[byte_off / 4u + 1u]; }

fn dda_root_depth(tree_base: u32) -> u32 {
	return dda_u8(tree_base); // root_depth is stored in the first byte; 3 bytes of padding follow
}
fn dda_nodes_base(tree_base: u32) -> u32 {
	return tree_base + 4u; // 4-byte aligned: 1 byte root_depth + 3 bytes pad
}
fn dda_node_byte_off(nodes_base: u32, slot_index: u32) -> u32 {
	return nodes_base + slot_index * DDA_SLOT_BYTES;
}
fn dda_parent_offset(node_byte_off: u32) -> u32 {
	return dda_u16(node_byte_off + 0u);
}
fn dda_voxel_data_offset(node_byte_off: u32) -> u32 {
	return dda_u16(node_byte_off + 2u);
}
fn dda_bitmap_lo(node_byte_off: u32) -> u32 {
	return dda_u64_lo(node_byte_off + 4u);
}
fn dda_bitmap_hi(node_byte_off: u32) -> u32 {
	return dda_u64_hi(node_byte_off + 4u);
}
fn dda_bitmap_has(lo: u32, hi: u32, cell_index: u32) -> bool {
	if cell_index < 32u {
		return (lo & (1u << cell_index)) != 0u;
	} else {
		return (hi & (1u << (cell_index - 32u))) != 0u;
	}
}
fn dda_data_index(low: u32, high: u32, cell_index: u32) -> u32 {
	let mask = (1u << (cell_index & 31u)) - 1u;
	if cell_index >= 32u {
		return countOneBits(low) + countOneBits(high & mask);
	} else {
		return countOneBits(low & mask);
	}
}
// Non-leaf data entry: 0x0000 = DATA, 1..0xFFFF = NODE slot offset
fn dda_node_entry(node_byte_off: u32, data_idx: u32) -> u32 {
	return dda_u16(node_byte_off + 12u + data_idx * 2u);
}

fn dda_node_size(depth: u32) -> u32  { return 1u << (DDA_LOG_SIZE * (depth + 1u)); }

fn get_child_contents_index(contents_pos: vec3<u32>) -> u32 {
	return contents_pos.x + contents_pos.y * DDA_SIZE + contents_pos.z * DDA_SIZE * DDA_SIZE;
}

struct DDAHit {
	normal: u32, // if 0 then no hit
	voxel_data_index: u32,      // slot index of the node containing the hit voxel
	voxel_index: u32,      // cell index within that node (use popcount(bitmap & mask) for voxel_buf index)
	dist: f32,
}

fn dda_raycast(
	origin:     vec3<f32>,
	dir:        vec3<f32>,
	max_length: f32,
	tree_base:  u32,
) -> DDAHit {
	let root_size = dda_node_size(dda_root_depth(tree_base));

	// Origin is expected to be in root-relative space (no root_pos offset).
	let root_min = vec3<f32>(0.0);
	let root_max = vec3<f32>(f32(root_size));

	let inv_dir = vec3<f32>(1.0) / dir;
	let t1      = (root_min - origin) * inv_dir;
	let t2      = (root_max - origin) * inv_dir;
	let tmin    = max(max(min(t1.x, t2.x), min(t1.y, t2.y)), min(t1.z, t2.z));
	let tmax    = min(min(max(t1.x, t2.x), max(t1.y, t2.y)), max(t1.z, t2.z));
	if tmin > tmax || tmax < 0.0 {
		var no_hit: DDAHit;
		no_hit.normal = 0u;;
		return no_hit;
	}
	let distance_to_aabb = max(tmin, 0.0);
	if distance_to_aabb > max_length {
		var no_hit: DDAHit;
		no_hit.normal = 0u;;
		return no_hit;
	}
	let delta = abs(inv_dir);
	let step = u32(dir.x >= 0.0) + u32(dir.y >= 0.0) * 2 + u32(dir.z >= 0.0) * 4;

	let post_aabb_origin_pre_shift = origin + dir * distance_to_aabb;
	let post_aabb_origin_clamped   = clamp(
		post_aabb_origin_pre_shift,
		root_min,
		vec3<f32>(f32(root_size) - 0.00001),
	);
	let move_towards_target = floor(post_aabb_origin_clamped) + vec3<f32>(0.5);
	let move_towards_delta  = move_towards_target - post_aabb_origin_clamped;
	let move_towards_length = length(move_towards_delta);
	let post_aabb_origin = select(
		post_aabb_origin_clamped + move_towards_delta / move_towards_length * 0.001,
		post_aabb_origin_clamped,
		move_towards_length < 0.001,
	);

	let root_relative_post_aabb_origin = post_aabb_origin;

	let axis_distances_boundary = vec3<f32>(
		select(floor(root_relative_post_aabb_origin.x), ceil(root_relative_post_aabb_origin.x), (step & 1) != 0),
		select(floor(root_relative_post_aabb_origin.y), ceil(root_relative_post_aabb_origin.y), (step & 2) != 0),
		select(floor(root_relative_post_aabb_origin.z), ceil(root_relative_post_aabb_origin.z), (step & 4) != 0),
	);
	var axis_distances = inv_dir * (axis_distances_boundary - root_relative_post_aabb_origin)
		+ vec3<f32>(distance_to_aabb, distance_to_aabb, distance_to_aabb);

	var root_relative_grid_pos = vec3<u32>(vec3<i32>(root_relative_post_aabb_origin));

	var last_step_axis: u32;
	let pre_shift_delta = abs(post_aabb_origin_pre_shift - post_aabb_origin);
	if pre_shift_delta.x >= pre_shift_delta.y && pre_shift_delta.x >= pre_shift_delta.z { last_step_axis = 0u; }
	else if pre_shift_delta.y >= pre_shift_delta.z                                      { last_step_axis = 1u; }
	else                                                                                { last_step_axis = 2u; }

	if max_length < distance_to_aabb {
		var no_hit: DDAHit;
		no_hit.normal = 0u;;
		return no_hit;
	}

	let nodes_base = dda_nodes_base(tree_base);
	var node_size = root_size;
	var node_byte_off = dda_node_byte_off(nodes_base, 0u);
	var bitmap_low = dda_bitmap_lo(node_byte_off);
	var bitmap_high = dda_bitmap_hi(node_byte_off);

	loop {
		let current_relative_pos = root_relative_grid_pos & vec3<u32>(node_size - 1u);
		let contents_pos         = current_relative_pos / vec3<u32>(node_size >> DDA_LOG_SIZE);
		let cell_index           = get_child_contents_index(contents_pos);

		if !dda_bitmap_has(bitmap_low, bitmap_high, cell_index) {
			// EMPTY: step forward
			if node_size != 4u {
				let node_child_size = node_size >> DDA_LOG_SIZE;
				let node_cell_relative_grid_pos = root_relative_grid_pos & vec3<u32>(node_child_size - 1u);
				let cell_step_amount = vec3<u32>(
					select(node_cell_relative_grid_pos.x, node_child_size - node_cell_relative_grid_pos.x - 1u, (step & 1) != 0),
					select(node_cell_relative_grid_pos.y, node_child_size - node_cell_relative_grid_pos.y - 1u, (step & 2) != 0),
					select(node_cell_relative_grid_pos.z, node_child_size - node_cell_relative_grid_pos.z - 1u, (step & 4) != 0),
				);

				let axis_distances_at_cell_edge = axis_distances + delta * vec3<f32>(cell_step_amount);
				var dis_to_edge = min(min(axis_distances_at_cell_edge.x, axis_distances_at_cell_edge.y), axis_distances_at_cell_edge.z);
				var adjusted_step_amount = vec3<u32>(abs((vec3<f32>(dis_to_edge, dis_to_edge, dis_to_edge) - axis_distances + delta) / delta));

				if      axis_distances_at_cell_edge.x <= axis_distances_at_cell_edge.y &&
						axis_distances_at_cell_edge.x <= axis_distances_at_cell_edge.z { adjusted_step_amount.x = cell_step_amount.x; }
				else if axis_distances_at_cell_edge.y <= axis_distances_at_cell_edge.z { adjusted_step_amount.y = cell_step_amount.y; }
				else                                                                   { adjusted_step_amount.z = cell_step_amount.z; }

				axis_distances += delta * vec3<f32>(adjusted_step_amount);
				root_relative_grid_pos = vec3<u32>(vec3<i32>(root_relative_grid_pos) + vec3<i32>(adjusted_step_amount) * vec3<i32>(
					i32(step & 1) * 2 - 1, i32(step & 2) - 1, i32(step & 4) / 2 - 1
				));
			}

			last_step_axis = select(select(2u, 1u, axis_distances.y <= axis_distances.z), 0u, axis_distances.x <= axis_distances.y && axis_distances.x <= axis_distances.z);
			if max_length < axis_distances[last_step_axis] {
				var no_hit: DDAHit;
				no_hit.normal = 0u;;
				return no_hit;
			}
			axis_distances[last_step_axis] += delta[last_step_axis];

			let new_pos_signed = i32(root_relative_grid_pos[last_step_axis]) + (i32((step & (1u << last_step_axis)) != 0) * 2 - 1);
			if new_pos_signed < 0 || u32(new_pos_signed) >= root_size {
				var no_hit: DDAHit;
				no_hit.normal = 0u;;
				return no_hit;
			}

			if root_relative_grid_pos[last_step_axis] / node_size != u32(new_pos_signed) / node_size {
				loop {
					if node_size == root_size {
						var no_hit: DDAHit;
						no_hit.normal = 0u;;
						return no_hit;
					}
					node_byte_off -= dda_parent_offset(node_byte_off) * DDA_SLOT_BYTES;
					node_size = node_size << DDA_LOG_SIZE;
					if root_relative_grid_pos[last_step_axis] / node_size == u32(new_pos_signed) / node_size { break; }
				}
				bitmap_low = dda_bitmap_lo(node_byte_off);
				bitmap_high = dda_bitmap_hi(node_byte_off);
			}
			root_relative_grid_pos[last_step_axis] = u32(new_pos_signed);
		} else {
			let data_idx  = dda_data_index(bitmap_low, bitmap_high, cell_index);
			if node_size == 4u {
				// Depth-0: every set bitmap bit is a voxel - immediate hit.
				var hit_result: DDAHit;
				hit_result.normal = step + (last_step_axis << 3u) + (1 << 10); // make normal non 0 and encode the normal in it
				hit_result.voxel_data_index = dda_voxel_data_offset(node_byte_off);
				hit_result.voxel_index = data_idx;
				hit_result.dist = axis_distances[last_step_axis] - delta[last_step_axis];
				return hit_result;
			}

			let entry_val = dda_node_entry(node_byte_off, data_idx);

			if entry_val == 0u {
				// DATA: hit
				var hit_result: DDAHit;
				hit_result.normal = step + (last_step_axis << 3u) + (1 << 10); // make normal non 0 and encode the normal in it
				hit_result.voxel_data_index = dda_voxel_data_offset(node_byte_off);
				hit_result.voxel_index = data_idx;
				hit_result.dist = axis_distances[last_step_axis] - delta[last_step_axis];
				return hit_result;
			} else {
				// NODE: descend
				node_byte_off += entry_val * DDA_SLOT_BYTES;
				node_size = node_size >> DDA_LOG_SIZE;
				bitmap_low = dda_bitmap_lo(node_byte_off);
				bitmap_high = dda_bitmap_hi(node_byte_off);
			}
		}
	}
	var no_hit: DDAHit;
	no_hit.normal = 0u;;
	return no_hit;
}
