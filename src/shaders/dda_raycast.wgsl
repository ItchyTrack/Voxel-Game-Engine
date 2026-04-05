const DDA_LOG_SIZE:           u32 = 2u;
const DDA_SIZE:               u32 = 4u;
const DDA_SLOT_BYTES:         u32 = 16u;
const DDA_ROOT_BYTES:         u32 = 6u;
const DDA_ROOT_DEPTH_BYTES:   u32 = 1u;
const DDA_PALETTE_LEN_BYTES:  u32 = 1u;
const DDA_COLOR_BYTES:        u32 = 4u;

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
	return dda_u8(tree_base + DDA_ROOT_BYTES);
}
fn dda_palette_len(tree_base: u32) -> u32 {
	return dda_u8(tree_base + DDA_ROOT_BYTES + DDA_ROOT_DEPTH_BYTES);
}
fn dda_nodes_base(tree_base: u32) -> u32 {
	return tree_base
		+ DDA_ROOT_BYTES
		+ DDA_ROOT_DEPTH_BYTES
		+ DDA_PALETTE_LEN_BYTES
		+ dda_palette_len(tree_base) * DDA_COLOR_BYTES;
}
fn dda_read_root_pos(tree_base: u32) -> vec3<i32> {
	let word_zero = grid_tree_buf[tree_base / 4u];
	let word_one  = grid_tree_buf[(tree_base + 4u) / 4u];
	var root_x = i32(word_zero & 0xFFFFu);          if (root_x & 0x8000) != 0 { root_x -= 0x10000; }
	var root_y = i32((word_zero >> 16u) & 0xFFFFu); if (root_y & 0x8000) != 0 { root_y -= 0x10000; }
	var root_z = i32(word_one & 0xFFFFu);            if (root_z & 0x8000) != 0 { root_z -= 0x10000; }
	return vec3<i32>(root_x, root_y, root_z);
}
fn dda_palette_color(tree_base: u32, palette_index: u32) -> vec4<f32> {
	let base = tree_base + DDA_ROOT_BYTES + DDA_ROOT_DEPTH_BYTES + DDA_PALETTE_LEN_BYTES;
	let off  = base + palette_index * DDA_COLOR_BYTES;
	return vec4<f32>(
		f32(dda_u8(off + 0u)) / 255.0,
		f32(dda_u8(off + 1u)) / 255.0,
		f32(dda_u8(off + 2u)) / 255.0,
		f32(dda_u8(off + 3u)) / 255.0,
	);
}

fn dda_node_byte_off(tree_base: u32, slot_index: u32) -> u32 {
	return dda_nodes_base(tree_base) + slot_index * DDA_SLOT_BYTES;
}
fn dda_bitmap_lo(node_byte_off: u32) -> u32 {
	return dda_u64_lo(node_byte_off);
}
fn dda_bitmap_hi(node_byte_off: u32) -> u32 {
	return dda_u64_hi(node_byte_off);
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
fn dda_nonleaf_data(node_byte_off: u32, data_idx: u32) -> u32 {
	let byte_off = node_byte_off + 10u + data_idx * 2u;
	return dda_u16(byte_off);
}
fn dda_leaf_data(node_byte_off: u32, data_idx: u32) -> u32 {
	let byte_off = node_byte_off + 9u + data_idx;
	return dda_u8(byte_off);
}
fn dda_nonleaf_parent_offset(node_byte_off: u32) -> u32 {
	return dda_u16(node_byte_off + 8u);
}
fn dda_leaf_parent_offset(node_byte_off: u32) -> u32 {
	return dda_u8(node_byte_off + 8u);
}

fn dda_node_size(depth: u32) -> u32 { return 1u << (DDA_LOG_SIZE * (depth + 1u)); }
fn dda_child_size(depth: u32) -> u32 { return 1u << (DDA_LOG_SIZE * depth); }

fn dda_cell_type(raw_value: u32) -> u32 {
	return raw_value >> 15u;
}
fn dda_cell_value(raw_value: u32) -> u32 { return raw_value & 0x7FFFu; }

fn get_child_contents_index(contents_pos: vec3<u32>) -> u32 {
	return contents_pos.x + contents_pos.y * DDA_SIZE + contents_pos.z * DDA_SIZE * DDA_SIZE;
}

fn dda_read_cell(
	node_byte_off:	u32,
	cell_index: 	u32,
	is_leaf:    	bool,
	bitmap_lo:  	u32,
	bitmap_hi:  	u32,
) -> u32 {
	let data_idx = dda_data_index(bitmap_lo, bitmap_hi, cell_index);
	if is_leaf {
		return dda_leaf_data(node_byte_off, data_idx);
	} else {
		return dda_nonleaf_data(node_byte_off, data_idx);
	}
}

struct DDAHit {
	hit:    bool,
	value:  u32,
	normal: vec3<f32>,
	dist:   f32,
}

fn dda_raycast(
	origin:     vec3<f32>,
	dir:        vec3<f32>,
	max_length: f32,
	tree_base:  u32,
) -> DDAHit {
	var no_hit: DDAHit;
	no_hit.hit = false;

	let root_pos_i32  = dda_read_root_pos(tree_base);
	let root_pos      = vec3<f32>(root_pos_i32);
	let root_depth    = dda_root_depth(tree_base);
	let root_size_u32 = dda_node_size(root_depth);
	let root_size     = f32(root_size_u32);
	let root_min      = root_pos;
	let root_max      = root_pos + vec3<f32>(root_size);

	let inv_dir = vec3<f32>(1.0) / dir;
	let t1      = (root_min - origin) * inv_dir;
	let t2      = (root_max - origin) * inv_dir;
	let tmin    = max(max(min(t1.x, t2.x), min(t1.y, t2.y)), min(t1.z, t2.z));
	let tmax    = min(min(max(t1.x, t2.x), max(t1.y, t2.y)), max(t1.z, t2.z));
	if tmin > tmax || tmax < 0.0 { return no_hit; }

	let distance_to_aabb = max(tmin, 0.0);
	if distance_to_aabb > max_length { return no_hit; }

	let post_aabb_origin_pre_shift = origin + dir * distance_to_aabb;
	let post_aabb_origin_clamped   = clamp(
		post_aabb_origin_pre_shift,
		root_pos,
		root_pos + vec3<f32>(root_size - 0.00001),
	);
	let move_towards_target = floor(post_aabb_origin_clamped) + vec3<f32>(0.5);
	let move_towards_delta  = move_towards_target - post_aabb_origin_clamped;
	let move_towards_length = length(move_towards_delta);
	let post_aabb_origin = select(
		post_aabb_origin_clamped + move_towards_delta / move_towards_length * 0.001,
		post_aabb_origin_clamped,
		move_towards_length < 0.001,
	);

	let root_relative_post_aabb_origin = post_aabb_origin - root_pos;

	let delta = abs(inv_dir);

	let step = vec3<i32>(
		select(-1, 1, dir.x >= 0.0),
		select(-1, 1, dir.y >= 0.0),
		select(-1, 1, dir.z >= 0.0),
	);

	let axis_distances_boundary = vec3<f32>(
		select(floor(root_relative_post_aabb_origin.x), ceil(root_relative_post_aabb_origin.x), step.x > 0),
		select(floor(root_relative_post_aabb_origin.y), ceil(root_relative_post_aabb_origin.y), step.y > 0),
		select(floor(root_relative_post_aabb_origin.z), ceil(root_relative_post_aabb_origin.z), step.z > 0),
	);
	var axis_distances = inv_dir * (axis_distances_boundary - root_relative_post_aabb_origin)
		+ vec3<f32>(distance_to_aabb, distance_to_aabb, distance_to_aabb);

	var root_relative_grid_pos = vec3<u32>(vec3<i32>(root_relative_post_aabb_origin));

	var last_step_axis: u32;
	let pre_shift_delta = abs(post_aabb_origin_pre_shift - post_aabb_origin);
	if pre_shift_delta.x >= pre_shift_delta.y && pre_shift_delta.x >= pre_shift_delta.z { last_step_axis = 0u; }
	else if pre_shift_delta.y >= pre_shift_delta.z                                      { last_step_axis = 1u; }
	else                                                                                { last_step_axis = 2u; }

	var current_node_index: u32 = 0u;
	var last_distance = distance_to_aabb;
	if max_length < last_distance { return no_hit; }

	var current_depth = root_depth;

	var node_byte_off = dda_nodes_base(tree_base);
	var bitmap_low = dda_bitmap_lo(node_byte_off);
	var bitmap_high = dda_bitmap_hi(node_byte_off);

	loop {
		var node_size:  u32  	 	= dda_node_size(current_depth);
		var node_child_size: u32    = node_size >> DDA_LOG_SIZE;
		let current_relative_pos	= root_relative_grid_pos & vec3<u32>(node_size - 1u);
		let contents_pos        	= current_relative_pos / vec3<u32>(node_child_size);
		let cell_index          	= get_child_contents_index(contents_pos);

		if !dda_bitmap_has(bitmap_low, bitmap_high, cell_index) {
			// EMPTY: step forward
			if node_child_size != 1u {
				let node_cell_relative_grid_pos = root_relative_grid_pos & vec3<u32>(node_child_size - 1u);
				let cell_step_amount = vec3<u32>(
					select(node_cell_relative_grid_pos.x, node_child_size - node_cell_relative_grid_pos.x - 1u, step.x > 0),
					select(node_cell_relative_grid_pos.y, node_child_size - node_cell_relative_grid_pos.y - 1u, step.y > 0),
					select(node_cell_relative_grid_pos.z, node_child_size - node_cell_relative_grid_pos.z - 1u, step.z > 0),
				);

				let axis_distances_at_cell_edge = axis_distances + delta * vec3<f32>(cell_step_amount);
				var dis_to_edge = min(min(axis_distances_at_cell_edge.x, axis_distances_at_cell_edge.y), axis_distances_at_cell_edge.z);
				var adjusted_step_amount = vec3<u32>(abs((vec3<f32>(dis_to_edge, dis_to_edge, dis_to_edge) - axis_distances + delta) / delta));

				if      axis_distances_at_cell_edge.x <= axis_distances_at_cell_edge.y &&
						axis_distances_at_cell_edge.x <= axis_distances_at_cell_edge.z { adjusted_step_amount.x = cell_step_amount.x; }
				else if axis_distances_at_cell_edge.y <= axis_distances_at_cell_edge.z { adjusted_step_amount.y = cell_step_amount.y; }
				else                                                                   { adjusted_step_amount.z = cell_step_amount.z; }

				axis_distances += delta * vec3<f32>(adjusted_step_amount);
				root_relative_grid_pos = vec3<u32>(vec3<i32>(root_relative_grid_pos) + vec3<i32>(adjusted_step_amount) * step);
			}

			last_step_axis = select(select(2u, 1u, axis_distances.y <= axis_distances.z), 0u, axis_distances.x <= axis_distances.y && axis_distances.x <= axis_distances.z);
			last_distance = axis_distances[last_step_axis];
			if max_length < last_distance { return no_hit; }
			axis_distances[last_step_axis] += delta[last_step_axis];

			let new_pos_signed = i32(root_relative_grid_pos[last_step_axis]) + step[last_step_axis];
			if new_pos_signed < 0 || u32(new_pos_signed) >= root_size_u32 { return no_hit; }

			// Walk up until the new position shares a node with the current one.
			if root_relative_grid_pos[last_step_axis] / node_size != u32(new_pos_signed) / node_size {
				// current_depth = root_depth; // I want to try this instead of walking up but it does not work????
				// current_node_index = 0;
				var new_node_size = node_size;
				loop {
					if current_depth == root_depth { return no_hit; }
					node_byte_off = dda_node_byte_off(tree_base, current_node_index);
					let walk_up_offset = select(
						dda_nonleaf_parent_offset(node_byte_off),
						dda_leaf_parent_offset(node_byte_off),
						current_depth == 0u,
					);
					current_node_index -= walk_up_offset;
					current_depth      += 1u;
					new_node_size = new_node_size << DDA_LOG_SIZE;
					if root_relative_grid_pos[last_step_axis] / new_node_size == u32(new_pos_signed) / new_node_size { break; }
				}
				node_byte_off = dda_node_byte_off(tree_base, current_node_index);
				bitmap_low 	= dda_bitmap_lo(node_byte_off);
				bitmap_high = dda_bitmap_hi(node_byte_off);
			}
			root_relative_grid_pos[last_step_axis] = u32(new_pos_signed);
		} else {
			let raw_cell        = dda_read_cell(node_byte_off, cell_index, current_depth == 0, bitmap_low, bitmap_high);
			let cell_value_type = dda_cell_type(raw_cell);
			let cell_value_data = dda_cell_value(raw_cell);
			if cell_value_type == 0u {
				// DATA: hit
				var normal_vec = vec3<f32>(0.0);
				normal_vec[last_step_axis] = -f32(step[last_step_axis]);
				var hit_result: DDAHit;
				hit_result.hit    = true;
				hit_result.value  = cell_value_data;
				hit_result.normal = normal_vec;
				hit_result.dist   = last_distance;
				return hit_result;
			} else {
				// NODE: descend
				current_node_index += cell_value_data;
				current_depth      -= 1u;
				node_byte_off 		= dda_node_byte_off(tree_base, current_node_index);
				bitmap_low        	= dda_bitmap_lo(node_byte_off);
				bitmap_high        	= dda_bitmap_hi(node_byte_off);
			}
		}
	}
	return no_hit;
}
