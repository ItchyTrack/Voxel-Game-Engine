const DDA_LOG_SIZE:          u32 = 2u;
const DDA_SIZE:              u32 = 4u;
const DDA_HALF_UNIT_BYTES: u32 = 65;
const DDA_NONLEAF_HEADER_BYTES: u32 = 2u;
const DDA_LEAF_HEADER_BYTES: u32 = 1u;
const DDA_MAX_STEPS:         u32 = 512u;
const DDA_ROOT_BYTES:        u32 = 6u;
const DDA_ROOT_DEPTH_BYTES:  u32 = 1u;
const DDA_PALETTE_LEN_BYTES: u32 = 1u;
const DDA_COLOR_BYTES:       u32 = 4u; // rgba u8

@group(2) @binding(0) var<storage, read> grid_tree_buf: array<u32>;

// Raw buffer accessors

fn dda_u16(byte_off: u32) -> u32 {
    return (grid_tree_buf[byte_off / 4u] >> ((byte_off % 4u) * 8u)) & 0xFFFFu;
}
fn dda_u8(byte_off: u32) -> u32 {
    return (grid_tree_buf[byte_off / 4u] >> ((byte_off % 4u) * 8u)) & 0xFFu;
}

// Node accessors

fn dda_node_byte_off(tree_base: u32, half_index: u32) -> u32 {
    return dda_nodes_base(tree_base) + half_index * DDA_HALF_UNIT_BYTES;
}
fn dda_root_depth(tree_base: u32) -> u32 {
    return dda_u8(tree_base + DDA_ROOT_BYTES);
}
fn dda_nonleaf_cell_raw(tree_base: u32, half_index: u32, contents_index: u32) -> u32 {
    return dda_u16(
        dda_node_byte_off(tree_base, half_index)
        + DDA_NONLEAF_HEADER_BYTES
        + contents_index * 2u
    );
}

fn dda_leaf_cell_raw(tree_base: u32, half_index: u32, contents_index: u32) -> u32 {
    let raw = dda_u8(
        dda_node_byte_off(tree_base, half_index)
        + DDA_LEAF_HEADER_BYTES
        + contents_index
    );
    if raw == 0xFFu { return 0xFFFFu; }
    return raw;
}

fn dda_nonleaf_parent_offset(tree_base: u32, half_index: u32) -> u32 {
    return dda_u16(dda_node_byte_off(tree_base, half_index));
}

fn dda_leaf_parent_offset(tree_base: u32, half_index: u32) -> u32 {
    return dda_u8(dda_node_byte_off(tree_base, half_index));
}
fn dda_node_size(depth: u32)  -> u32 { return 1u << (DDA_LOG_SIZE * (depth + 1u)); }
fn dda_child_size(depth: u32) -> u32 { return 1u << (DDA_LOG_SIZE * depth); }
fn dda_cell_type(raw_value: u32) -> u32 {
    if raw_value == 0xFFFFu { return 0u; }
    return 1u + (raw_value >> 15u);
}
fn dda_cell_value(raw_value: u32) -> u32 { return raw_value & 0x7FFFu; }
fn get_child_contents_index(contents_pos: vec3<u32>) -> u32 {
    return contents_pos.x + contents_pos.y * DDA_SIZE + contents_pos.z * DDA_SIZE * DDA_SIZE;
}

fn dda_read_root_pos(tree_base: u32) -> vec3<i32> {
    let word_zero = grid_tree_buf[tree_base / 4u];
    let word_one  = grid_tree_buf[(tree_base + 4u) / 4u];
    var root_x = i32(word_zero & 0xFFFFu);          if (root_x & 0x8000) != 0 { root_x -= 0x10000; }
    var root_y = i32((word_zero >> 16u) & 0xFFFFu); if (root_y & 0x8000) != 0 { root_y -= 0x10000; }
    var root_z = i32(word_one & 0xFFFFu);            if (root_z & 0x8000) != 0 { root_z -= 0x10000; }
    return vec3<i32>(root_x, root_y, root_z);
}
fn dda_palette_len(tree_base: u32) -> u32 {
    return dda_u8(tree_base + DDA_ROOT_BYTES + DDA_ROOT_DEPTH_BYTES);
}
fn dda_nodes_base(tree_base: u32) -> u32 {
    let palette_length = dda_palette_len(tree_base);
    return tree_base
        + DDA_ROOT_BYTES
		+ DDA_ROOT_DEPTH_BYTES
        + DDA_PALETTE_LEN_BYTES
        + palette_length * DDA_COLOR_BYTES;
}
fn dda_palette_color(tree_base: u32, palette_index: u32) -> vec4<f32> {
    let palette_base_offset = tree_base + DDA_ROOT_BYTES + DDA_ROOT_DEPTH_BYTES + DDA_PALETTE_LEN_BYTES;
    let byte_offset = palette_base_offset + palette_index * DDA_COLOR_BYTES;
    let channel_r = f32(dda_u8(byte_offset + 0u)) / 255.0;
    let channel_g = f32(dda_u8(byte_offset + 1u)) / 255.0;
    let channel_b = f32(dda_u8(byte_offset + 2u)) / 255.0;
    let channel_a = f32(dda_u8(byte_offset + 3u)) / 255.0;
    return vec4<f32>(channel_r, channel_g, channel_b, channel_a);
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
    var axis_distances = inv_dir * (axis_distances_boundary - root_relative_post_aabb_origin) + vec3<f32>(distance_to_aabb, distance_to_aabb, distance_to_aabb);

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

    for (var _iteration: u32 = 0u; _iteration < DDA_MAX_STEPS; _iteration++) {
        let node_size     = dda_node_size(current_depth);
        let node_child_sz = dda_child_size(current_depth);

        let current_relative_pos = root_relative_grid_pos & vec3<u32>(node_size - 1u);
        let contents_pos         = current_relative_pos / vec3<u32>(node_child_sz);
        let contents_index       = get_child_contents_index(contents_pos);
		let raw_cell = select(
			dda_nonleaf_cell_raw(tree_base, current_node_index, contents_index),
			dda_leaf_cell_raw   (tree_base, current_node_index, contents_index),
			current_depth == 0u,
		);
        let cell_value_type      = dda_cell_type(raw_cell);
        let cell_value_data      = dda_cell_value(raw_cell);

        if cell_value_type == 2u {
            current_node_index += cell_value_data;
			current_depth -= 1;

        } else if cell_value_type == 1u {
            var normal_vec = vec3<f32>(0.0);
            normal_vec[last_step_axis] = -f32(step[last_step_axis]);
            var hit_result: DDAHit;
            hit_result.hit    = true;
            hit_result.value  = cell_value_data;
            hit_result.normal = normal_vec;
            hit_result.dist   = last_distance;
            return hit_result;

        } else {
            if node_child_sz != 1u {
                let node_cell_relative_grid_pos = current_relative_pos & vec3<u32>(node_child_sz - 1u);
                let cell_step_amount = vec3<u32>(
                    select(node_cell_relative_grid_pos.x, node_child_sz - node_cell_relative_grid_pos.x - 1, step.x > 0),
                    select(node_cell_relative_grid_pos.y, node_child_sz - node_cell_relative_grid_pos.y - 1, step.y > 0),
                    select(node_cell_relative_grid_pos.z, node_child_sz - node_cell_relative_grid_pos.z - 1, step.z > 0),
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
			// Pick axis with smallest axis_distance
			let lt_xy = axis_distances.x <= axis_distances.y;
			let lt_xz = axis_distances.x <= axis_distances.z;
			let lt_yz = axis_distances.y <= axis_distances.z;
			let axis  = select(select(2u, 1u, lt_yz), 0u, lt_xy && lt_xz);

			let ad = axis_distances[axis];
			if max_length < ad { return no_hit; }

			let new_pos_signed = i32(root_relative_grid_pos[axis]) + step[axis];
			if new_pos_signed < 0 || u32(new_pos_signed) >= root_size_u32 { return no_hit; }

			// walk up
			loop {
				let sz = dda_node_size(current_depth);
				if root_relative_grid_pos[axis] / sz == u32(new_pos_signed) / sz { break; }
				let walk_up_parent_offset = select(
					dda_nonleaf_parent_offset(tree_base, current_node_index),
					dda_leaf_parent_offset   (tree_base, current_node_index),
					current_depth == 0u,
				);
				if walk_up_parent_offset == 0u { break; }
				current_node_index -= walk_up_parent_offset;
				current_depth += 1;
			}

			root_relative_grid_pos[axis] = u32(new_pos_signed);
			last_distance = ad;
			axis_distances[axis] += delta[axis];
			last_step_axis = axis;
        }
    }
    return no_hit;
}
