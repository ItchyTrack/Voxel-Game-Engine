import package::dda::data::{
	DDA_SIZE, DDAHit, DDA_LOG_SIZE, DDA_SLOT_BYTES, dda_bitmap_has, dda_data_index,
	dda_get_child_contents_index, dda_node_byte_off, dda_node_size, dda_nodes_base,
};
import package::generated::dda::__DATA_MODULE__::{
	__DDA_NODE_ENTRY__, __DDA_PARENT_OFFSET__, __DDA_U32__, __DDA_VOXEL_DATA_OFFSET__,
};

fn dda_initial_hit_normal(
	start_pos: vec3<f32>,
	dir: vec3<f32>,
	step: u32,
	cell_min_u32: vec3<u32>,
	cell_size: u32,
) -> u32 {
	let cell_min = vec3<f32>(cell_min_u32);
	let cell_max = cell_min + vec3<f32>(f32(cell_size));
	let back_distances = vec3<f32>(
		select(1e38, (start_pos.x - cell_min.x) / dir.x, dir.x > 0.0),
		select(1e38, (start_pos.y - cell_min.y) / dir.y, dir.y > 0.0),
		select(1e38, (start_pos.z - cell_min.z) / dir.z, dir.z > 0.0),
	);
	let back_distances_neg = vec3<f32>(
		select((cell_max.x - start_pos.x) / -dir.x, back_distances.x, dir.x > 0.0),
		select((cell_max.y - start_pos.y) / -dir.y, back_distances.y, dir.y > 0.0),
		select((cell_max.z - start_pos.z) / -dir.z, back_distances.z, dir.z > 0.0),
	);
	let clamped_back_distances = max(back_distances_neg, vec3<f32>(0.0));
	let entry_axis = select(
		select(2u, 1u, clamped_back_distances.y <= clamped_back_distances.z),
		0u,
		clamped_back_distances.x <= clamped_back_distances.y && clamped_back_distances.x <= clamped_back_distances.z,
	);
	return step + (entry_axis << 3u) + (1u << 10u);
}

fn dda_make_hit(
	start_pos: vec3<f32>,
	dir: vec3<f32>,
	step: u32,
	last_step_axis: u32,
	axis_distances: vec3<f32>,
	delta: vec3<f32>,
	root_relative_grid_pos: vec3<u32>,
	node_size: u32,
	voxel_data_index: u32,
	voxel_index: u32,
) -> DDAHit {
	var hit_result: DDAHit;
	let hit_dist = axis_distances[last_step_axis] - delta[last_step_axis];
	if hit_dist < 0.0 {
		let cell_size = node_size >> DDA_LOG_SIZE;
		let cell_min = root_relative_grid_pos - (root_relative_grid_pos & vec3<u32>(cell_size - 1u));
		hit_result.normal = dda_initial_hit_normal(start_pos, dir, step, cell_min, cell_size);
		hit_result.dist = 0.0;
	} else {
		hit_result.normal = step + (last_step_axis << 3u) + (1u << 10u);
		hit_result.dist = hit_dist;
	}
	hit_result.voxel_data_index = voxel_data_index;
	hit_result.voxel_index = voxel_index;
	return hit_result;
}

fn __DDA_RAYCAST__(
	origin:     vec3<f32>,
	dir:        vec3<f32>,
	max_length: f32,
	tree_base:  u32,
) -> DDAHit {
	let tree_header = __DDA_U32__(tree_base);
	let root_size = dda_node_size(tree_header & 0xFFu);
	let root_min = vec3<f32>(0.0);
	let root_max = vec3<f32>(f32(root_size));

	let inv_dir = vec3<f32>(1.0) / dir;
	let t1      = (root_min - origin) * inv_dir;
	let t2      = (root_max - origin) * inv_dir;
	let tmin    = max(max(min(t1.x, t2.x), min(t1.y, t2.y)), min(t1.z, t2.z));
	let tmax    = min(min(max(t1.x, t2.x), max(t1.y, t2.y)), max(t1.z, t2.z));
	if tmin > tmax || tmax < 0.0 {
		var no_hit: DDAHit;
		no_hit.normal = 0u;
		return no_hit;
	}
	let distance_to_aabb = max(tmin, 0.0);
	if distance_to_aabb > max_length {
		var no_hit: DDAHit;
		no_hit.normal = 0u;
		return no_hit;
	}

	var starting_distance = distance_to_aabb;
	let plane_u32 = vec3<u32>(
		(tree_header >> 8) & 0xF,
		(tree_header >> 12) & 0xF,
		(tree_header >> 16) & 0xF
	);
	var max_length_reduced = max_length;
	if (plane_u32.x != 0 || plane_u32.y != 0 || plane_u32.z != 0) {
		let plane = vec3<f32>(plane_u32) - vec3<f32>(7.5, 7.5, 7.5);
		let m = f32((tree_header >> 20) & 0xFF) / 32.0 - 3.984375;

		let root_half_size_f32 = f32(root_size) / 2;
		let nearest_plane_distance_ish_signed = m * f32(root_size) - dot(plane, origin + dir * starting_distance - vec3<f32>(root_half_size_f32, root_half_size_f32, root_half_size_f32));
		let plane_dist = nearest_plane_distance_ish_signed / dot(plane, dir);
		if (nearest_plane_distance_ish_signed / length(plane) <= 0) {
			starting_distance += plane_dist;
			if (plane_dist < 0 || starting_distance > max_length_reduced) {
				var no_hit: DDAHit;
				no_hit.normal = 0u;
				return no_hit;
			}
			let hit_point = origin + dir * starting_distance;
			if (hit_point.x < 0 || hit_point.x > f32(root_size) + 1 || hit_point.y < 0 || hit_point.y > f32(root_size) + 1 || hit_point.z < 0 || hit_point.z > f32(root_size) + 1) {
				var no_hit: DDAHit;
				no_hit.normal = 0u;
				return no_hit;
			}
		} else if (plane_dist >= 0) {
			max_length_reduced = starting_distance + plane_dist;
		}
	}

	let delta = abs(inv_dir);
	let step = u32(dir.x >= 0.0) + u32(dir.y >= 0.0) * 2 + u32(dir.z >= 0.0) * 4;
	let post_aabb_origin_pre_shift = origin + dir * starting_distance;
	let post_aabb_origin_clamped   = clamp(post_aabb_origin_pre_shift, root_min, vec3<f32>(f32(root_size) - 0.00001));
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
		+ vec3<f32>(starting_distance, starting_distance, starting_distance);
	var root_relative_grid_pos = vec3<u32>(vec3<i32>(root_relative_post_aabb_origin));

	var last_step_axis: u32;
	let pre_shift_delta = abs(post_aabb_origin_pre_shift - post_aabb_origin);
	if pre_shift_delta.x >= pre_shift_delta.y && pre_shift_delta.x >= pre_shift_delta.z { last_step_axis = 0u; }
	else if pre_shift_delta.y >= pre_shift_delta.z                                      { last_step_axis = 1u; }
	else                                                                                { last_step_axis = 2u; }

	if max_length_reduced < starting_distance {
		var no_hit: DDAHit;
		no_hit.normal = 0u;
		return no_hit;
	}

	let nodes_base = dda_nodes_base(tree_base);
	var node_size = root_size;
	var node_byte_off = dda_node_byte_off(nodes_base, 0u);
	var bitmap_status = __DDA_PARENT_OFFSET__(node_byte_off) >> 14;
	var bitmap_low = 0u;
	var bitmap_high = 0u;
	if (bitmap_status & 1) != 0 { bitmap_low = __DDA_U32__(node_byte_off + 4); }
	if (bitmap_status & 2) != 0 { bitmap_high = __DDA_U32__(node_byte_off + 4 + (bitmap_status & 1) * 4u); }

	loop {
		let current_relative_pos = root_relative_grid_pos & vec3<u32>(node_size - 1u);
		let contents_pos = current_relative_pos / vec3<u32>(node_size >> DDA_LOG_SIZE);
		let cell_index = dda_get_child_contents_index(contents_pos);

		if !dda_bitmap_has(bitmap_low, bitmap_high, bitmap_status, cell_index) {
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
				if      axis_distances_at_cell_edge.x <= axis_distances_at_cell_edge.y && axis_distances_at_cell_edge.x <= axis_distances_at_cell_edge.z { adjusted_step_amount.x = cell_step_amount.x; }
				else if axis_distances_at_cell_edge.y <= axis_distances_at_cell_edge.z { adjusted_step_amount.y = cell_step_amount.y; }
				else                                                                   { adjusted_step_amount.z = cell_step_amount.z; }
				axis_distances += delta * vec3<f32>(adjusted_step_amount);
				root_relative_grid_pos = vec3<u32>(vec3<i32>(root_relative_grid_pos) + vec3<i32>(adjusted_step_amount) * vec3<i32>(
					i32(step & 1) * 2 - 1, i32(step & 2) - 1, i32(step & 4) / 2 - 1
				));
			}

			last_step_axis = select(select(2u, 1u, axis_distances.y <= axis_distances.z), 0u, axis_distances.x <= axis_distances.y && axis_distances.x <= axis_distances.z);
			if max_length_reduced < axis_distances[last_step_axis] {
				var no_hit: DDAHit;
				no_hit.normal = 0u;
				return no_hit;
			}
			axis_distances[last_step_axis] += delta[last_step_axis];

			let new_pos_signed = i32(root_relative_grid_pos[last_step_axis]) + (i32((step & (1u << last_step_axis)) != 0) * 2 - 1);
			if new_pos_signed < 0 || u32(new_pos_signed) >= root_size {
				var no_hit: DDAHit;
				no_hit.normal = 0u;
				return no_hit;
			}

			if root_relative_grid_pos[last_step_axis] / node_size != u32(new_pos_signed) / node_size {
				loop {
					if node_size == root_size {
						var no_hit: DDAHit;
						no_hit.normal = 0u;
						return no_hit;
					}
					node_byte_off -= (__DDA_PARENT_OFFSET__(node_byte_off) & 0x3FFF) * DDA_SLOT_BYTES;
					node_size = node_size << DDA_LOG_SIZE;
					if root_relative_grid_pos[last_step_axis] / node_size == u32(new_pos_signed) / node_size {
						bitmap_status = __DDA_PARENT_OFFSET__(node_byte_off) >> 14;
						break;
					}
				}
				if (bitmap_status & 1) != 0 { bitmap_low = __DDA_U32__(node_byte_off + 4); }
				if (bitmap_status & 2) != 0 { bitmap_high = __DDA_U32__(node_byte_off + 4 + (bitmap_status & 1) * 4u); }
			}
			root_relative_grid_pos[last_step_axis] = u32(new_pos_signed);
		} else {
			let data_idx = dda_data_index(bitmap_low, bitmap_high, bitmap_status, cell_index);
			if node_size == 4u {
				return dda_make_hit(
					post_aabb_origin_clamped,
					dir,
					step,
					last_step_axis,
					axis_distances,
					delta,
					root_relative_grid_pos,
					node_size,
					__DDA_VOXEL_DATA_OFFSET__(node_byte_off),
					data_idx,
				);
			}

			let entry_val = __DDA_NODE_ENTRY__(node_byte_off, bitmap_status, data_idx, node_size);
			if entry_val == 0u {
				return dda_make_hit(
					post_aabb_origin_clamped,
					dir,
					step,
					last_step_axis,
					axis_distances,
					delta,
					root_relative_grid_pos,
					node_size,
					__DDA_VOXEL_DATA_OFFSET__(node_byte_off),
					data_idx,
				);
			} else {
				node_byte_off += entry_val * DDA_SLOT_BYTES;
				node_size = node_size >> DDA_LOG_SIZE;
				bitmap_status = __DDA_PARENT_OFFSET__(node_byte_off) >> 14;
				if (bitmap_status & 1) != 0 { bitmap_low = __DDA_U32__(node_byte_off + 4); }
				if (bitmap_status & 2) != 0 { bitmap_high = __DDA_U32__(node_byte_off + 4 + (bitmap_status & 1) * 4u); }
			}
		}
	}
	var no_hit: DDAHit;
	no_hit.normal = 0u;
	return no_hit;
}
