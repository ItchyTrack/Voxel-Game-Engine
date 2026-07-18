import package::dda::data::{
	DDA_LOG_SIZE, DDA_SLOT_BYTES, dda_bitmap_has, dda_data_index,
	dda_get_child_contents_index, dda_node_byte_off, dda_node_size, dda_nodes_base,
};
import package::generated::dda::__DATA_MODULE__::{
	__DDA_NODE_ENTRY__, __DDA_PARENT_OFFSET__, __DDA_U32__,
};
fn beam_conservative_cell_hit_distance(
	origin: vec3<f32>,
	inv_dir: vec3<f32>,
	beam_radius_local: f32,
	hit_distance: f32,
	root_relative_grid_pos: vec3<u32>,
	current_relative_pos: vec3<u32>,
	child_cell_size: u32,
) -> f32 {
	let child_cell_min_u32 = root_relative_grid_pos - (current_relative_pos & vec3<u32>(child_cell_size - 1u));
	let child_cell_min = vec3<f32>(child_cell_min_u32);
	let child_cell_max = child_cell_min + vec3<f32>(f32(child_cell_size));
	let expanded_near_distances = min(
		(child_cell_min - origin) * inv_dir,
		(child_cell_max - origin) * inv_dir,
	) - vec3<f32>(beam_radius_local) * abs(inv_dir);
	let conservative_entry_distance = max(max(expanded_near_distances.x, expanded_near_distances.y), expanded_near_distances.z);
	return max(0.0, min(hit_distance, conservative_entry_distance));
}

fn __BEAM_DDA_RAYCAST__(
	origin:     vec3<f32>,
	dir:        vec3<f32>,
	max_length: f32,
	tree_base:  u32,
	beam_start_distance: f32,
	beam_expand_k: f32,
	item_scale: f32,
) -> f32 {
	let tree_header = __DDA_U32__(tree_base);
	let root_size = dda_node_size(tree_header & 0xFFu);
	let root_min = vec3<f32>(0.0);
	let root_max = vec3<f32>(f32(root_size));

	let inv_dir = vec3<f32>(1.0) / dir;
	let distance_to_min_planes = (root_min - origin) * inv_dir;
	let distance_to_max_planes = (root_max - origin) * inv_dir;
	let near_distances = min(distance_to_min_planes, distance_to_max_planes);
	let entry_distance = max(max(near_distances.x, near_distances.y), near_distances.z);
	let exit_distance = min(min(max(distance_to_min_planes.x, distance_to_max_planes.x), max(distance_to_min_planes.y, distance_to_max_planes.y)), max(distance_to_min_planes.z, distance_to_max_planes.z));
	if entry_distance > exit_distance || exit_distance < 0.0 { return 1e38; }
	let distance_to_aabb = max(entry_distance, 0.0);
	if distance_to_aabb > max_length { return 1e38; }

	var starting_distance = distance_to_aabb;
	let plane_bits = vec3<u32>(
		(tree_header >> 8) & 0xFu,
		(tree_header >> 12) & 0xFu,
		(tree_header >> 16) & 0xFu
	);
	var max_length_reduced = max_length;
	if (plane_bits.x != 0u || plane_bits.y != 0u || plane_bits.z != 0u) {
		let plane = vec3<f32>(plane_bits) - vec3<f32>(7.5, 7.5, 7.5);
		let plane_scale = f32((tree_header >> 20) & 0xFFu) / 32.0 - 3.984375;
		let root_half_size = f32(root_size) / 2.0;
		let nearest_plane_distance_signed = plane_scale * f32(root_size) - dot(plane, origin + dir * starting_distance - vec3<f32>(root_half_size));
		let plane_distance = nearest_plane_distance_signed / dot(plane, dir);
		if (nearest_plane_distance_signed / length(plane) <= 0.0) {
			starting_distance += plane_distance;
			if (plane_distance < 0.0 || starting_distance > max_length_reduced) { return 1e38; }
			let hit_point = origin + dir * starting_distance;
			if (hit_point.x < 0.0 || hit_point.x > f32(root_size) + 1.0 || hit_point.y < 0.0 || hit_point.y > f32(root_size) + 1.0 || hit_point.z < 0.0 || hit_point.z > f32(root_size) + 1.0) {
				return 1e38;
			}
		} else if (plane_distance >= 0.0) {
			max_length_reduced = starting_distance + plane_distance;
		}
	}

	let axis_step_distances = abs(inv_dir);
	let step_mask = u32(dir.x >= 0.0) + u32(dir.y >= 0.0) * 2u + u32(dir.z >= 0.0) * 4u;
	let post_aabb_origin_pre_shift = origin + dir * starting_distance;
	let post_aabb_origin_clamped = clamp(post_aabb_origin_pre_shift, root_min, vec3<f32>(f32(root_size) - 0.00001));
	let move_towards_target = floor(post_aabb_origin_clamped) + vec3<f32>(0.5);
	let move_towards_delta = move_towards_target - post_aabb_origin_clamped;
	let move_towards_length = length(move_towards_delta);
	let post_aabb_origin = select(
		post_aabb_origin_clamped + move_towards_delta / move_towards_length * 0.001,
		post_aabb_origin_clamped,
		move_towards_length < 0.001,
	);

	let root_relative_post_aabb_origin = post_aabb_origin;
	let axis_boundary_distances = vec3<f32>(
		select(floor(root_relative_post_aabb_origin.x), ceil(root_relative_post_aabb_origin.x), (step_mask & 1u) != 0u),
		select(floor(root_relative_post_aabb_origin.y), ceil(root_relative_post_aabb_origin.y), (step_mask & 2u) != 0u),
		select(floor(root_relative_post_aabb_origin.z), ceil(root_relative_post_aabb_origin.z), (step_mask & 4u) != 0u),
	);
	var axis_hit_distances = inv_dir * (axis_boundary_distances - root_relative_post_aabb_origin)
		+ vec3<f32>(starting_distance, starting_distance, starting_distance);
	var root_relative_grid_pos = vec3<u32>(vec3<i32>(root_relative_post_aabb_origin));

	var current_cell_entry_distance = starting_distance;
	let safe_item_scale = max(item_scale, 1e-6);
	let initial_beam_radius_local = beam_start_distance * beam_expand_k / safe_item_scale;
	let beam_radius_growth_per_local_distance = item_scale * beam_expand_k / safe_item_scale;

	if max_length_reduced < starting_distance { return 1e38; }

	let nodes_base = dda_nodes_base(tree_base);
	var node_size = root_size;
	var node_byte_offset = dda_node_byte_off(nodes_base, 0u);
	var bitmap_status = __DDA_PARENT_OFFSET__(node_byte_offset) >> 14;
	var bitmap_low_bits = 0u;
	var bitmap_high_bits = 0u;
	if (bitmap_status & 1u) != 0u { bitmap_low_bits = __DDA_U32__(node_byte_offset + 4u); }
	if (bitmap_status & 2u) != 0u { bitmap_high_bits = __DDA_U32__(node_byte_offset + 4u + (bitmap_status & 1u) * 4u); }

	loop {
		let current_relative_pos = root_relative_grid_pos & vec3<u32>(node_size - 1u);
		let child_cell_size = node_size >> DDA_LOG_SIZE;
		let hit_distance = current_cell_entry_distance;
		let beam_radius_local = initial_beam_radius_local + hit_distance * beam_radius_growth_per_local_distance;
		let beam_diameter_local = beam_radius_local * 2.0;
		if beam_diameter_local >= f32(child_cell_size) {
			return beam_conservative_cell_hit_distance(
				origin,
				inv_dir,
				beam_radius_local,
				hit_distance,
				root_relative_grid_pos,
				current_relative_pos,
				child_cell_size,
			);
		}

		let contents_pos = current_relative_pos / vec3<u32>(child_cell_size);
		let cell_index = dda_get_child_contents_index(contents_pos);

		if !dda_bitmap_has(bitmap_low_bits, bitmap_high_bits, bitmap_status, cell_index) {
			if node_size != 4u {
				let child_node_size = node_size >> DDA_LOG_SIZE;
				let cell_relative_grid_pos = root_relative_grid_pos & vec3<u32>(child_node_size - 1u);
				let cell_step_counts = vec3<u32>(
					select(cell_relative_grid_pos.x, child_node_size - cell_relative_grid_pos.x - 1u, (step_mask & 1u) != 0u),
					select(cell_relative_grid_pos.y, child_node_size - cell_relative_grid_pos.y - 1u, (step_mask & 2u) != 0u),
					select(cell_relative_grid_pos.z, child_node_size - cell_relative_grid_pos.z - 1u, (step_mask & 4u) != 0u),
				);
				let axis_distances_at_cell_edge = axis_hit_distances + axis_step_distances * vec3<f32>(cell_step_counts);
				var distance_to_edge = min(min(axis_distances_at_cell_edge.x, axis_distances_at_cell_edge.y), axis_distances_at_cell_edge.z);
				var adjusted_step_counts = vec3<u32>(abs((vec3<f32>(distance_to_edge) - axis_hit_distances + axis_step_distances) / axis_step_distances));
				if      axis_distances_at_cell_edge.x <= axis_distances_at_cell_edge.y && axis_distances_at_cell_edge.x <= axis_distances_at_cell_edge.z { adjusted_step_counts.x = cell_step_counts.x; }
				else if axis_distances_at_cell_edge.y <= axis_distances_at_cell_edge.z { adjusted_step_counts.y = cell_step_counts.y; }
				else                                                                   { adjusted_step_counts.z = cell_step_counts.z; }
				axis_hit_distances += axis_step_distances * vec3<f32>(adjusted_step_counts);
				root_relative_grid_pos = vec3<u32>(vec3<i32>(root_relative_grid_pos) + vec3<i32>(adjusted_step_counts) * vec3<i32>(
					i32(step_mask & 1u) * 2 - 1, i32(step_mask & 2u) - 1, i32(step_mask & 4u) / 2 - 1
				));
			}

			var last_step_axis = select(select(2u, 1u, axis_hit_distances.y <= axis_hit_distances.z), 0u, axis_hit_distances.x <= axis_hit_distances.y && axis_hit_distances.x <= axis_hit_distances.z);
			if max_length_reduced < axis_hit_distances[last_step_axis] { return 1e38; }
			current_cell_entry_distance = axis_hit_distances[last_step_axis];
			axis_hit_distances[last_step_axis] += axis_step_distances[last_step_axis];

			let next_pos_signed = i32(root_relative_grid_pos[last_step_axis]) + (i32((step_mask & (1u << last_step_axis)) != 0u) * 2 - 1);
			if next_pos_signed < 0 || u32(next_pos_signed) >= root_size { return 1e38; }

			if root_relative_grid_pos[last_step_axis] / node_size != u32(next_pos_signed) / node_size {
				loop {
					if node_size == root_size { return 1e38; }
					node_byte_offset -= (__DDA_PARENT_OFFSET__(node_byte_offset) & 0x3FFFu) * DDA_SLOT_BYTES;
					node_size = node_size << DDA_LOG_SIZE;
					if root_relative_grid_pos[last_step_axis] / node_size == u32(next_pos_signed) / node_size {
						bitmap_status = __DDA_PARENT_OFFSET__(node_byte_offset) >> 14;
						break;
					}
				}
				if (bitmap_status & 1u) != 0u { bitmap_low_bits = __DDA_U32__(node_byte_offset + 4u); }
				if (bitmap_status & 2u) != 0u { bitmap_high_bits = __DDA_U32__(node_byte_offset + 4u + (bitmap_status & 1u) * 4u); }
			}
			root_relative_grid_pos[last_step_axis] = u32(next_pos_signed);
		} else {
			let data_index = dda_data_index(bitmap_low_bits, bitmap_high_bits, bitmap_status, cell_index);
			let conservative_hit_distance = beam_conservative_cell_hit_distance(
				origin,
				inv_dir,
				beam_radius_local,
				hit_distance,
				root_relative_grid_pos,
				current_relative_pos,
				child_cell_size,
			);
			if node_size == 4u { return conservative_hit_distance; }

			let entry_value = __DDA_NODE_ENTRY__(node_byte_offset, bitmap_status, data_index, node_size);
			if entry_value == 0u {
				return conservative_hit_distance;
			} else {
				node_byte_offset += entry_value * DDA_SLOT_BYTES;
				node_size = node_size >> DDA_LOG_SIZE;
				bitmap_status = __DDA_PARENT_OFFSET__(node_byte_offset) >> 14;
				if (bitmap_status & 1u) != 0u { bitmap_low_bits = __DDA_U32__(node_byte_offset + 4u); }
				if (bitmap_status & 2u) != 0u { bitmap_high_bits = __DDA_U32__(node_byte_offset + 4u + (bitmap_status & 1u) * 4u); }
			}
		}
	}

	return 1e38;
}
