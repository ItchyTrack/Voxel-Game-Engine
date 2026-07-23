#pragma once

#include "dda/data.slang"

uint dda_initial_hit_normal(float3 start_pos, float3 dir, uint step, uint3 cell_min_u32, uint cell_size) {
	float3 cell_min = float3(cell_min_u32);
	float3 cell_max = cell_min + float3(float(cell_size));
	float3 back_distances = float3(
		dir.x > 0.0 ? (start_pos.x - cell_min.x) / dir.x : 1e38,
		dir.y > 0.0 ? (start_pos.y - cell_min.y) / dir.y : 1e38,
		dir.z > 0.0 ? (start_pos.z - cell_min.z) / dir.z : 1e38
	);
	float3 back_distances_neg = float3(
		dir.x > 0.0 ? back_distances.x : (cell_max.x - start_pos.x) / -dir.x,
		dir.y > 0.0 ? back_distances.y : (cell_max.y - start_pos.y) / -dir.y,
		dir.z > 0.0 ? back_distances.z : (cell_max.z - start_pos.z) / -dir.z
	);
	float3 clamped_back_distances = max(back_distances_neg, float3(0.0));
	uint entry_axis = (clamped_back_distances.x <= clamped_back_distances.y && clamped_back_distances.x <= clamped_back_distances.z)
		? 0u
		: ((clamped_back_distances.y <= clamped_back_distances.z) ? 1u : 2u);
	return step + (entry_axis << 3u) + (1u << 10u);
}

struct DDAHit {
	uint normal;
	uint voxel_data_index;
	uint voxel_index;
	uint sample_octant;
	float dist;
};

uint dda_sample_octant(float3 ray_origin, float3 dir, float hit_dist, uint normal, uint3 root_relative_grid_pos, uint node_size) {
	uint cell_size = node_size >> DDA_LOG_SIZE;
	uint3 cell_min = root_relative_grid_pos - (root_relative_grid_pos & uint3(cell_size - 1u));
	float3 cell_pos = (ray_origin + dir * max(hit_dist, 0.0) - float3(cell_min)) / float(cell_size);
	cell_pos = clamp(cell_pos, float3(0.0), float3(1.0));

	uint axis = (normal >> 3u) & 0xFu;
	if (axis < 3u) {
		cell_pos[axis] = (normal & (1u << axis)) != 0u ? 0.0 : 1.0;
	}

	return (cell_pos.x >= 0.5 ? 1u : 0u)
		+ (cell_pos.y >= 0.5 ? 2u : 0u)
		+ (cell_pos.z >= 0.5 ? 4u : 0u);
}

DDAHit dda_make_hit(float3 ray_origin, float3 start_pos, float3 dir, uint step, uint last_step_axis, float3 axis_distances, float3 delta, uint3 root_relative_grid_pos, uint node_size, uint voxel_data_index, uint voxel_index) {
	DDAHit hit_result;
	float hit_dist = axis_distances[last_step_axis] - delta[last_step_axis];
	if (hit_dist < 0.0) {
		uint cell_size = node_size >> DDA_LOG_SIZE;
		uint3 cell_min = root_relative_grid_pos - (root_relative_grid_pos & uint3(cell_size - 1u));
		hit_result.normal = dda_initial_hit_normal(start_pos, dir, step, cell_min, cell_size);
		hit_result.dist = 0.0;
	} else {
		hit_result.normal = step + (last_step_axis << 3u) + (1u << 10u);
		hit_result.dist = hit_dist;
	}
	hit_result.voxel_data_index = voxel_data_index;
	hit_result.voxel_index = voxel_index;
	hit_result.sample_octant = dda_sample_octant(ray_origin, dir, hit_dist, hit_result.normal, root_relative_grid_pos, node_size);
	return hit_result;
}

DDAHit dda_no_hit() {
	DDAHit no_hit;
	no_hit.normal = 0u;
	no_hit.voxel_data_index = 0u;
	no_hit.voxel_index = 0u;
	no_hit.sample_octant = 0u;
	no_hit.dist = 0.0;
	return no_hit;
}

DDAHit dda_raycast<T : IDDADataSource>(float3 origin, float3 dir, float max_length, uint tree_base) {
	uint tree_header = dda_u32<T>(tree_base);
	uint root_size = dda_node_size(tree_header & 0xFFu);
	float3 root_min = float3(0.0);
	float3 root_max = float3(float(root_size));

	float3 inv_dir = float3(1.0) / dir;
	float3 t1 = (root_min - origin) * inv_dir;
	float3 t2 = (root_max - origin) * inv_dir;
	float tmin = max(max(min(t1.x, t2.x), min(t1.y, t2.y)), min(t1.z, t2.z));
	float tmax = min(min(max(t1.x, t2.x), max(t1.y, t2.y)), max(t1.z, t2.z));
	if ((tmin > tmax || tmax < 0.0)) { return dda_no_hit(); }
	float distance_to_aabb = max(tmin, 0.0);
	if ((distance_to_aabb > max_length)) { return dda_no_hit(); }

	float starting_distance = distance_to_aabb;
	uint3 plane_u32 = uint3((tree_header >> 8u) & 0xFu, (tree_header >> 12u) & 0xFu, (tree_header >> 16u) & 0xFu);
	float max_length_reduced = max_length;
	if (plane_u32.x != 0u || plane_u32.y != 0u || plane_u32.z != 0u) {
		float3 plane = float3(plane_u32) - float3(7.5, 7.5, 7.5);
		float m = float((tree_header >> 20u) & 0xFFu) / 32.0 - 3.984375;
		float root_half_size_f32 = float(root_size) / 2.0;
		float nearest_plane_distance_ish_signed = m * float(root_size) - dot(plane, origin + dir * starting_distance - float3(root_half_size_f32, root_half_size_f32, root_half_size_f32));
		float plane_dist = nearest_plane_distance_ish_signed / dot(plane, dir);
		if (nearest_plane_distance_ish_signed / length(plane) <= 0.0) {
			starting_distance += plane_dist;
			if ((plane_dist < 0.0 || starting_distance > max_length_reduced)) { return dda_no_hit(); }
			float3 hit_point = origin + dir * starting_distance;
			if ((hit_point.x < 0.0 || hit_point.x > float(root_size) + 1.0 || hit_point.y < 0.0 || hit_point.y > float(root_size) + 1.0 || hit_point.z < 0.0 || hit_point.z > float(root_size) + 1.0)) { return dda_no_hit(); }
		} else if (plane_dist >= 0.0) {
			max_length_reduced = starting_distance + plane_dist;
		}
	}

	float3 delta = abs(inv_dir);
	uint step = ((dir.x >= 0.0) ? 1u : 0u) + ((dir.y >= 0.0) ? 1u : 0u) * 2u + ((dir.z >= 0.0) ? 1u : 0u) * 4u;
	float3 post_aabb_origin_pre_shift = origin + dir * starting_distance;
	float3 post_aabb_origin_clamped = clamp(post_aabb_origin_pre_shift, root_min, float3(float(root_size) - 0.00001));
	float3 move_towards_target = floor(post_aabb_origin_clamped) + float3(0.5);
	float3 move_towards_delta = move_towards_target - post_aabb_origin_clamped;
	float move_towards_length = length(move_towards_delta);
	float3 post_aabb_origin = move_towards_length < 0.001 ? post_aabb_origin_clamped : post_aabb_origin_clamped + move_towards_delta / move_towards_length * 0.001;

	float3 root_relative_post_aabb_origin = post_aabb_origin;
	float3 axis_distances_boundary = float3(
		(step & 1u) != 0u ? ceil(root_relative_post_aabb_origin.x) : floor(root_relative_post_aabb_origin.x),
		(step & 2u) != 0u ? ceil(root_relative_post_aabb_origin.y) : floor(root_relative_post_aabb_origin.y),
		(step & 4u) != 0u ? ceil(root_relative_post_aabb_origin.z) : floor(root_relative_post_aabb_origin.z)
	);
	float3 axis_distances = inv_dir * (axis_distances_boundary - root_relative_post_aabb_origin) + float3(starting_distance, starting_distance, starting_distance);
	uint3 root_relative_grid_pos = uint3(int3(root_relative_post_aabb_origin));

	uint last_step_axis;
	float3 pre_shift_delta = abs(post_aabb_origin_pre_shift - post_aabb_origin);
	if ((pre_shift_delta.x >= pre_shift_delta.y && pre_shift_delta.x >= pre_shift_delta.z)) { last_step_axis = 0u; }
	else if ((pre_shift_delta.y >= pre_shift_delta.z)) { last_step_axis = 1u; }
	else { last_step_axis = 2u; }

	if ((max_length_reduced < starting_distance)) { return dda_no_hit(); }

	uint nodes_base = dda_nodes_base(tree_base);
	uint node_size = root_size;
	uint node_byte_off = dda_node_byte_off(nodes_base, 0u);
	uint bitmap_status = dda_parent_offset<T>(node_byte_off) >> 14u;
	uint bitmap_low = 0u;
	uint bitmap_high = 0u;
	if ((bitmap_status & 1u) != 0u) { bitmap_low = dda_u32<T>(node_byte_off + 4u); }
	if ((bitmap_status & 2u) != 0u) { bitmap_high = dda_u32<T>(node_byte_off + 4u + (bitmap_status & 1u) * 4u); }

	for (uint traversal_guard = 0u; traversal_guard < 0xffffffffu; traversal_guard++) {
		uint3 current_relative_pos = root_relative_grid_pos & uint3(node_size - 1u);
		uint3 contents_pos = current_relative_pos / uint3(node_size >> DDA_LOG_SIZE);
		uint cell_index = dda_get_child_contents_index(contents_pos);

		if (!dda_bitmap_has(bitmap_low, bitmap_high, bitmap_status, cell_index)) {
			if (node_size != 4u) {
				uint node_child_size = node_size >> DDA_LOG_SIZE;
				uint3 node_cell_relative_grid_pos = root_relative_grid_pos & uint3(node_child_size - 1u);
				uint3 cell_step_amount = uint3(
					(step & 1u) != 0u ? node_child_size - node_cell_relative_grid_pos.x - 1u : node_cell_relative_grid_pos.x,
					(step & 2u) != 0u ? node_child_size - node_cell_relative_grid_pos.y - 1u : node_cell_relative_grid_pos.y,
					(step & 4u) != 0u ? node_child_size - node_cell_relative_grid_pos.z - 1u : node_cell_relative_grid_pos.z
				);
				float3 axis_distances_at_cell_edge = axis_distances + delta * float3(cell_step_amount);
				float dis_to_edge = min(min(axis_distances_at_cell_edge.x, axis_distances_at_cell_edge.y), axis_distances_at_cell_edge.z);
				uint3 adjusted_step_amount = uint3(abs((float3(dis_to_edge, dis_to_edge, dis_to_edge) - axis_distances + delta) / delta));
				if ((axis_distances_at_cell_edge.x <= axis_distances_at_cell_edge.y && axis_distances_at_cell_edge.x <= axis_distances_at_cell_edge.z)) { adjusted_step_amount.x = cell_step_amount.x; }
				else if ((axis_distances_at_cell_edge.y <= axis_distances_at_cell_edge.z)) { adjusted_step_amount.y = cell_step_amount.y; }
				else { adjusted_step_amount.z = cell_step_amount.z; }
				axis_distances += delta * float3(adjusted_step_amount);
				root_relative_grid_pos = uint3(int3(root_relative_grid_pos) + int3(adjusted_step_amount) * int3(int(step & 1u) * 2 - 1, int(step & 2u) - 1, int(step & 4u) / 2 - 1));
			}

			last_step_axis = (axis_distances.x <= axis_distances.y && axis_distances.x <= axis_distances.z) ? 0u : ((axis_distances.y <= axis_distances.z) ? 1u : 2u);
			if ((max_length_reduced < axis_distances[last_step_axis])) { return dda_no_hit(); }
			axis_distances[last_step_axis] += delta[last_step_axis];

			int new_pos_signed = int(root_relative_grid_pos[last_step_axis]) + (((step & (1u << last_step_axis)) != 0u) ? 1 : -1);
			if ((new_pos_signed < 0 || uint(new_pos_signed) >= root_size)) { return dda_no_hit(); }

			if (root_relative_grid_pos[last_step_axis] / node_size != uint(new_pos_signed) / node_size) {
				for (;;) {
					if ((node_size == root_size)) { return dda_no_hit(); }
					node_byte_off -= (dda_parent_offset<T>(node_byte_off) & 0x3FFFu) * DDA_SLOT_BYTES;
					node_size = node_size << DDA_LOG_SIZE;
					if (root_relative_grid_pos[last_step_axis] / node_size == uint(new_pos_signed) / node_size) {
						bitmap_status = dda_parent_offset<T>(node_byte_off) >> 14u;
						break;
					}
				}
				if ((bitmap_status & 1u) != 0u) { bitmap_low = dda_u32<T>(node_byte_off + 4u); }
				if ((bitmap_status & 2u) != 0u) { bitmap_high = dda_u32<T>(node_byte_off + 4u + (bitmap_status & 1u) * 4u); }
			}
			root_relative_grid_pos[last_step_axis] = uint(new_pos_signed);
		} else {
			uint data_idx = dda_data_index(bitmap_low, bitmap_high, bitmap_status, cell_index);
			if (node_size == 4u) {
				return dda_make_hit(origin, post_aabb_origin_clamped, dir, step, last_step_axis, axis_distances, delta, root_relative_grid_pos, node_size, dda_voxel_data_offset<T>(node_byte_off), data_idx);
			}

			uint entry_val = dda_node_entry<T>(node_byte_off, bitmap_status, data_idx, node_size);
			if (entry_val == 0u) {
				return dda_make_hit(origin, post_aabb_origin_clamped, dir, step, last_step_axis, axis_distances, delta, root_relative_grid_pos, node_size, dda_voxel_data_offset<T>(node_byte_off), data_idx);
			} else {
				node_byte_off += entry_val * DDA_SLOT_BYTES;
				node_size = node_size >> DDA_LOG_SIZE;
				bitmap_status = dda_parent_offset<T>(node_byte_off) >> 14u;
				if ((bitmap_status & 1u) != 0u) { bitmap_low = dda_u32<T>(node_byte_off + 4u); }
				if ((bitmap_status & 2u) != 0u) { bitmap_high = dda_u32<T>(node_byte_off + 4u + (bitmap_status & 1u) * 4u); }
			}
		}
	}
	return dda_no_hit();
}

