#pragma once

#include "dda/raycast.slang"

float beam_conservative_cell_hit_distance(float3 origin, float3 inv_dir, float beam_radius_local, float hit_distance, uint3 root_relative_grid_pos, uint3 current_relative_pos, uint child_cell_size) {
	uint3 child_cell_min_u32 = root_relative_grid_pos - (current_relative_pos & uint3(child_cell_size - 1u));
	float3 child_cell_min = float3(child_cell_min_u32);
	float3 child_cell_max = child_cell_min + float3(float(child_cell_size));
	float3 expanded_near_distances = min((child_cell_min - origin) * inv_dir, (child_cell_max - origin) * inv_dir) - float3(beam_radius_local) * abs(inv_dir);
	float conservative_entry_distance = max(max(expanded_near_distances.x, expanded_near_distances.y), expanded_near_distances.z);
	return max(0.0, min(hit_distance, conservative_entry_distance));
}

float beam_dda_raycast<T : IDDADataSource>(float3 origin, float3 dir, float max_length, uint tree_base, float beam_start_distance, float beam_expand_k, float item_scale) {
	uint tree_header = dda_u32<T>(tree_base);
	uint root_size = dda_node_size(tree_header & 0xFFu);
	let root_intersection = dda_root_intersection(origin, dir, root_size);
	if (!root_intersection.hit || root_intersection.entry_distance > max_length) { return 1e38; }

	float starting_distance = root_intersection.entry_distance;
	uint3 plane_bits = uint3((tree_header >> 8u) & 0xFu, (tree_header >> 12u) & 0xFu, (tree_header >> 16u) & 0xFu);
	float max_length_reduced = max_length;
	if (plane_bits.x != 0u || plane_bits.y != 0u || plane_bits.z != 0u) {
		float3 plane = float3(plane_bits) - float3(7.5, 7.5, 7.5);
		float plane_scale = float((tree_header >> 20u) & 0xFFu) / 32.0 - 3.984375;
		float root_half_size = float(root_size) / 2.0;
		float nearest_plane_distance_signed = plane_scale * float(root_size) - dot(plane, origin + dir * starting_distance - float3(root_half_size));
		let plane_dir_dot = dot(plane, dir);
		if (plane_dir_dot == 0.0) {
			if (nearest_plane_distance_signed < 0.0) { return 1e38; }
		} else {
			let plane_distance = nearest_plane_distance_signed / plane_dir_dot;
			if (nearest_plane_distance_signed < 0.0) {
				starting_distance += plane_distance;
				if ((plane_distance < 0.0 || starting_distance > max_length_reduced)) { return 1e38; }
				float3 hit_point = origin + dir * starting_distance;
				if ((hit_point.x < 0.0 || hit_point.x > float(root_size) + 1.0 || hit_point.y < 0.0 || hit_point.y > float(root_size) + 1.0 || hit_point.z < 0.0 || hit_point.z > float(root_size) + 1.0)) { return 1e38; }
			} else if (nearest_plane_distance_signed == 0.0 && plane_dir_dot > 0.0) {
				max_length_reduced = starting_distance;
			} else if (nearest_plane_distance_signed > 0.0 && plane_distance >= 0.0) {
				max_length_reduced = min(max_length_reduced, starting_distance + plane_distance);
			}
		}
	}

	let initialization = dda_initialize(origin, dir, starting_distance, root_size);
	if (!initialization.valid || max_length_reduced < starting_distance) { return 1e38; }
	float3 inv_dir = initialization.inv_dir;
	float3 axis_step_distances = initialization.delta;
	uint step_mask = initialization.step;
	float3 axis_hit_distances = initialization.axis_distances;
	uint3 root_relative_grid_pos = initialization.grid_pos;
	float current_cell_entry_distance = starting_distance;
	float safe_item_scale = max(item_scale, 1e-6);
	float initial_beam_radius_local = beam_start_distance * beam_expand_k / safe_item_scale;
	float beam_radius_growth_per_local_distance = item_scale * beam_expand_k / safe_item_scale;

	uint nodes_base = dda_nodes_base(tree_base);
	uint node_size = root_size;
	uint node_byte_offset = dda_node_byte_off(nodes_base, 0u);
	uint bitmap_status = dda_parent_offset<T>(node_byte_offset) >> 14u;
	uint bitmap_low_bits = 0u;
	uint bitmap_high_bits = 0u;
	if ((bitmap_status & 1u) != 0u) { bitmap_low_bits = dda_u32<T>(node_byte_offset + 4u); }
	if ((bitmap_status & 2u) != 0u) { bitmap_high_bits = dda_u32<T>(node_byte_offset + 4u + (bitmap_status & 1u) * 4u); }

	for (uint traversal_guard = 0u; traversal_guard < 0xffffffffu; traversal_guard++) {
		uint3 current_relative_pos = root_relative_grid_pos & uint3(node_size - 1u);
		uint child_cell_size = node_size >> DDA_LOG_SIZE;
		float hit_distance = current_cell_entry_distance;
		float beam_radius_local = initial_beam_radius_local + hit_distance * beam_radius_growth_per_local_distance;
		float beam_diameter_local = beam_radius_local * 2.0;
		if (beam_diameter_local >= float(child_cell_size)) {
			return beam_conservative_cell_hit_distance(origin, inv_dir, beam_radius_local, hit_distance, root_relative_grid_pos, current_relative_pos, child_cell_size);
		}

		uint3 contents_pos = current_relative_pos / uint3(child_cell_size);
		uint cell_index = dda_get_child_contents_index(contents_pos);

		if (!dda_bitmap_has(bitmap_low_bits, bitmap_high_bits, bitmap_status, cell_index)) {
			if (node_size != 4u) {
				uint child_node_size = node_size >> DDA_LOG_SIZE;
				uint3 cell_relative_grid_pos = root_relative_grid_pos & uint3(child_node_size - 1u);
				uint3 cell_step_counts = uint3(
					dir.x == 0.0 ? 0u : ((step_mask & 1u) != 0u ? child_node_size - cell_relative_grid_pos.x - 1u : cell_relative_grid_pos.x),
					dir.y == 0.0 ? 0u : ((step_mask & 2u) != 0u ? child_node_size - cell_relative_grid_pos.y - 1u : cell_relative_grid_pos.y),
					dir.z == 0.0 ? 0u : ((step_mask & 4u) != 0u ? child_node_size - cell_relative_grid_pos.z - 1u : cell_relative_grid_pos.z)
				);
				float3 axis_distances_at_cell_edge = axis_hit_distances + axis_step_distances * float3(cell_step_counts);
				float distance_to_edge = min(min(axis_distances_at_cell_edge.x, axis_distances_at_cell_edge.y), axis_distances_at_cell_edge.z);
				uint3 adjusted_step_counts = uint3(max(ceil((float3(distance_to_edge) - axis_hit_distances) / axis_step_distances), float3(0.0)));
				if ((axis_distances_at_cell_edge.x <= axis_distances_at_cell_edge.y && axis_distances_at_cell_edge.x <= axis_distances_at_cell_edge.z)) { adjusted_step_counts.x = cell_step_counts.x; }
				else if ((axis_distances_at_cell_edge.y <= axis_distances_at_cell_edge.z)) { adjusted_step_counts.y = cell_step_counts.y; }
				else { adjusted_step_counts.z = cell_step_counts.z; }
				axis_hit_distances += axis_step_distances * float3(adjusted_step_counts);
				root_relative_grid_pos = uint3(int3(root_relative_grid_pos) + int3(adjusted_step_counts) * int3(int(step_mask & 1u) * 2 - 1, int(step_mask & 2u) - 1, int(step_mask & 4u) / 2 - 1));
			}

			uint last_step_axis = (axis_hit_distances.x <= axis_hit_distances.y && axis_hit_distances.x <= axis_hit_distances.z) ? 0u : ((axis_hit_distances.y <= axis_hit_distances.z) ? 1u : 2u);
			if ((max_length_reduced < axis_hit_distances[last_step_axis])) { return 1e38; }
			current_cell_entry_distance = axis_hit_distances[last_step_axis];
			axis_hit_distances[last_step_axis] += axis_step_distances[last_step_axis];

			int next_pos_signed = int(root_relative_grid_pos[last_step_axis]) + (((step_mask & (1u << last_step_axis)) != 0u) ? 1 : -1);
			if ((next_pos_signed < 0 || uint(next_pos_signed) >= root_size)) { return 1e38; }

			if (root_relative_grid_pos[last_step_axis] / node_size != uint(next_pos_signed) / node_size) {
				for (;;) {
					if ((node_size == root_size)) { return 1e38; }
					node_byte_offset -= (dda_parent_offset<T>(node_byte_offset) & 0x3FFFu) * DDA_SLOT_BYTES;
					node_size = node_size << DDA_LOG_SIZE;
					if (root_relative_grid_pos[last_step_axis] / node_size == uint(next_pos_signed) / node_size) {
						bitmap_status = dda_parent_offset<T>(node_byte_offset) >> 14u;
						break;
					}
				}
				if ((bitmap_status & 1u) != 0u) { bitmap_low_bits = dda_u32<T>(node_byte_offset + 4u); }
				if ((bitmap_status & 2u) != 0u) { bitmap_high_bits = dda_u32<T>(node_byte_offset + 4u + (bitmap_status & 1u) * 4u); }
			}
			root_relative_grid_pos[last_step_axis] = uint(next_pos_signed);
		} else {
			uint data_index = dda_data_index(bitmap_low_bits, bitmap_high_bits, bitmap_status, cell_index);
			float conservative_hit_distance = beam_conservative_cell_hit_distance(origin, inv_dir, beam_radius_local, hit_distance, root_relative_grid_pos, current_relative_pos, child_cell_size);
			if ((node_size == 4u)) { return conservative_hit_distance; }

			uint entry_value = dda_node_entry<T>(node_byte_offset, bitmap_status, data_index, node_size);
			if (entry_value == 0u) {
				return conservative_hit_distance;
			} else {
				node_byte_offset += entry_value * DDA_SLOT_BYTES;
				node_size = node_size >> DDA_LOG_SIZE;
				bitmap_status = dda_parent_offset<T>(node_byte_offset) >> 14u;
				if ((bitmap_status & 1u) != 0u) { bitmap_low_bits = dda_u32<T>(node_byte_offset + 4u); }
				if ((bitmap_status & 2u) != 0u) { bitmap_high_bits = dda_u32<T>(node_byte_offset + 4u + (bitmap_status & 1u) * 4u); }
			}
		}
	}

	return 1e38;
}

