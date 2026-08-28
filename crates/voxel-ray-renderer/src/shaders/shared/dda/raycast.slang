#pragma once

#include "dda/data.slang"

struct DDARootIntersection {
	bool hit;
	float entry_distance;
	float exit_distance;
};

float2 dda_axis_intersection(float origin, float dir, float axis_min, float axis_max) {
	if (dir == 0.0) {
		return origin >= axis_min && origin < axis_max
			? float2(-1e38, 1e38)
			: float2(1e38, -1e38);
	}
	let distance_to_min = (axis_min - origin) / dir;
	let distance_to_max = (axis_max - origin) / dir;
	return float2(min(distance_to_min, distance_to_max), max(distance_to_min, distance_to_max));
}

DDARootIntersection dda_root_intersection(float3 origin, float3 dir, uint root_size) {
	if (dir.x == 0.0 && dir.y == 0.0 && dir.z == 0.0) {
		DDARootIntersection miss = {};
		return miss;
	}

	let root_max = float(root_size);
	let x_distances = dda_axis_intersection(origin.x, dir.x, 0.0, root_max);
	let y_distances = dda_axis_intersection(origin.y, dir.y, 0.0, root_max);
	let z_distances = dda_axis_intersection(origin.z, dir.z, 0.0, root_max);
	let entry_distance = max(max(x_distances.x, y_distances.x), z_distances.x);
	let exit_distance = min(min(x_distances.y, y_distances.y), z_distances.y);

	DDARootIntersection result;
	result.hit = entry_distance <= exit_distance && exit_distance >= 0.0;
	result.entry_distance = max(entry_distance, 0.0);
	result.exit_distance = exit_distance;
	return result;
}

struct DDAInitialization {
	bool valid;
	float3 inv_dir;
	float3 delta;
	float3 axis_distances;
	uint3 grid_pos;
	uint step;
};

DDAInitialization dda_initialize(float3 origin, float3 dir, float starting_distance, uint root_size) {
	DDAInitialization result = {};
	let entry_pos = origin + dir * starting_distance;
	let root_max = float(root_size);
	if (
		(entry_pos.x <= 0.0 && dir.x < 0.0) || (entry_pos.x >= root_max && dir.x > 0.0) ||
		(entry_pos.y <= 0.0 && dir.y < 0.0) || (entry_pos.y >= root_max && dir.y > 0.0) ||
		(entry_pos.z <= 0.0 && dir.z < 0.0) || (entry_pos.z >= root_max && dir.z > 0.0) ||
		(dir.x == 0.0 && (entry_pos.x < 0.0 || entry_pos.x >= root_max)) ||
		(dir.y == 0.0 && (entry_pos.y < 0.0 || entry_pos.y >= root_max)) ||
		(dir.z == 0.0 && (entry_pos.z < 0.0 || entry_pos.z >= root_max))
	) {
		return result;
	}

	var grid_pos = int3(floor(entry_pos));
	if (dir.x < 0.0) { grid_pos.x = int(ceil(entry_pos.x)) - 1; }
	if (dir.y < 0.0) { grid_pos.y = int(ceil(entry_pos.y)) - 1; }
	if (dir.z < 0.0) { grid_pos.z = int(ceil(entry_pos.z)) - 1; }
	grid_pos = clamp(grid_pos, int3(0), int3(int(root_size) - 1));

	result.valid = true;
	result.inv_dir = float3(
		dir.x != 0.0 ? 1.0 / dir.x : 1e38,
		dir.y != 0.0 ? 1.0 / dir.y : 1e38,
		dir.z != 0.0 ? 1.0 / dir.z : 1e38
	);
	result.delta = float3(
		dir.x != 0.0 ? abs(result.inv_dir.x) : 1e38,
		dir.y != 0.0 ? abs(result.inv_dir.y) : 1e38,
		dir.z != 0.0 ? abs(result.inv_dir.z) : 1e38
	);
	result.step = (dir.x > 0.0 ? 1u : 0u) + (dir.y > 0.0 ? 2u : 0u) + (dir.z > 0.0 ? 4u : 0u);
	let next_boundaries = float3(
		dir.x > 0.0 ? float(grid_pos.x + 1) : float(grid_pos.x),
		dir.y > 0.0 ? float(grid_pos.y + 1) : float(grid_pos.y),
		dir.z > 0.0 ? float(grid_pos.z + 1) : float(grid_pos.z)
	);
	result.axis_distances = float3(
		dir.x != 0.0 ? max((next_boundaries.x - origin.x) / dir.x, starting_distance) : 1e38,
		dir.y != 0.0 ? max((next_boundaries.y - origin.y) / dir.y, starting_distance) : 1e38,
		dir.z != 0.0 ? max((next_boundaries.z - origin.z) / dir.z, starting_distance) : 1e38
	);
	result.grid_pos = uint3(grid_pos);
	return result;
}

struct DDAHit {
	uint normal;
	uint voxel_data_index;
	uint voxel_index;
	float dist;
};

static const uint DDA_FACE_OCCLUSION_SHIFT = 11u;
static const uint DDA_VOXEL_COORDINATE_SHIFT = 14u;

uint dda_camera_face_occlusion_bits<T : IDDADataSource>(uint tree_base, uint3 cell_pos, uint normal) {
	uint bits = 0u;
	uint normal_axis = (normal >> 3u) & 0x3u;
	for (uint axis = 0u; axis < 3u; axis++) {
		if (axis == normal_axis) { continue; }
		int3 neighbor = int3(cell_pos);
		neighbor[axis] += (normal & (1u << axis)) != 0u ? -1 : 1;
		if (dda_is_occupied<T>(tree_base, neighbor)) {
			bits |= 1u << (DDA_FACE_OCCLUSION_SHIFT + axis);
		}
	}
	return bits;
}

uint dda_initial_cell_entry_axis(float3 entry_pos, float3 dir, uint3 cell_min_u32, uint cell_size) {
	let cell_min = float3(cell_min_u32);
	let cell_max = cell_min + float3(float(cell_size));
	let back_distances = float3(
		dir.x > 0.0 ? (entry_pos.x - cell_min.x) / dir.x : (dir.x < 0.0 ? (entry_pos.x - cell_max.x) / dir.x : 1e38),
		dir.y > 0.0 ? (entry_pos.y - cell_min.y) / dir.y : (dir.y < 0.0 ? (entry_pos.y - cell_max.y) / dir.y : 1e38),
		dir.z > 0.0 ? (entry_pos.z - cell_min.z) / dir.z : (dir.z < 0.0 ? (entry_pos.z - cell_max.z) / dir.z : 1e38)
	);
	return back_distances.x <= back_distances.y && back_distances.x <= back_distances.z
		? 0u
		: (back_distances.y <= back_distances.z ? 1u : 2u);
}

DDAHit dda_make_hit<T : IDDADataSource>(uint tree_base, uint step, uint last_step_axis, float hit_distance, uint3 root_relative_grid_pos, uint voxel_data_index, uint voxel_index) {
	DDAHit hit_result;
	hit_result.normal = step + (last_step_axis << 3u) + (1u << 10u);
	hit_result.normal |= dda_camera_face_occlusion_bits<T>(tree_base, root_relative_grid_pos, hit_result.normal);
	let packed_coordinate = root_relative_grid_pos.x
		| (root_relative_grid_pos.y << 6u)
		| (root_relative_grid_pos.z << 12u);
	hit_result.normal |= packed_coordinate << DDA_VOXEL_COORDINATE_SHIFT;
	hit_result.dist = hit_distance;
	hit_result.voxel_data_index = voxel_data_index;
	hit_result.voxel_index = voxel_index;
	return hit_result;
}

DDAHit dda_no_hit() {
	DDAHit no_hit;
	no_hit.normal = 0u;
	no_hit.voxel_data_index = 0u;
	no_hit.voxel_index = 0u;
	no_hit.dist = 0.0;
	return no_hit;
}

DDAHit dda_raycast<T : IDDADataSource>(float3 origin, float3 dir, float max_length, uint tree_base) {
	uint tree_header = dda_u32<T>(tree_base);
	uint root_size = dda_node_size(tree_header & 0xFFu);
	let root_intersection = dda_root_intersection(origin, dir, root_size);
	if (!root_intersection.hit || root_intersection.entry_distance > max_length) { return dda_no_hit(); }

	float starting_distance = root_intersection.entry_distance;
	uint3 plane_u32 = uint3((tree_header >> 8u) & 0xFu, (tree_header >> 12u) & 0xFu, (tree_header >> 16u) & 0xFu);
	float max_length_reduced = max_length;
	if (plane_u32.x != 0u || plane_u32.y != 0u || plane_u32.z != 0u) {
		float3 plane = float3(plane_u32) - float3(7.5, 7.5, 7.5);
		float m = float((tree_header >> 20u) & 0xFFu) / 32.0 - 3.984375;
		float root_half_size_f32 = float(root_size) / 2.0;
		float nearest_plane_distance_ish_signed = m * float(root_size) - dot(plane, origin + dir * starting_distance - float3(root_half_size_f32, root_half_size_f32, root_half_size_f32));
		let plane_dir_dot = dot(plane, dir);
		if (plane_dir_dot == 0.0) {
			if (nearest_plane_distance_ish_signed < 0.0) { return dda_no_hit(); }
		} else {
			let plane_dist = nearest_plane_distance_ish_signed / plane_dir_dot;
			if (nearest_plane_distance_ish_signed < 0.0) {
				starting_distance += plane_dist;
				if ((plane_dist < 0.0 || starting_distance > max_length_reduced)) { return dda_no_hit(); }
				float3 hit_point = origin + dir * starting_distance;
				if ((hit_point.x < 0.0 || hit_point.x > float(root_size) + 1.0 || hit_point.y < 0.0 || hit_point.y > float(root_size) + 1.0 || hit_point.z < 0.0 || hit_point.z > float(root_size) + 1.0)) { return dda_no_hit(); }
			} else if (nearest_plane_distance_ish_signed == 0.0 && plane_dir_dot > 0.0) {
				max_length_reduced = starting_distance;
			} else if (nearest_plane_distance_ish_signed > 0.0 && plane_dist >= 0.0) {
				max_length_reduced = min(max_length_reduced, starting_distance + plane_dist);
			}
		}
	}

	let initialization = dda_initialize(origin, dir, starting_distance, root_size);
	if (!initialization.valid || max_length_reduced < starting_distance) { return dda_no_hit(); }
	float3 delta = initialization.delta;
	uint step = initialization.step;
	float3 axis_distances = initialization.axis_distances;
	uint3 root_relative_grid_pos = initialization.grid_pos;
	uint last_step_axis = 0u;
	float current_cell_entry_distance = starting_distance;
	bool initial_cell = true;

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
					dir.x == 0.0 ? 0u : ((step & 1u) != 0u ? node_child_size - node_cell_relative_grid_pos.x - 1u : node_cell_relative_grid_pos.x),
					dir.y == 0.0 ? 0u : ((step & 2u) != 0u ? node_child_size - node_cell_relative_grid_pos.y - 1u : node_cell_relative_grid_pos.y),
					dir.z == 0.0 ? 0u : ((step & 4u) != 0u ? node_child_size - node_cell_relative_grid_pos.z - 1u : node_cell_relative_grid_pos.z)
				);
				float3 axis_distances_at_cell_edge = axis_distances + delta * float3(cell_step_amount);
				float dis_to_edge = min(min(axis_distances_at_cell_edge.x, axis_distances_at_cell_edge.y), axis_distances_at_cell_edge.z);
				uint3 adjusted_step_amount = uint3(max(ceil((float3(dis_to_edge) - axis_distances) / delta), float3(0.0)));
				if ((axis_distances_at_cell_edge.x <= axis_distances_at_cell_edge.y && axis_distances_at_cell_edge.x <= axis_distances_at_cell_edge.z)) { adjusted_step_amount.x = cell_step_amount.x; }
				else if ((axis_distances_at_cell_edge.y <= axis_distances_at_cell_edge.z)) { adjusted_step_amount.y = cell_step_amount.y; }
				else { adjusted_step_amount.z = cell_step_amount.z; }
				axis_distances += delta * float3(adjusted_step_amount);
				root_relative_grid_pos = uint3(int3(root_relative_grid_pos) + int3(adjusted_step_amount) * int3(int(step & 1u) * 2 - 1, int(step & 2u) - 1, int(step & 4u) / 2 - 1));
			}

			last_step_axis = (axis_distances.x <= axis_distances.y && axis_distances.x <= axis_distances.z) ? 0u : ((axis_distances.y <= axis_distances.z) ? 1u : 2u);
			if ((max_length_reduced < axis_distances[last_step_axis])) { return dda_no_hit(); }
			current_cell_entry_distance = axis_distances[last_step_axis];
			axis_distances[last_step_axis] += delta[last_step_axis];
			initial_cell = false;

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
			uint hit_axis = last_step_axis;
			if (initial_cell) {
				uint cell_size = node_size >> DDA_LOG_SIZE;
				uint3 cell_min = root_relative_grid_pos - (root_relative_grid_pos & uint3(cell_size - 1u));
				hit_axis = dda_initial_cell_entry_axis(origin + dir * starting_distance, dir, cell_min, cell_size);
			}
			if (node_size == 4u) {
				return dda_make_hit<T>(tree_base, step, hit_axis, current_cell_entry_distance, root_relative_grid_pos, dda_voxel_data_offset<T>(node_byte_off), data_idx);
			}

			uint entry_val = dda_node_entry<T>(node_byte_off, bitmap_status, data_idx, node_size);
			if (entry_val == 0u) {
				return dda_make_hit<T>(tree_base, step, hit_axis, current_cell_entry_distance, root_relative_grid_pos, dda_voxel_data_offset<T>(node_byte_off), data_idx);
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

