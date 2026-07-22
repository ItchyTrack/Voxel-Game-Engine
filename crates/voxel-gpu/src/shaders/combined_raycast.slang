#pragma once
#include "bvh/data.slang"
#include "bvh/raycast.slang"
#include "direction_feedback.slang"
#include "dda/data.slang"
#include "dda/raycast.slang"


float3 quat_inv_rotate(float4 q, float3 v) {
	let t = 2.0 * cross(q.xyz, v);
	return v - q.w * t + cross(q.xyz, t);
}

struct RaycastHit {
	uint normal;
	uint bvh_item_idx;
	uint voxel_data_index;
	uint voxel_index;
	float total_dist;
};

RaycastHit full_raycast(float3 ray_pos, float3 ray_dir, float max_dist) {
	RaycastHit best;
	best.normal = 0;
	best.total_dist = max_dist;

	var iter = bvh_iter_new(ray_pos, ray_dir);

	for (;;) {
		let candidate = bvh_iter_next(iter, best.total_dist);
		if (!candidate.valid) { break; }

		let item = bvh_items[candidate.bvh_item_idx];
		let transform_pos = float3(item.pos_x,  item.pos_y,  item.pos_z);
		let transform_quat = float4(item.quat_x, item.quat_y, item.quat_z, item.quat_w);

		let entry_world = ray_pos + ray_dir * candidate.dist;
		let local_pos = quat_inv_rotate(transform_quat, entry_world - transform_pos) / item.scale;
		let local_dir = quat_inv_rotate(transform_quat, ray_dir);
		record_bvh_item_directions(candidate.bvh_item_idx, local_dir);
		let data_source = item.aabb_size >> 24u;
		if (data_source == 0u) { continue; }
		let remaining = min(best.total_dist - candidate.dist, candidate.aabb_internal_dist);
		DDAHit dda_hit;
		if ((data_source == 2u)) {
			dda_hit = dda_raycast<MainDDADataSource>(local_pos, local_dir, remaining / item.scale, item.item_index);
		} else {
			dda_hit = dda_raycast<ResidencyDDADataSource>(local_pos, local_dir, remaining / item.scale, item.item_index);
		}

		if ((dda_hit.normal != 0)) {
			let total = candidate.dist + dda_hit.dist * item.scale;
			if ((total < best.total_dist)) {
				best.normal = dda_hit.normal;
				best.bvh_item_idx = candidate.bvh_item_idx;
				best.voxel_data_index = dda_hit.voxel_data_index;
				best.voxel_index = dda_hit.voxel_index;
				best.total_dist = total;
			}
		}
	}

	return best;
}
