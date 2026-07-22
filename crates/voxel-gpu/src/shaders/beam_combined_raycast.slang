#pragma once

#include "bvh/beam_raycast.slang"
#include "bvh/data.slang"
#include "direction_feedback.slang"
#include "dda/beam_raycast.slang"
#include "helpers/quat.slang"

float beam_raycast(float3 ray_pos, float3 ray_dir, float expand_k) {
	var best_total_dist = 1e38;
	BVH::BeamRaycastPolicy bvh_policy;
	bvh_policy.expand_k = expand_k;
	var iter = BVH::iter_new(ray_pos, ray_dir, bvh_policy);

	for (;;) {
		let candidate = BVH::iter_next(iter, best_total_dist);
		if (!candidate.valid) { break; }

		let item = bvh_items[candidate.bvh_item_idx];
		let transform_pos = float3(item.pos_x, item.pos_y, item.pos_z);
		let transform_quat = float4(item.quat_x, item.quat_y, item.quat_z, item.quat_w);

		let entry_world = ray_pos + ray_dir * candidate.dist;
		let local_pos = Quat::inv_rotate(transform_quat, entry_world - transform_pos) / item.scale;
		let local_dir = Quat::inv_rotate(transform_quat, ray_dir);
		record_bvh_item_directions(candidate.bvh_item_idx, local_dir);
		let data_source = item.aabb_size >> 24u;
		if (data_source == 0u) { continue; }
		let remaining = min(best_total_dist - candidate.dist, candidate.aabb_internal_dist);

		float dda_distance;
		if ((data_source == 2u)) {
			dda_distance = beam_dda_raycast<MainDDADataSource>(
				local_pos, local_dir, remaining / item.scale, item.item_index,
				candidate.dist, expand_k, item.scale,
			);
		} else {
			dda_distance = beam_dda_raycast<ResidencyDDADataSource>(
				local_pos, local_dir, remaining / item.scale, item.item_index,
				candidate.dist, expand_k, item.scale,
			);
		}

		if ((dda_distance < 1e37)) {
			let total = candidate.dist + dda_distance * item.scale;
			if ((total < best_total_dist)) {
				best_total_dist = total;
			}
		}
	}

	return best_total_dist;
}
