#pragma once

#include "bvh/raycast.slang"
#include "direction_feedback.slang"
#include "dda/raycast.slang"
#include "helpers/quat.slang"

bool occlusion_raycast(float3 ray_pos, float3 ray_dir, float max_dist) {
	BVH::ExactRaycastPolicy bvh_policy;
	var iter = BVH::iter_new(ray_pos, ray_dir, bvh_policy);

	for (;;) {
		let candidate = BVH::iter_next(iter, max_dist);
		if (!candidate.valid) { break; }

		let item = bvh_items[candidate.bvh_item_idx];
		let transform_pos = float3(item.pos_x,  item.pos_y,  item.pos_z);
		let transform_quat = float4(item.quat_x, item.quat_y, item.quat_z, item.quat_w);

		let entry_world = ray_pos + ray_dir * candidate.dist;
		let local_pos = Quat::inv_rotate(transform_quat, entry_world - transform_pos) / item.scale;
		let local_dir = Quat::inv_rotate(transform_quat, ray_dir);
		record_bvh_item_directions(candidate.bvh_item_idx, local_dir);
		let data_source = item.aabb_size >> 24u;
		if (data_source == 0u) { continue; }
		let remaining = min(max_dist - candidate.dist, candidate.aabb_internal_dist);
		DDAHit dda_hit;
		if ((data_source == 2u)) {
			dda_hit = dda_raycast<MainDDADataSource>(local_pos, local_dir, remaining / item.scale, item.item_index);
		} else {
			dda_hit = dda_raycast<ResidencyDDADataSource>(local_pos, local_dir, remaining / item.scale, item.item_index);
		}
		if ((dda_hit.normal != 0)) {
			let total = candidate.dist + dda_hit.dist * item.scale;
			if ((total < max_dist)) {
				return true;
			}
		}
	}
	return false;
}
