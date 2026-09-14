#pragma once

#include "bvh/raycast.slang"
#include "direction_feedback.slang"
#include "dda/raycast.slang"
#include "helpers/quat.slang"
#include "face_geometry.slang"

bool occlusion_raycast_impl(float3 ray_pos, float3 ray_dir, float max_dist, bool unknown_blocks) {
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
		if (data_source == 0u) {
			if (unknown_blocks) { return true; }
			continue;
		}
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
	return unknown_blocks && iter.dropped_dist < max_dist;
}

bool occlusion_raycast(float3 ray_pos, float3 ray_dir, float max_dist) {
	return occlusion_raycast_impl(ray_pos, ray_dir, max_dist, false);
}

bool occlusion_raycast_conservative(float3 ray_pos, float3 ray_dir, float max_dist) {
	return occlusion_raycast_impl(ray_pos, ray_dir, max_dist, true);
}

uint face_sun_visible_samples(uint3 face, GiFaceGeometry geometry, uint sample_count) {
	let sun_dir = normalize(float3(0.5, 1.0, 0.2));
	if (dot(geometry.normal, sun_dir) <= 0.0) { return 0u; }
	let width = sample_count == 9u ? 3u : (sample_count == 4u ? 2u : 1u);
	let axis = (face.x >> 3u) & 3u;
	let scale = bvh_items[face.y].scale;
	uint visible = 0u;
	for (uint sample = 0u; sample < sample_count; sample++) {
		// Cell centers in a fixed 1x1, 2x2, or 3x3 grid on the voxel face.
		float3 offset = float3(0.0);
		offset[(axis + 1u) % 3u] = (float(sample % width) + 0.5) / float(width) - 0.5;
		offset[(axis + 2u) % 3u] = (float(sample / width) + 0.5) / float(width) - 0.5;
		let origin = geometry.center + Quat::rotate(geometry.rotation, offset * scale)
			+ geometry.normal * geometry.bias;
		if (!occlusion_raycast_conservative(origin, sun_dir, 1e38)) { visible++; }
	}
	return visible;
}
