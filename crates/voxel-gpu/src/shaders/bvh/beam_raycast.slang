#pragma once
#include "bvh/data.slang"
#include "helpers/raycast.slang"

static const uint BEAM_BVH_RAYCAST_ITEM_FLAG = 0x80000000u;
static const int BEAM_BVH_RAYCAST_STACK_DEPTH = 32;

struct BeamBVHRaycastStackEntry { uint idx; float dist; };

struct BeamBVHRaycastIter {
	float3 ray_pos;
	float3 inv_dir;
	float expand_k;
	BeamBVHRaycastStackEntry stack[32];
	int top;
};

struct BeamBVHRaycastHit {
	uint bvh_item_idx;
	float dist;
	float aabb_internal_dist;
	bool valid;
};

void beam_bvh_raycast_push(inout BeamBVHRaycastIter iter, uint idx, float dist) {
	if (iter.top >= BEAM_BVH_RAYCAST_STACK_DEPTH) { return; }
	iter.stack[iter.top] = BeamBVHRaycastStackEntry(idx, dist);
	iter.top += 1;
}

BeamBVHRaycastIter beam_bvh_iter_new(float3 ray_pos, float3 ray_dir, float expand_k) {
	BeamBVHRaycastIter iter;
	iter.ray_pos = ray_pos;
	iter.inv_dir = float3(1.0) / ray_dir;
	iter.expand_k = expand_k;
	iter.top = 0;

	let sentinel = bvh[0];
	let distances = ray_aabb(ray_pos, iter.inv_dir, sentinel.c0_min_and_ref.xyz, sentinel.c0_max_and_ref2.xyz);
	let root_ref = asuint(sentinel.c0_min_and_ref.w);
	if ((distances.x < 1e38 && root_ref != 0u)) {
		beam_bvh_raycast_push(iter, root_ref, distances.x);
	}
	return iter;
}

BeamBVHRaycastHit beam_bvh_iter_next(inout BeamBVHRaycastIter iter, float max_dist) {
	BeamBVHRaycastHit miss;
	miss.valid = false;

	let ray_pos = iter.ray_pos;
	let inv_dir = iter.inv_dir;
	let expand_k = iter.expand_k;

	for (;;) {
		if (iter.top <= 0) { break; }
		iter.top -= 1;
		let entry = iter.stack[iter.top];
		if (entry.dist >= max_dist) { continue; }

		if ((entry.idx & BEAM_BVH_RAYCAST_ITEM_FLAG) != 0u) {
			let item_index = entry.idx ^ BEAM_BVH_RAYCAST_ITEM_FLAG;
			let item = bvh_items[item_index];
			let size = unpack4xU8(item.aabb_size);
			let min_corner = float3(item.min_x, item.min_y, item.min_z);
			let max_corner = min_corner + float3(float(size.x), float(size.y), float(size.z)) * item.scale;
			let center = (min_corner + max_corner) * 0.5;
			let depth = length(center - ray_pos);
			let expand = depth * expand_k;
			let distances = ray_aabb(ray_pos, inv_dir, min_corner - float3(expand), max_corner + float3(expand));
			if ((distances.x < max_dist)) {
				return BeamBVHRaycastHit(item_index, distances.x, distances.y - distances.x, true);
			}
			continue;
		}

		let node = bvh[entry.idx];
		let flags = asuint(node.c1_min_and_flags.w);
		let child0_ref = asuint(node.c0_min_and_ref.w);
		let child1_ref = asuint(node.c0_max_and_ref2.w);

		if ((flags & 1u) != 0u) {
			let base = child0_ref;
			let count = child1_ref;
			for (uint i = 0u; i < count; i++) {
				let item_index = base + i;
				let item = bvh_items[item_index];
				let size = unpack4xU8(item.aabb_size);
				let min_corner = float3(item.min_x, item.min_y, item.min_z);
				let max_corner = min_corner + float3(float(size.x), float(size.y), float(size.z)) * item.scale;
				let center = (min_corner + max_corner) * 0.5;
				let depth = length(center - ray_pos);
				let expand = depth * expand_k;
				let distances = ray_aabb(ray_pos, inv_dir, min_corner - float3(expand), max_corner + float3(expand));
				if (distances.x < max_dist) { beam_bvh_raycast_push(iter, BEAM_BVH_RAYCAST_ITEM_FLAG | item_index, distances.x); }
			}
		} else {
			let child0_min = node.c0_min_and_ref.xyz;
			let child0_max = node.c0_max_and_ref2.xyz;
			let child0_center = (child0_min + child0_max) * 0.5;
			let child0_expand = length(child0_center - ray_pos) * expand_k;
			let child0_hit = ray_aabb(ray_pos, inv_dir, child0_min - float3(child0_expand), child0_max + float3(child0_expand));

			let child1_min = node.c1_min_and_flags.xyz;
			let child1_max = node.c1_max_pad.xyz;
			let child1_center = (child1_min + child1_max) * 0.5;
			let child1_expand = length(child1_center - ray_pos) * expand_k;
			let child1_hit = ray_aabb(ray_pos, inv_dir, child1_min - float3(child1_expand), child1_max + float3(child1_expand));

			let child0_hit_ok = child0_hit.x < max_dist;
			let child1_hit_ok = child1_hit.x < max_dist;

			if ((child0_hit_ok && child1_hit_ok)) {
				if ((child0_hit.x < child1_hit.x)) {
					beam_bvh_raycast_push(iter, child1_ref, child1_hit.x);
					beam_bvh_raycast_push(iter, child0_ref, child0_hit.x);
				} else {
					beam_bvh_raycast_push(iter, child0_ref, child0_hit.x);
					beam_bvh_raycast_push(iter, child1_ref, child1_hit.x);
				}
			} else if (child0_hit_ok) { beam_bvh_raycast_push(iter, child0_ref, child0_hit.x); }
			  else if (child1_hit_ok) { beam_bvh_raycast_push(iter, child1_ref, child1_hit.x); }
		}
	}

	return miss;
}

