#pragma once
#include "bvh/data.slang"
#include "helpers/raycast.slang"

static const uint BVH_RAYCAST_ITEM_FLAG = 0x80000000u;
static const int BVH_RAYCAST_STACK_DEPTH = 32;

struct BVHRaycastStackEntry { uint idx; float dist; };

struct BVHRaycastIter {
	float3 ray_pos;
	float3 inv_dir;
	BVHRaycastStackEntry stack[32];
	int top;
};

struct BVHRaycastHit {
	uint bvh_item_idx;
	float dist;
	float aabb_internal_dist;
	bool valid;
};

void bvh_raycast_push(inout BVHRaycastIter it, uint idx, float dist) {
	if (it.top >= BVH_RAYCAST_STACK_DEPTH) { return; }
	it.stack[it.top] = BVHRaycastStackEntry(idx, dist);
	it.top += 1;
}

BVHRaycastIter bvh_iter_new(float3 ray_pos, float3 ray_dir) {
	BVHRaycastIter iter;
	iter.ray_pos = ray_pos;
	iter.inv_dir = float3(1.0) / ray_dir;
	iter.top     = 0;

	let sentinel = bvh[0];
	let distances = ray_aabb(ray_pos, iter.inv_dir, sentinel.c0_min_and_ref.xyz, sentinel.c0_max_and_ref2.xyz);
	let root_ref = asuint(sentinel.c0_min_and_ref.w);
	if ((distances.x < 1e38 && root_ref != 0u)) {
		bvh_raycast_push(iter, root_ref, distances.x);
	}
	return iter;
}

BVHRaycastHit bvh_iter_next(inout BVHRaycastIter iter, float max_dist) {
	BVHRaycastHit miss;
	miss.valid = false;

	let ray_pos = iter.ray_pos;
	let inv_dir = iter.inv_dir;

	for (;;) {
		if (iter.top <= 0) { break; }
		iter.top -= 1;
		let entry = iter.stack[iter.top];
		if (entry.dist >= max_dist) { continue; }

		if ((entry.idx & BVH_RAYCAST_ITEM_FLAG) != 0u) {
			let item_index = entry.idx ^ BVH_RAYCAST_ITEM_FLAG;
			let item = bvh_items[item_index];
			let size = unpack4xU8(item.aabb_size);
			let min_corner = float3(item.min_x, item.min_y, item.min_z);
			let max_corner = min_corner + float3(float(size.x), float(size.y), float(size.z)) * item.scale;
			let distances = ray_aabb(ray_pos, inv_dir, min_corner, max_corner);
			if ((distances.x < max_dist)) {
				return BVHRaycastHit(item_index, distances.x, distances.y - distances.x, true);
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
				let distances = ray_aabb(ray_pos, inv_dir, min_corner, max_corner);
				if (distances.x < max_dist) { bvh_raycast_push(iter, BVH_RAYCAST_ITEM_FLAG | item_index, distances.x); }
			}
		} else {
			let child0_hit = ray_aabb(ray_pos, inv_dir, node.c0_min_and_ref.xyz,   node.c0_max_and_ref2.xyz);
			let child1_hit = ray_aabb(ray_pos, inv_dir, node.c1_min_and_flags.xyz, node.c1_max_pad.xyz);
			let child0_hit_ok = child0_hit.x < max_dist;
			let child1_hit_ok = child1_hit.x < max_dist;

			if ((child0_hit_ok && child1_hit_ok)) {
				if ((child0_hit.x < child1_hit.x)) {
					bvh_raycast_push(iter, child1_ref, child1_hit.x);
					bvh_raycast_push(iter, child0_ref, child0_hit.x);
				} else {
					bvh_raycast_push(iter, child0_ref, child0_hit.x);
					bvh_raycast_push(iter, child1_ref, child1_hit.x);
				}
			} else if (child0_hit_ok) { bvh_raycast_push(iter, child0_ref, child0_hit.x); }
			  else if (child1_hit_ok) { bvh_raycast_push(iter, child1_ref, child1_hit.x); }
		}
	}

	return miss;
}
