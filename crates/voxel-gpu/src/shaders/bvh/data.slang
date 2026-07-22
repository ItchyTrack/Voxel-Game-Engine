#pragma once

// Shared BVH data layout, bindings, and low-level helpers.

struct BVHNode {
	float4 c0_min_and_ref;
	float4 c0_max_and_ref2;
	float4 c1_min_and_flags;
	float4 c1_max_pad;
};

struct BVHItem {
	float min_x; float min_y; float min_z;
	uint aabb_size;
	uint item_index;
	uint item_index_2;
	float pos_x; float pos_y; float pos_z;
	float quat_x; float quat_y; float quat_z; float quat_w;
	float scale;
};

StructuredBuffer<BVHNode> bvh : register(t0, space1);
StructuredBuffer<BVHItem> bvh_items : register(t1, space1);