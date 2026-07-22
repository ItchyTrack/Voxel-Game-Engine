#pragma once

#include "bvh/raycast.slang"

namespace BVH {

struct BeamRaycastPolicy : IRaycastPolicy {
	float expand_k;

	float2 hit_aabb(float3 ray_pos, float3 inv_dir, float3 aabb_min, float3 aabb_max) {
		let center = (aabb_min + aabb_max) * 0.5;
		let expand = length(center - ray_pos) * expand_k;
		return ray_aabb(ray_pos, inv_dir, aabb_min - float3(expand), aabb_max + float3(expand));
	}
};

}
