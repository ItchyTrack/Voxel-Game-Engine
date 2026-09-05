#pragma once

float2 ray_aabb_axis(float ray_origin, float ray_dir, float aabb_min, float aabb_max) {
	if (ray_dir == 0.0) {
		return ray_origin >= aabb_min && ray_origin <= aabb_max
			? float2(-1e38, 1e38)
			: float2(1e38, -1e38);
	}
	let distance_to_min = (aabb_min - ray_origin) / ray_dir;
	let distance_to_max = (aabb_max - ray_origin) / ray_dir;
	return float2(min(distance_to_min, distance_to_max), max(distance_to_min, distance_to_max));
}

float2 ray_aabb(float3 ray_origin, float3 ray_dir, float3 aabb_min, float3 aabb_max) {
	let x_distances = ray_aabb_axis(ray_origin.x, ray_dir.x, aabb_min.x, aabb_max.x);
	let y_distances = ray_aabb_axis(ray_origin.y, ray_dir.y, aabb_min.y, aabb_max.y);
	let z_distances = ray_aabb_axis(ray_origin.z, ray_dir.z, aabb_min.z, aabb_max.z);
	let entry_distance = max(max(x_distances.x, y_distances.x), z_distances.x);
	let exit_distance = min(min(x_distances.y, y_distances.y), z_distances.y);
	if (exit_distance < 0.0 || entry_distance > exit_distance) { return float2(1e38, 0.0); }
	return float2(max(entry_distance, 0.0), exit_distance);
}
