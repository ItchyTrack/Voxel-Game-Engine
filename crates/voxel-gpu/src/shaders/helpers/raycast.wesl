fn ray_aabb(ray_origin: vec3<f32>, inv_ray_dir: vec3<f32>, aabb_min: vec3<f32>, aabb_max: vec3<f32>) -> vec2<f32> {
	let distance_to_min_planes = (aabb_min - ray_origin) * inv_ray_dir;
	let distance_to_max_planes = (aabb_max - ray_origin) * inv_ray_dir;
	let entry_distance = max(
		max(
			min(distance_to_min_planes.x, distance_to_max_planes.x),
			min(distance_to_min_planes.y, distance_to_max_planes.y)
		),
		min(distance_to_min_planes.z, distance_to_max_planes.z)
	);
	let exit_distance = min(
		min(
			max(distance_to_min_planes.x, distance_to_max_planes.x),
			max(distance_to_min_planes.y, distance_to_max_planes.y)
		),
		max(distance_to_min_planes.z, distance_to_max_planes.z)
	);
	if exit_distance < 0.0 || entry_distance > exit_distance { return vec2<f32>(1e38, 0.0); }
	return vec2<f32>(max(entry_distance, 0.0), exit_distance);
}
