#include "coloring_common.slang"

struct RaycastKernel {
	uint4 top_left;
	uint4 top_right;
	uint4 bottom_left;
	uint4 bottom_right;
};

struct VoxelContributionRaycastData {
	bool valid;
	uint camera_face_occlusion_mask;
	float3 ray_origin_relative_min;
	float3 ray_origin_relative_max;
	float4 transform_quat;
	float inverse_scale;
};

struct VoxelContributionRaycastHit {
	bool hit;
	bool face_occluded;
	float distance;
	float3 normal;
	float3 surface_pos;
};

struct AnisotropicSamplingState {
	bool anisotropic;
	float face_dot;
	float2 direction;
};

static const float ANISOTROPIC_FACE_DOT_CUTOFF = 0.383;

float2 subpixel_jitter(uint2 pixel, uint sample_index) {
	let seed = hash_uint(pixel.x + pixel.y * 0x9e3779b9u + sample_index * 0x85ebca6bu);
	let jitter_x = hash_uint(seed) & 0x00FFFFFFu;
	let jitter_y = hash_uint(seed ^ 0xc2b2ae35u) & 0x00FFFFFFu;
	return float2(float(jitter_x), float(jitter_y)) / 16777216.0;
}

RaycastKernel load_raycast_kernel(uint2 texture_pos) {
	RaycastKernel kernel;
	kernel.top_left = intermediate_textured_read.Load(int3(int2(texture_pos), 0));
	kernel.top_right = intermediate_textured_read.Load(int3(int2(texture_pos + uint2(1, 0)), 0));
	kernel.bottom_left = intermediate_textured_read.Load(int3(int2(texture_pos + uint2(0, 1)), 0));
	kernel.bottom_right = intermediate_textured_read.Load(int3(int2(texture_pos + uint2(1, 1)), 0));
	return kernel;
}

bool same_voxel(VoxelContribution a, VoxelContribution b) {
	return a.hit
		&& b.hit
		&& a.bvh_item_idx == b.bvh_item_idx
		&& a.voxel_data_index == b.voxel_data_index
		&& a.voxel_index == b.voxel_index
		&& all(a.local_voxel_min == b.local_voxel_min);
}

VoxelContributionRaycastData prepare_voxel_contribution_raycast(VoxelContribution contribution, float3 world_ray_origin) {
	VoxelContributionRaycastData data = {};
	if (!contribution.hit) { return data; }

	let item = bvh_items[contribution.bvh_item_idx];
	data.valid = true;
	data.camera_face_occlusion_mask = contribution.camera_face_occlusion_mask;
	data.transform_quat = float4(item.quat_x, item.quat_y, item.quat_z, item.quat_w);
	data.inverse_scale = 1.0 / item.scale;
	let item_pos = float3(item.pos_x, item.pos_y, item.pos_z);
	let local_ray_origin = Quat::inv_rotate(data.transform_quat, world_ray_origin - item_pos);
	data.ray_origin_relative_min = contribution.local_voxel_min * item.scale - local_ray_origin;
	data.ray_origin_relative_max = data.ray_origin_relative_min + float3(item.scale);
	return data;
}

VoxelContributionRaycastHit raycast_voxel_contribution(VoxelContributionRaycastData data, float3 normalized_world_ray_dir) {
	VoxelContributionRaycastHit result = {};
	if (!data.valid) { return result; }

	let local_ray_dir = Quat::inv_rotate(data.transform_quat, normalized_world_ray_dir);
	if (
		(local_ray_dir.x == 0.0 && (data.ray_origin_relative_min.x > 0.0 || data.ray_origin_relative_max.x < 0.0)) ||
		(local_ray_dir.y == 0.0 && (data.ray_origin_relative_min.y > 0.0 || data.ray_origin_relative_max.y < 0.0)) ||
		(local_ray_dir.z == 0.0 && (data.ray_origin_relative_min.z > 0.0 || data.ray_origin_relative_max.z < 0.0))
	) {
		return result;
	}

	let inv_local_ray_dir = float3(
		local_ray_dir.x != 0.0 ? 1.0 / local_ray_dir.x : 1e38,
		local_ray_dir.y != 0.0 ? 1.0 / local_ray_dir.y : 1e38,
		local_ray_dir.z != 0.0 ? 1.0 / local_ray_dir.z : 1e38
	);
	let distances_to_min = data.ray_origin_relative_min * inv_local_ray_dir;
	let distances_to_max = data.ray_origin_relative_max * inv_local_ray_dir;
	let entry_distances = min(distances_to_min, distances_to_max);
	let exit_distances = max(distances_to_min, distances_to_max);
	let entry_distance = max(max(entry_distances.x, entry_distances.y), entry_distances.z);
	let exit_distance = min(min(exit_distances.x, exit_distances.y), exit_distances.z);
	if (exit_distance < 0.0 || entry_distance > exit_distance) { return result; }

	let normal_axis = entry_distances.x >= entry_distances.y && entry_distances.x >= entry_distances.z
		? 0u
		: (entry_distances.y >= entry_distances.z ? 1u : 2u);
	float3 local_normal = float3(0.0);
	local_normal[normal_axis] = local_ray_dir[normal_axis] >= 0.0 ? -1.0 : 1.0;

	result.hit = true;
	result.face_occluded = (data.camera_face_occlusion_mask & (1u << normal_axis)) != 0u;
	result.distance = max(entry_distance, 0.0);
	result.normal = Quat::rotate(data.transform_quat, local_normal);
	let local_hit = local_ray_dir * result.distance;
	result.surface_pos = clamp((local_hit - data.ray_origin_relative_min) * data.inverse_scale, float3(0.0), float3(1.0));
	return result;
}

uint anisotropic_sampling_face_mask(VoxelContribution contribution) {
	if (!contribution.hit) { return 0u; }

	uint face_mask = 0u;
	for (uint axis = 0u; axis < 3u; axis++) {
		if ((contribution.camera_face_occlusion_mask & (1u << axis)) != 0u) { continue; }

		let face_sign = (contribution.normal >> axis) & 1u;
		face_mask |= 1u << (axis * 2u + face_sign);
	}
	return face_mask;
}

AnisotropicSamplingState add_anisotropic_sampling_faces(
	AnisotropicSamplingState state,
	VoxelContribution contribution,
	uint face_mask,
	float3 view_direction
) {
	if (!contribution.hit || face_mask == 0u) { return state; }

	let item = bvh_items[contribution.bvh_item_idx];
	let transform_quat = float4(item.quat_x, item.quat_y, item.quat_z, item.quat_w);
	for (uint axis = 0u; axis < 3u; axis++) {
		for (uint face_sign = 0u; face_sign < 2u; face_sign++) {
			if ((face_mask & (1u << (axis * 2u + face_sign))) == 0u) { continue; }

			float3 local_normal = float3(0.0);
			local_normal[axis] = face_sign != 0u ? -1.0 : 1.0;
			let world_normal = Quat::rotate(transform_quat, local_normal);
			let face_dot = abs(dot(world_normal, view_direction));
			if (
				face_dot >= ANISOTROPIC_FACE_DOT_CUTOFF ||
				(state.anisotropic && face_dot >= state.face_dot)
			) {
				continue;
			}

			let view_normal = mul(view.view_from_world, float4(world_normal, 0.0)).xyz;
			let projected_normal = float2(view_normal.x, -view_normal.y);
			if (dot(projected_normal, projected_normal) <= 1e-12) { continue; }

			state.anisotropic = true;
			state.face_dot = face_dot;
			state.direction = normalize(projected_normal);
		}
	}
	return state;
}

float4 anti_aliased_voxel_color(uint2 texture_pos, uint2 texture_size) {
	let kernel = load_raycast_kernel(texture_pos);
	var top_left = calculate_voxel_contribution(kernel.top_left, float2(texture_pos), texture_size);
	var top_right = calculate_voxel_contribution(kernel.top_right, float2(texture_pos + uint2(1, 0)), texture_size);
	var bottom_left = calculate_voxel_contribution(kernel.bottom_left, float2(texture_pos + uint2(0, 1)), texture_size);
	var bottom_right = calculate_voxel_contribution(kernel.bottom_right, float2(texture_pos + uint2(1, 1)), texture_size);
	let top_left_corner_valid = top_left.hit;
	let top_right_corner_valid = top_right.hit;
	let bottom_left_corner_valid = bottom_left.hit;
	let bottom_right_corner_valid = bottom_right.hit;
	if (!(top_left_corner_valid || top_right_corner_valid || bottom_left_corner_valid || bottom_right_corner_valid)) {
		discard;
		return float4(0.0);
	}

	let camera_position = view.world_position;
	let texture_denominator = float2(texture_size - uint2(1));
	let pixel_min = float2(texture_pos);
	let pixel_center = pixel_min + float2(0.5);
	let view_direction = normalize(world_ray_vector(pixel_center, texture_denominator));
	uint top_left_face_mask = anisotropic_sampling_face_mask(top_left);
	uint top_right_face_mask = anisotropic_sampling_face_mask(top_right);
	uint bottom_left_face_mask = anisotropic_sampling_face_mask(bottom_left);
	uint bottom_right_face_mask = anisotropic_sampling_face_mask(bottom_right);

	if (same_voxel(top_right, top_left)) {
		top_left_face_mask |= top_right_face_mask;
		top_right.hit = false;
	}

	if (same_voxel(bottom_left, top_left)) {
		top_left_face_mask |= bottom_left_face_mask;
		bottom_left.hit = false;
	} else if (same_voxel(bottom_left, top_right)) {
		top_right_face_mask |= bottom_left_face_mask;
		bottom_left.hit = false;
	}

	if (same_voxel(bottom_right, top_left)) {
		top_left_face_mask |= bottom_right_face_mask;
		bottom_right.hit = false;
	} else if (same_voxel(bottom_right, top_right)) {
		top_right_face_mask |= bottom_right_face_mask;
		bottom_right.hit = false;
	} else if (same_voxel(bottom_right, bottom_left)) {
		bottom_left_face_mask |= bottom_right_face_mask;
		bottom_right.hit = false;
	}

	AnisotropicSamplingState sampling_state = {};
	sampling_state = add_anisotropic_sampling_faces(sampling_state, top_left, top_left_face_mask, view_direction);
	sampling_state = add_anisotropic_sampling_faces(sampling_state, top_right, top_right_face_mask, view_direction);
	sampling_state = add_anisotropic_sampling_faces(sampling_state, bottom_left, bottom_left_face_mask, view_direction);
	sampling_state = add_anisotropic_sampling_faces(sampling_state, bottom_right, bottom_right_face_mask, view_direction);

	let top_left_raycast_data = prepare_voxel_contribution_raycast(top_left, camera_position);
	let top_right_raycast_data = prepare_voxel_contribution_raycast(top_right, camera_position);
	let bottom_left_raycast_data = prepare_voxel_contribution_raycast(bottom_left, camera_position);
	let bottom_right_raycast_data = prepare_voxel_contribution_raycast(bottom_right, camera_position);
	let major_direction = sampling_state.anisotropic ? sampling_state.direction : float2(1.0, 0.0);
	let minor_direction = float2(-major_direction.y, major_direction.x);
	let sample_grid_size = sampling_state.anisotropic ? uint2(8u, 2u) : uint2(4u, 4u);

	float3 accumulated_color = float3(0.0);
	uint ray_hit_count = 0u;
	for (uint sample_index = 0u; sample_index < 16u; sample_index++) {
		let sample_cell = uint2(sample_index % sample_grid_size.x, sample_index / sample_grid_size.x);
		let jitter = subpixel_jitter(texture_pos, sample_index);
		let stratified_position = (float2(sample_cell) + jitter) / float2(sample_grid_size) - float2(0.5);
		let rotated_sample_offset = float2(0.5) + major_direction * stratified_position.x + minor_direction * stratified_position.y;
		let sample_offset = frac(rotated_sample_offset);
		let ray_direction = normalize(world_ray_vector(pixel_min + sample_offset, texture_denominator));

		var closest_contribution = top_left;
		var closest_hit = raycast_voxel_contribution(top_left_raycast_data, ray_direction);
		{
			let hit = raycast_voxel_contribution(top_right_raycast_data, ray_direction);
			if (hit.hit && (!closest_hit.hit || hit.distance < closest_hit.distance)) {
				closest_contribution = top_right;
				closest_hit = hit;
			}
		}
		{
			let hit = raycast_voxel_contribution(bottom_left_raycast_data, ray_direction);
			if (hit.hit && (!closest_hit.hit || hit.distance < closest_hit.distance)) {
				closest_contribution = bottom_left;
				closest_hit = hit;
			}
		}
		{
			let hit = raycast_voxel_contribution(bottom_right_raycast_data, ray_direction);
			if (hit.hit && (!closest_hit.hit || hit.distance < closest_hit.distance)) {
				closest_contribution = bottom_right;
				closest_hit = hit;
			}
		}

		if (closest_hit.hit && !closest_hit.face_occluded) {
			accumulated_color += color_voxel_surface(closest_contribution, closest_hit.surface_pos, closest_hit.normal);
			ray_hit_count++;
		}
	}
	if (ray_hit_count != 0u) {
		return float4(accumulated_color / float(ray_hit_count), 1.0);
	}

	float3 fallback_color = float3(0.0);
	uint valid_corner_count = 0u;
	if (top_left_corner_valid) {
		fallback_color += color_voxel_surface(top_left, top_left.surface_pos, voxel_contribution_world_normal(top_left));
		valid_corner_count++;
	}
	if (top_right_corner_valid) {
		fallback_color += color_voxel_surface(top_right, top_right.surface_pos, voxel_contribution_world_normal(top_right));
		valid_corner_count++;
	}
	if (bottom_left_corner_valid) {
		fallback_color += color_voxel_surface(bottom_left, bottom_left.surface_pos, voxel_contribution_world_normal(bottom_left));
		valid_corner_count++;
	}
	if (bottom_right_corner_valid) {
		fallback_color += color_voxel_surface(bottom_right, bottom_right.surface_pos, voxel_contribution_world_normal(bottom_right));
		valid_corner_count++;
	}
	return float4(fallback_color / float(valid_corner_count), 1.0);
}
