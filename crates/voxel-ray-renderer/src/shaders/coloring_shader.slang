#include "bvh/data.slang"
#include "helpers/quat.slang"
#include "voxel_reader.slang"

struct VertexOutput {
	float4 clip_position : SV_Position;
	float2 screen_pos : TEXCOORD0;
};

[shader("vertex")]
VertexOutput vs_main(uint vertex_index : SV_VertexID) {
	VertexOutput out;
	out.screen_pos = float2(float((vertex_index << 1u) & 2u), float(vertex_index & 2u));
	out.clip_position = float4(out.screen_pos * 2.0 - 1.0, 0.0, 1.0);
	return out;
}

Texture2D<uint4> intermediate_textured_read : register(t0, space3);

struct ViewUniform {
	float4x4 clip_from_world;
	float4x4 unjittered_clip_from_world;
	float4x4 world_from_clip;
	float4x4 world_from_view;
	float4x4 view_from_world;
	float4x4 clip_from_view;
	float4x4 view_from_clip;
	float3 world_position;
	float exposure;
};
ConstantBuffer<ViewUniform> view : register(b0, space0);

float3 voxel_color(uint value) {
	let h = float(value % 17u) / 17.0 * 6.0;
	let s = uint(h);
	let f = h - float(s);
	let q = 1.0 - f;
	switch (s % 6u) {
		case 0u: return float3(1.0, f, 0.0);
		case 1u: return float3(q, 1.0, 0.0);
		case 2u: return float3(0.0, 1.0, f);
		case 3u: return float3(0.0, q, 1.0);
		case 4u: return float3(f, 0.0, 1.0);
		default: return float3(1.0, 0.0, q);
	}
}

float3 shade(float3 base_color, float3 world_normal, bool light_visible) {
	let light_dir = normalize(float3(0.5, 1.0, 0.2));
	let ndotl = max(dot(world_normal, light_dir), 0.0);
	let shadow = light_visible ? ndotl : ndotl * 0.1;
	let ambient = 0.25;
	return base_color * (ambient + (1.0 - ambient) * shadow);
}

uint hash_uint(uint x) {
	uint h = x;
	h ^= h >> 16u;
	h *= 0x7feb352du;
	h ^= h >> 15u;
	h *= 0x846ca68bu;
	h ^= h >> 16u;
	return h;
}

float3 id_to_color(uint id) {
	let h = hash_uint(id);
	let r = float((h >> 0u) & 0xFFu) / 255.0;
	let g = float((h >> 8u) & 0xFFu) / 255.0;
	let b = float((h >> 16u) & 0xFFu) / 255.0;
	return float3(r, g, b);
}

float2 subpixel_jitter(uint2 pixel, uint sample_index) {
	let seed = hash_uint(pixel.x + pixel.y * 0x9e3779b9u + sample_index * 0x85ebca6bu);
	let jitter_x = hash_uint(seed) & 0x00FFFFFFu;
	let jitter_y = hash_uint(seed ^ 0xc2b2ae35u) & 0x00FFFFFFu;
	return float2(float(jitter_x), float(jitter_y)) / 16777216.0;
}

struct RaycastKernel {
	uint4 top_left;
	uint4 top_right;
	uint4 bottom_left;
	uint4 bottom_right;
};

RaycastKernel load_raycast_kernel(uint2 texture_pos) {
	RaycastKernel kernel;
	kernel.top_left = intermediate_textured_read.Load(int3(int2(texture_pos), 0));
	kernel.top_right = intermediate_textured_read.Load(int3(int2(texture_pos + uint2(1, 0)), 0));
	kernel.bottom_left = intermediate_textured_read.Load(int3(int2(texture_pos + uint2(0, 1)), 0));
	kernel.bottom_right = intermediate_textured_read.Load(int3(int2(texture_pos + uint2(1, 1)), 0));
	return kernel;
}

struct VoxelContribution {
	bool hit;
	bool light_visible;
	uint normal;
	uint camera_face_occlusion_mask;
	uint bvh_item_idx;
	uint voxel_data_index;
	uint voxel_index;
	float distance;
	float3 local_voxel_min;
	float3 surface_pos;
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

bool same_voxel(VoxelContribution a, VoxelContribution b) {
	return a.hit
		&& b.hit
		&& a.bvh_item_idx == b.bvh_item_idx
		&& a.voxel_data_index == b.voxel_data_index
		&& a.voxel_index == b.voxel_index
		&& all(a.local_voxel_min == b.local_voxel_min);
}

VoxelContribution calculate_voxel_contribution(uint4 data, uint2 texture_pos, uint2 texture_size) {
	VoxelContribution contribution = {};
	if (data.x == 0u) { return contribution; }

	contribution.hit = true;
	contribution.normal = data.x & 0xFFu;
	contribution.light_visible = (data.x & 256u) != 0u;
	contribution.camera_face_occlusion_mask = (data.x >> 11u) & 7u;
	contribution.bvh_item_idx = data.y;
	contribution.voxel_data_index = data.z & 0xFFFFu;
	contribution.voxel_index = data.z >> 16u;

	let item = bvh_items[contribution.bvh_item_idx];
	let distance = asfloat(data.w);
	contribution.distance = distance;
	let screen_pos = float2(texture_pos) / float2(texture_size - uint2(1));
	let ray_clip = float4(screen_pos.x * 2.0 - 1.0, 1.0 - screen_pos.y * 2.0, 1.0, 1.0);
	let ray_view_h = mul(view.view_from_clip, ray_clip);
	let ray_view = ray_view_h.xyz / ray_view_h.w;
	let ray_dir = normalize(mul(view.world_from_view, float4(ray_view, 0.0)).xyz);
	let world_hit = view.world_position + ray_dir * distance;
	let transform_quat = float4(item.quat_x, item.quat_y, item.quat_z, item.quat_w);
	let local_hit = Quat::inv_rotate(transform_quat, world_hit - float3(item.pos_x, item.pos_y, item.pos_z)) / item.scale;
	var surface_pos = frac(local_hit);
	contribution.local_voxel_min = floor(local_hit);
	let normal_axis = (contribution.normal >> 3u) & 0xFu;
	if (normal_axis < 3u) {
		surface_pos[normal_axis] = (contribution.normal & (1u << normal_axis)) != 0u ? 0.0 : 1.0;
		contribution.local_voxel_min[normal_axis] = round(local_hit[normal_axis]) - surface_pos[normal_axis];
	}
	contribution.surface_pos = surface_pos;
	return contribution;
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

float3 color_voxel_surface(VoxelContribution contribution, float3 surface_pos, float3 world_normal) {
	let item = bvh_items[contribution.bvh_item_idx];
	float3 base_color;
	if ((item.aabb_size >> 24u) == 2u) {
		base_color = main_voxel_sampler_color(
			item.voxel_type_id,
			item.item_index_2,
			contribution.voxel_data_index,
			contribution.voxel_index,
			surface_pos
		).xyz;
	} else {
		base_color = residency_voxel_sampler_color(
			item.voxel_type_id,
			item.item_index_2,
			contribution.voxel_data_index,
			contribution.voxel_index,
			surface_pos
		).xyz;
	}
	return shade(base_color, world_normal, contribution.light_visible);
}

float3 voxel_contribution_world_normal(VoxelContribution contribution) {
	let item = bvh_items[contribution.bvh_item_idx];
	let transform_quat = float4(item.quat_x, item.quat_y, item.quat_z, item.quat_w);
	let normal_axis = (contribution.normal >> 3u) & 0xFu;
	float3 local_normal = float3(0.0);
	local_normal[normal_axis] = (contribution.normal & (1u << normal_axis)) != 0u ? -1.0 : 1.0;
	return Quat::rotate(transform_quat, local_normal);
}

float3 voxel_contribution_world_surface_position(VoxelContribution contribution) {
	let item = bvh_items[contribution.bvh_item_idx];
	let transform_quat = float4(item.quat_x, item.quat_y, item.quat_z, item.quat_w);
	let local_surface_position = (contribution.local_voxel_min + contribution.surface_pos) * item.scale;
	return float3(item.pos_x, item.pos_y, item.pos_z) + Quat::rotate(transform_quat, local_surface_position);
}

float3 world_ray_vector(float2 texture_position, float2 texture_denominator) {
	let screen_pos = texture_position / texture_denominator;
	let ray_clip = float4(screen_pos.x * 2.0 - 1.0, 1.0 - screen_pos.y * 2.0, 1.0, 1.0);
	let ray_view_h = mul(view.view_from_clip, ray_clip);
	let ray_view = ray_view_h.xyz / ray_view_h.w;
	return mul(view.world_from_view, float4(ray_view, 0.0)).xyz;
}

struct AnisotropicSamplingCandidate {
	bool valid;
	bool supports_anisotropy;
	float2 direction;
	float weight;
};

AnisotropicSamplingCandidate calculate_anisotropic_sampling_candidate(
	VoxelContribution contribution,
	float3 camera_position,
	float3 center_ray_vector,
	float3 ray_vector_dx,
	float3 ray_vector_dy
) {
	AnisotropicSamplingCandidate candidate = {};
	if (!contribution.hit) { return candidate; }

	candidate.valid = true;
	let item = bvh_items[contribution.bvh_item_idx];
	let transform_quat = float4(item.quat_x, item.quat_y, item.quat_z, item.quat_w);
	let world_normal = voxel_contribution_world_normal(contribution);
	let world_surface_position = voxel_contribution_world_surface_position(contribution);
	let plane_offset = dot(world_surface_position - camera_position, world_normal);
	let ray_plane_dot = dot(center_ray_vector, world_normal);
	if (abs(ray_plane_dot) <= length(center_ray_vector) * 1e-5) { return candidate; }

	// Differentiate the camera ray's intersection with the candidate face plane.
	let derivative_scale = plane_offset / (ray_plane_dot * ray_plane_dot);
	let world_surface_dx = derivative_scale * (
		ray_vector_dx * ray_plane_dot - center_ray_vector * dot(ray_vector_dx, world_normal)
	);
	let world_surface_dy = derivative_scale * (
		ray_vector_dy * ray_plane_dot - center_ray_vector * dot(ray_vector_dy, world_normal)
	);
	let local_surface_dx = Quat::inv_rotate(transform_quat, world_surface_dx) / item.scale;
	let local_surface_dy = Quat::inv_rotate(transform_quat, world_surface_dy) / item.scale;

	let xx = dot(local_surface_dx, local_surface_dx);
	let xy = dot(local_surface_dx, local_surface_dy);
	let yy = dot(local_surface_dy, local_surface_dy);
	let trace = xx + yy;
	if (trace <= 1e-12) { return candidate; }

	let eigenvalue_delta = sqrt(max((xx - yy) * (xx - yy) + 4.0 * xy * xy, 0.0));
	let major_eigenvalue = 0.5 * (trace + eigenvalue_delta);
	let minor_eigenvalue = 0.5 * (trace - eigenvalue_delta);
	let anisotropy_squared = major_eigenvalue / max(minor_eigenvalue, major_eigenvalue * 1e-6);
	let minimum_anisotropy_squared = 4.0;
	if (anisotropy_squared < minimum_anisotropy_squared) { return candidate; }

	let major_angle = 0.5 * atan2(2.0 * xy, xx - yy);
	candidate.supports_anisotropy = true;
	candidate.direction = float2(cos(major_angle), sin(major_angle));
	candidate.weight = sqrt(major_eigenvalue);
	return candidate;
}

struct AnisotropicSamplingState {
	bool coherent;
	uint candidate_count;
	float2 reference_direction;
	float2 direction_sum;
};

AnisotropicSamplingState add_anisotropic_sampling_candidate(
	AnisotropicSamplingState state,
	AnisotropicSamplingCandidate candidate
) {
	if (!candidate.valid || !state.coherent) { return state; }
	if (!candidate.supports_anisotropy) {
		state.coherent = false;
		return state;
	}

	if (state.candidate_count == 0u) {
		state.reference_direction = candidate.direction;
		state.direction_sum = candidate.direction * candidate.weight;
		state.candidate_count = 1u;
		return state;
	}

	let alignment = dot(state.reference_direction, candidate.direction);
	let minimum_direction_alignment = 0.9238795;
	if (abs(alignment) < minimum_direction_alignment) {
		state.coherent = false;
		return state;
	}
	let aligned_direction = alignment < 0.0 ? -candidate.direction : candidate.direction;
	state.direction_sum += aligned_direction * candidate.weight;
	state.candidate_count++;
	return state;
}

[shader("fragment")]
float4 fs_main(VertexOutput input) : SV_Target {
	uint texture_width;
	uint texture_height;
	intermediate_textured_read.GetDimensions(texture_width, texture_height);
	uint2 texture_size = uint2(texture_width, texture_height);
	uint2 texture_pos = uint2(input.clip_position.xy);
	let kernel = load_raycast_kernel(texture_pos);
	var top_left = calculate_voxel_contribution(kernel.top_left, texture_pos, texture_size);
	var top_right = calculate_voxel_contribution(kernel.top_right, texture_pos + uint2(1, 0), texture_size);
	var bottom_left = calculate_voxel_contribution(kernel.bottom_left, texture_pos + uint2(0, 1), texture_size);
	var bottom_right = calculate_voxel_contribution(kernel.bottom_right, texture_pos + uint2(1, 1), texture_size);
	let top_left_corner_valid = top_left.hit;
	let top_right_corner_valid = top_right.hit;
	let bottom_left_corner_valid = bottom_left.hit;
	let bottom_right_corner_valid = bottom_right.hit;

	if (same_voxel(top_right, top_left)) {
		top_right.hit = false;
	}
	if (same_voxel(bottom_left, top_left) || same_voxel(bottom_left, top_right)) {
		bottom_left.hit = false;
	}
	if (same_voxel(bottom_right, top_left) || same_voxel(bottom_right, top_right) || same_voxel(bottom_right, bottom_left)) {
		bottom_right.hit = false;
	}

	if (!(top_left.hit || top_right.hit || bottom_left.hit || bottom_right.hit)) {
		discard;
		return float4(0.0);
	}

	let camera_position = view.world_position;
	let top_left_raycast_data = prepare_voxel_contribution_raycast(top_left, camera_position);
	let top_right_raycast_data = prepare_voxel_contribution_raycast(top_right, camera_position);
	let bottom_left_raycast_data = prepare_voxel_contribution_raycast(bottom_left, camera_position);
	let bottom_right_raycast_data = prepare_voxel_contribution_raycast(bottom_right, camera_position);

	let texture_denominator = float2(texture_size - uint2(1));
	let pixel_center = float2(texture_pos) + float2(0.5);
	let center_ray_vector = world_ray_vector(pixel_center, texture_denominator);
	let ray_vector_dx = world_ray_vector(pixel_center + float2(1.0, 0.0), texture_denominator) - center_ray_vector;
	let ray_vector_dy = world_ray_vector(pixel_center + float2(0.0, 1.0), texture_denominator) - center_ray_vector;
	let top_left_sampling_candidate = calculate_anisotropic_sampling_candidate(
		top_left, camera_position, center_ray_vector, ray_vector_dx, ray_vector_dy
	);
	let top_right_sampling_candidate = calculate_anisotropic_sampling_candidate(
		top_right, camera_position, center_ray_vector, ray_vector_dx, ray_vector_dy
	);
	let bottom_left_sampling_candidate = calculate_anisotropic_sampling_candidate(
		bottom_left, camera_position, center_ray_vector, ray_vector_dx, ray_vector_dy
	);
	let bottom_right_sampling_candidate = calculate_anisotropic_sampling_candidate(
		bottom_right, camera_position, center_ray_vector, ray_vector_dx, ray_vector_dy
	);
	AnisotropicSamplingState sampling_state = {};
	sampling_state.coherent = true;
	sampling_state = add_anisotropic_sampling_candidate(sampling_state, top_left_sampling_candidate);
	sampling_state = add_anisotropic_sampling_candidate(sampling_state, top_right_sampling_candidate);
	sampling_state = add_anisotropic_sampling_candidate(sampling_state, bottom_left_sampling_candidate);
	sampling_state = add_anisotropic_sampling_candidate(sampling_state, bottom_right_sampling_candidate);

	let use_anisotropic_sampling = sampling_state.coherent && sampling_state.candidate_count != 0u;
	let major_direction = use_anisotropic_sampling ? normalize(sampling_state.direction_sum) : float2(1.0, 0.0);
	let minor_direction = float2(-major_direction.y, major_direction.x);
	let sample_grid_size = use_anisotropic_sampling ? uint2(8u, 2u) : uint2(4u, 4u);

	float3 accumulated_color = float3(0.0);
	uint ray_hit_count = 0u;
	for (uint sample_index = 0u; sample_index < 16u; sample_index++) {
		let sample_cell = uint2(sample_index % sample_grid_size.x, sample_index / sample_grid_size.x);
		let jitter = subpixel_jitter(texture_pos, sample_index);
		let stratified_position = (float2(sample_cell) + jitter) / float2(sample_grid_size) - float2(0.5);
		let rotated_sample_offset = float2(0.5) + major_direction * stratified_position.x + minor_direction * stratified_position.y;
		let sample_offset = frac(rotated_sample_offset);
		let ray_direction = normalize(world_ray_vector(float2(texture_pos) + sample_offset, texture_denominator));

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
