#pragma once

#include "bvh/data.slang"
#include "helpers/quat.slang"
#include "voxel_reader.slang"

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

VoxelContribution calculate_voxel_contribution(uint4 data, float2 texture_position, uint2 texture_size) {
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
	let screen_pos = texture_position / float2(texture_size - uint2(1));
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

float3 world_ray_vector(float2 texture_position, float2 texture_denominator) {
	let screen_pos = texture_position / texture_denominator;
	let ray_clip = float4(screen_pos.x * 2.0 - 1.0, 1.0 - screen_pos.y * 2.0, 1.0, 1.0);
	let ray_view_h = mul(view.view_from_clip, ray_clip);
	let ray_view = ray_view_h.xyz / ray_view_h.w;
	return mul(view.world_from_view, float4(ray_view, 0.0)).xyz;
}
