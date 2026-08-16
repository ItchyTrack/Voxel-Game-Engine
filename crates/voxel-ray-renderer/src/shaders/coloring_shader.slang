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
	uint bvh_item_idx;
	uint voxel_data_index;
	uint voxel_index;
	float3 surface_pos;
};

bool same_voxel(VoxelContribution a, VoxelContribution b) {
	return a.hit && b.hit && a.bvh_item_idx == b.bvh_item_idx && a.voxel_data_index == b.voxel_data_index && a.voxel_index == b.voxel_index;
}

VoxelContribution calculate_voxel_contribution(uint4 data, uint2 texture_pos, uint2 texture_size) {
	VoxelContribution contribution = {};
	if (data.x == 0u) { return contribution; }

	contribution.hit = true;
	contribution.normal = data.x & 0xFFu;
	contribution.light_visible = (data.x & 256u) != 0u;
	let sample_octant = (data.x >> 11u) & 7u;
	contribution.bvh_item_idx = data.y;
	contribution.voxel_data_index = data.z & 0xFFFFu;
	contribution.voxel_index = data.z >> 16u;

	let item = bvh_items[contribution.bvh_item_idx];
	let distance = asfloat(data.w);
	let screen_pos = float2(texture_pos) / float2(texture_size - uint2(1));
	let ray_clip = float4(screen_pos.x * 2.0 - 1.0, 1.0 - screen_pos.y * 2.0, 1.0, 1.0);
	let ray_view_h = mul(view.view_from_clip, ray_clip);
	let ray_view = ray_view_h.xyz / ray_view_h.w;
	let ray_dir = normalize(mul(view.world_from_view, float4(ray_view, 0.0)).xyz);
	let world_hit = view.world_position + ray_dir * distance;
	let transform_quat = float4(item.quat_x, item.quat_y, item.quat_z, item.quat_w);
	let local_hit = Quat::inv_rotate(transform_quat, world_hit - float3(item.pos_x, item.pos_y, item.pos_z)) / item.scale;
	let raw_surface_pos = frac(local_hit);
	let half_low = asfloat(asuint(0.5) - 1u);
	contribution.surface_pos = float3(
		(sample_octant & 1u) != 0u ? max(raw_surface_pos.x, 0.5) : min(raw_surface_pos.x, half_low),
		(sample_octant & 2u) != 0u ? max(raw_surface_pos.y, 0.5) : min(raw_surface_pos.y, half_low),
		(sample_octant & 4u) != 0u ? max(raw_surface_pos.z, 0.5) : min(raw_surface_pos.z, half_low)
	);
	return contribution;
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

	if (same_voxel(top_right, top_left)) {
		top_right.hit = false;
	}
	if (same_voxel(bottom_left, top_left) || same_voxel(bottom_left, top_right)) {
		bottom_left.hit = false;
	}
	if (same_voxel(bottom_right, top_left) || same_voxel(bottom_right, top_right) || same_voxel(bottom_right, bottom_left)) {
		bottom_right.hit = false;
	}

	let num_hit = top_left.hit + top_right.hit + bottom_left.hit + bottom_right.hit;

	if (num_hit == 0) {
		discard;
		return float4(0.0);
	}

	if (num_hit < 3) {
		return float4(num_hit / 4.0, num_hit / 4.0, num_hit / 4.0, 1.0);
	} else if (num_hit == 3) {
		return float4(0.0, 1.0, 0.0, 1.0);
	} else {
		return float4(1.0, 0.0, 0.0, 1.0);
	}

	// if (!top_left.hit) {
	// 	discard;
	// 	return float4(0.0);
	// }

	// let item = bvh_items[top_left.bvh_item_idx];
	// let item_index_2 = item.item_index_2;
	// let transform_quat = float4(item.quat_x, item.quat_y, item.quat_z, item.quat_w);
	// float3 base_color;
	// if ((item.aabb_size >> 24u) == 2u) {
	// 	base_color = main_voxel_sampler_color(item.voxel_type_id, item_index_2, top_left.voxel_data_index, top_left.voxel_index, top_left.surface_pos).xyz;
	// } else {
	// 	base_color = residency_voxel_sampler_color(item.voxel_type_id, item_index_2, top_left.voxel_data_index, top_left.voxel_index, top_left.surface_pos).xyz;
	// }
	// float3 normal_vec = float3(0.0);
	// normal_vec[(top_left.normal >> 3u) & 0xFu] = ((top_left.normal & (1u << ((top_left.normal >> 3u) & 0xFu))) != 0u) ? -1.0 : 1.0;
	// let color = shade(base_color, Quat::rotate(transform_quat, normal_vec), top_left.light_visible);
	// return float4(color, 1.0);
}
