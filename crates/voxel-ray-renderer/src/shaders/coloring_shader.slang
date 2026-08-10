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

[shader("fragment")]
float4 fs_main(VertexOutput input) : SV_Target {
	uint texture_width;
	uint texture_height;
	intermediate_textured_read.GetDimensions(texture_width, texture_height);
	uint2 texture_size = uint2(texture_width, texture_height);
	let texture_pos = uint2(
		uint(input.screen_pos.x * float(texture_size.x)),
		uint((1.0 - input.screen_pos.y) * float(texture_size.y))
	);
	let data = intermediate_textured_read.Load(int3(int2(texture_pos), 0));

	if (data.x == 0u) {
		discard;
		return float4(0.0);
	}

	let normal = data.x & 0xFFu;
	let light_visible = (data.x & 256u) != 0u;
	let sample_octant = (data.x >> 11u) & 7u;
	let bvh_item_idx = data.y;
	let voxel_data_index = data.z & 0xFFFFu;
	let voxel_index = data.z >> 16u;

	let item = bvh_items[bvh_item_idx];
	let item_index_2 = item.item_index_2;
	let distance = asfloat(data.w);
	let ray_clip = float4(input.screen_pos.x * 2.0 - 1.0, input.screen_pos.y * 2.0 - 1.0, 1.0, 1.0);
	let ray_world_h = mul(view.world_from_clip, ray_clip);
	let ray_dir = normalize(ray_world_h.xyz / ray_world_h.w - view.world_position);
	let world_hit = view.world_position + ray_dir * distance;
	let transform_quat = float4(item.quat_x, item.quat_y, item.quat_z, item.quat_w);
	let local_hit = Quat::inv_rotate(transform_quat, world_hit - float3(item.pos_x, item.pos_y, item.pos_z)) / item.scale;
	let raw_surface_pos = frac(local_hit);
	let half_low = asfloat(asuint(0.5) - 1u);
	let surface_pos = float3(
		(sample_octant & 1u) != 0u ? max(raw_surface_pos.x, 0.5) : min(raw_surface_pos.x, half_low),
		(sample_octant & 2u) != 0u ? max(raw_surface_pos.y, 0.5) : min(raw_surface_pos.y, half_low),
		(sample_octant & 4u) != 0u ? max(raw_surface_pos.z, 0.5) : min(raw_surface_pos.z, half_low)
	);
	float3 base_color;
	if ((item.aabb_size >> 24u) == 2u) {
		base_color = main_voxel_sampler_color(item.voxel_type_id, item_index_2, voxel_data_index, voxel_index, surface_pos).xyz;
	} else {
		base_color = residency_voxel_sampler_color(item.voxel_type_id, item_index_2, voxel_data_index, voxel_index, surface_pos).xyz;
	}
	float3 normal_vec = float3(0.0);
	normal_vec[(normal >> 3u) & 0xFu] = ((normal & (1u << ((normal >> 3u) & 0xFu))) != 0u) ? -1.0 : 1.0;
	let color = shade(base_color, Quat::rotate(transform_quat, normal_vec), light_visible);
	return float4(color, 1.0);
}
