#include "bvh/data.slang"
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

struct CameraUniform {
	float4x4 camera_transform;
	float2 camera_view_size;
};
ConstantBuffer<CameraUniform> camera : register(b0, space0);

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

float3 quat_rotate(float4 q, float3 v) {
	let t = 2.0 * cross(q.xyz, v);
	return v + q.w * t + cross(q.xyz, t);
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
	let bvh_item_idx = data.y;
	let voxel_data_index = data.z & 0xFFFFu;
	let voxel_index = data.z >> 16u;

	let item = bvh_items[bvh_item_idx];
	let item_index_2 = item.item_index_2;
	float3 base_color;
	if ((item.aabb_size >> 24u) == 2u) {
		base_color = main_voxel_reader_color(item_index_2, voxel_data_index, voxel_index).xyz;
	} else {
		base_color = residency_voxel_reader_color(item_index_2, voxel_data_index, voxel_index).xyz;
	}
	float3 normal_vec = float3(0.0);
	normal_vec[(normal >> 3u) & 0xFu] = ((normal & (1u << ((normal >> 3u) & 0xFu))) != 0u) ? -1.0 : 1.0;
	let transform_quat = float4(item.quat_x, item.quat_y, item.quat_z, item.quat_w);
	let color = shade(base_color, quat_rotate(transform_quat, normal_vec), light_visible);
	return float4(color, 1.0);
}
