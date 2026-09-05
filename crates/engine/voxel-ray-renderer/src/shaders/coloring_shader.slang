#include "coloring_common.slang"

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

struct RenderSettings {
	uint4 values;
};
ConstantBuffer<RenderSettings> render_settings : register(b1, space0);

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

float3 id_to_color(uint id) {
	let h = hash_uint(id);
	let r = float((h >> 0u) & 0xFFu) / 255.0;
	let g = float((h >> 8u) & 0xFFu) / 255.0;
	let b = float((h >> 16u) & 0xFFu) / 255.0;
	return float3(r, g, b);
}

#include "anti_aliasing.slang"

[shader("fragment")]
float4 fs_main(VertexOutput input) : SV_Target {
	uint texture_width;
	uint texture_height;
	intermediate_textured_read.GetDimensions(texture_width, texture_height);
	let texture_size = uint2(texture_width, texture_height);
	let texture_pos = uint2(input.clip_position.xy);

	if (render_settings.values.y != 0u) {
		return anti_aliased_voxel_color(texture_pos, texture_size);
	}

	let data = intermediate_textured_read.Load(int3(int2(texture_pos), 0));
	let contribution = calculate_voxel_contribution(data, float2(texture_pos) + float2(0.5), texture_size);
	if (!contribution.hit) {
		discard;
		return float4(0.0);
	}
	return float4(color_voxel_surface(
		contribution,
		contribution.surface_pos,
		voxel_contribution_world_normal(contribution)
	), 1.0);
}
