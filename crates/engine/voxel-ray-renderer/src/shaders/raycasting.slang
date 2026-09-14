#include "face_geometry.slang"
#include "combined_raycast.slang"
#include "occlusion_raycast.slang"

// 0 : camera uniform
// 1 : BVH (nodes + items)
// 2 : grid tree buffer
// 3 : ray marching textures

WTexture2D<uint4> intermediate_textured : register(u0, space3);
Texture2D<float> bvh_beam_textured_read : register(t1, space3);
Texture2D<float> scene_depth : register(t2, space3);
StructuredBuffer<uint> gi_words : register(t3, space3);

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

struct RenderSettings {
	uint4 values;
};
ConstantBuffer<RenderSettings> render_settings : register(b1, space0);

void raycast_primary(uint3 global_invocation_id, bool fallback) {
	uint texture_width;
	uint texture_height;
	intermediate_textured.GetDimensions(texture_width, texture_height);
	uint2 texture_size = uint2(texture_width, texture_height);
	let anti_aliasing = render_settings.values.y != 0u;
	let raycast_size = anti_aliasing ? texture_size : texture_size - uint2(1);
	let pixel = global_invocation_id.xy;
	if (pixel.x >= raycast_size.x || pixel.y >= raycast_size.y) { return; }

	let sample_position = float2(pixel) + (anti_aliasing ? float2(0.0) : float2(0.5));
	let screen_pos = sample_position / float2(texture_size - uint2(1));

	let ray_start = view.world_position;
	let ray_clip = float4(screen_pos.x * 2.0 - 1.0, 1.0 - screen_pos.y * 2.0, 1.0, 1.0);
	let ray_view_h = mul(view.view_from_clip, ray_clip);
	let ray_view = ray_view_h.xyz / ray_view_h.w;
	let ray_dir = normalize(mul(view.world_from_view, float4(ray_view, 0.0)).xyz);

	uint depth_width;
	uint depth_height;
	scene_depth.GetDimensions(depth_width, depth_height);
	let depth_size = uint2(depth_width, depth_height);
	let depth_min = anti_aliasing
		? max(int2(pixel) - int2(1), int2(0))
		: min(int2(pixel), int2(depth_size) - int2(1));
	let depth_max = min(int2(pixel), int2(depth_size) - int2(1));
	var max_dist = 0.0;
	for (int y = depth_min.y; y <= depth_max.y; y++) {
		for (int x = depth_min.x; x <= depth_max.x; x++) {
			let depth = scene_depth.Load(int3(int2(x, y), 0)).r;
			if (depth <= 0.0) {
				max_dist = 1e38;
			} else {
				let clip = float4(screen_pos.x * 2.0 - 1.0, 1.0 - screen_pos.y * 2.0, depth, 1.0);
				let world_h = mul(view.world_from_clip, clip);
				let depth_world = world_h.xyz / world_h.w;
				max_dist = max(max_dist, max(0.0, dot(depth_world - ray_start, ray_dir)));
			}
		}
	}

	uint beam_width;
	uint beam_height;
	bvh_beam_textured_read.GetDimensions(beam_width, beam_height);
	uint2 beam_size = uint2(beam_width, beam_height);
	let beam_coord_f = screen_pos * float2(beam_size) - float2(0.5);
	let beam_coord_i = int2(beam_coord_f);
	let bsz = int2(beam_size);
	let c00 = clamp(beam_coord_i, int2(0), bsz - int2(1));
	let c10 = clamp(beam_coord_i + int2(1, 0), int2(0), bsz - int2(1));
	let c01 = clamp(beam_coord_i + int2(0, 1), int2(0), bsz - int2(1));
	let c11 = clamp(beam_coord_i + int2(1, 1), int2(0), bsz - int2(1));

	let d00 = bvh_beam_textured_read.Load(int3(c00, 0)).r;
	let d10 = bvh_beam_textured_read.Load(int3(c10, 0)).r;
	let d01 = bvh_beam_textured_read.Load(int3(c01, 0)).r;
	let d11 = bvh_beam_textured_read.Load(int3(c11, 0)).r;
	let beam_min_dist = min(min(d00, d10), min(d01, d11));

	if (beam_min_dist >= min(1e37, max_dist)) {
		intermediate_textured.Store(pixel, uint4(0, 0, 0, 0));
		return;
	}

	let hit = full_raycast(ray_start + ray_dir * beam_min_dist, ray_dir, max_dist - beam_min_dist);

	if (hit.normal == 0u) {
		intermediate_textured.Store(pixel, uint4(0, 0, 0, 0));
		return;
	}

	uint shadow_counts = 0u;
	if (fallback) {
		let hit_axis = (hit.normal >> 3u) & 3u;
		let occluded = (hit.normal >> 11u) & 7u;
		for (uint axis = 0u; axis < 3u; axis++) {
			if (anti_aliasing ? (occluded & (1u << axis)) != 0u : axis != hit_axis) { continue; }
			let face = uint3(gi_canonical_normal(hit.normal, axis), hit.bvh_item_idx,
				hit.voxel_data_index | (hit.voxel_index << 16u));
			let geometry = gi_face_geometry(face);
			shadow_counts |= face_sun_visible_samples(face, geometry, render_settings.values.z) << (4u * axis);
		}
	}
	// Four bits per face preserve exact counts up to nine for AA fallback.
	// X uses normal bits 5..8; Y/Z use material bits 24..31 (voxel_index is at most 63).
	intermediate_textured.Store(pixel, uint4(
		(hit.normal & ~0x1e0u) | ((shadow_counts & 0xfu) << 5u),
		hit.bvh_item_idx,
		hit.voxel_data_index | (hit.voxel_index << 16u) | ((shadow_counts >> 4u) << 24u),
		asuint(hit.total_dist + beam_min_dist)
	));
}

[shader("compute")]
[numthreads(8, 4, 1)]
void main(uint3 global_invocation_id : SV_DispatchThreadID) {
	raycast_primary(global_invocation_id, false);
}

[shader("compute")]
[numthreads(8, 4, 1)]
void fallback_main(uint3 global_invocation_id : SV_DispatchThreadID) {
	if (render_settings.values.x != 0u && gi_words[2u] != 0u) {
		raycast_primary(global_invocation_id, true);
	}
}
