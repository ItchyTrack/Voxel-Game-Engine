#include "bvh/data.slang"
#include "combined_raycast.slang"
#include "occlusion_raycast.slang"

// 0 : camera uniform
// 1 : BVH (nodes + items)
// 2 : grid tree buffer
// 3 : ray marching textures

WTexture2D<uint4> intermediate_textured : register(u0, space3);
Texture2D<float> bvh_beam_textured_read : register(t1, space3);
Texture2D<float> scene_depth : register(t2, space3);

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

[shader("compute")]
[numthreads(8, 4, 1)]
void main(uint3 global_invocation_id : SV_DispatchThreadID) {
	uint texture_width;
	uint texture_height;
	intermediate_textured.GetDimensions(texture_width, texture_height);
	uint2 texture_size = uint2(texture_width, texture_height);
	let pixel = global_invocation_id.xy;
	if (pixel.x >= texture_size.x || pixel.y >= texture_size.y) { return; }

	let screen_pos = float2(pixel) / float2(texture_size - uint2(1));

	let ray_start = view.world_position;
	let ray_clip = float4(screen_pos.x * 2.0 - 1.0, 1.0 - screen_pos.y * 2.0, 1.0, 1.0);
	let ray_view_h = mul(view.view_from_clip, ray_clip);
	let ray_view = ray_view_h.xyz / ray_view_h.w;
	let ray_dir = normalize(mul(view.world_from_view, float4(ray_view, 0.0)).xyz);

	uint depth_width;
	uint depth_height;
	scene_depth.GetDimensions(depth_width, depth_height);
	let depth_size = uint2(depth_width, depth_height);
	let depth_min = max(int2(pixel) - int2(1), int2(0));
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
	let sun_dir = normalize(float3(0.5, 1.0, 0.2));

	if (hit.normal == 0u) {
		intermediate_textured.Store(pixel, uint4(0, 0, 0, 0));
		return;
	}

	bool light_visible = true;
	if (render_settings.values.x != 0u) {
		let item = bvh_items[hit.bvh_item_idx];
		let hit_pos = ray_start + (hit.total_dist + beam_min_dist) * ray_dir;
		let shadow_bias = max(0.01, item.scale * 0.01);
		light_visible = !occlusion_raycast(hit_pos + sun_dir * shadow_bias, sun_dir, 1e38);
	}
	intermediate_textured.Store(pixel, uint4(
		hit.normal + 256u * (light_visible ? 1u : 0u) + (hit.sample_octant << 11u),
		hit.bvh_item_idx,
		hit.voxel_data_index + (hit.voxel_index << 16u),
		asuint(hit.total_dist + beam_min_dist)
	));
}
