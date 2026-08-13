#include "beam_combined_raycast.slang"

WTexture2D<float> bvh_beam_textured : register(u0, space3);
Texture2D<float> scene_depth : register(t1, space3);

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

float depth_ray_length(uint2 pixel, uint2 texture_size) {
	let depth = scene_depth.Load(int3(int2(pixel), 0)).r;
	if (depth <= 0.0) { return 1e38; }
	let screen_pos = (float2(pixel) + float2(0.5)) / float2(texture_size);
	let clip = float4(screen_pos.x * 2.0 - 1.0, 1.0 - screen_pos.y * 2.0, depth, 1.0);
	let world_h = mul(view.world_from_clip, clip);
	let world = world_h.xyz / world_h.w;
	return length(world - view.world_position);
}

[shader("compute")]
[numthreads(4, 4, 1)]
void main(uint3 global_invocation_id : SV_DispatchThreadID) {
	uint texture_width;
	uint texture_height;
	bvh_beam_textured.GetDimensions(texture_width, texture_height);
	uint2 texture_size = uint2(texture_width, texture_height);
	let pixel = global_invocation_id.xy;
	if (pixel.x >= texture_size.x || pixel.y >= texture_size.y) { return; }

	let screen_pos = (float2(pixel) + float2(0.5)) / float2(texture_size);
	let ray_pos = view.world_position;
	let clip = float4(screen_pos.x * 2.0 - 1.0, 1.0 - screen_pos.y * 2.0, 1.0, 1.0);
	let ray_view_h = mul(view.view_from_clip, clip);
	let ray_view = ray_view_h.xyz / ray_view_h.w;
	let ray_dir = normalize(mul(view.world_from_view, float4(ray_view, 0.0)).xyz);
	let camera_view_size = float2(view.view_from_clip[0][0], view.view_from_clip[1][1]);
	let angle_step = float2(
		camera_view_size.x * 2.0 / float(texture_size.x),
		camera_view_size.y * 2.0 / float(texture_size.y),
	);
	let expand_k = 0.5 * length(angle_step);

	uint depth_width;
	uint depth_height;
	scene_depth.GetDimensions(depth_width, depth_height);
	let depth_size = uint2(depth_width, depth_height);
	let depth_min = pixel * depth_size / texture_size;
	let depth_end = min((pixel + uint2(1)) * depth_size / texture_size, depth_size);
	var max_dist = 0.0;
	for (uint y = depth_min.y; y < depth_end.y; y++) {
		for (uint x = depth_min.x; x < depth_end.x; x++) {
			max_dist = max(max_dist, depth_ray_length(uint2(x, y), depth_size));
		}
	}
	let min_dist = beam_raycast(ray_pos, ray_dir, expand_k, max_dist);

	bvh_beam_textured.Store(pixel, min_dist);
}
