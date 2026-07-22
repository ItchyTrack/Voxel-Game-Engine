#include "beam_combined_raycast.slang"

WTexture2D<float> bvh_beam_textured : register(u0, space3);

struct CameraUniform {
	float4 camera_transform[4];
	float2 camera_view_size;
};
ConstantBuffer<CameraUniform> camera : register(b0, space0);

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
	let ray_pos = camera.camera_transform[3].xyz;
	let view_ray = float4(
		(screen_pos.x - 0.5) * camera.camera_view_size.x * 2.0,
		(0.5 - screen_pos.y) * camera.camera_view_size.y * 2.0,
		-1.0,
		0.0
	);
	let ray_dir = normalize((
		camera.camera_transform[0] * view_ray.x
		+ camera.camera_transform[1] * view_ray.y
		+ camera.camera_transform[2] * view_ray.z
		+ camera.camera_transform[3] * view_ray.w
	).xyz);

	let angle_step = float2(
		camera.camera_view_size.x * 2.0 / float(texture_size.x),
		camera.camera_view_size.y * 2.0 / float(texture_size.y),
	);
	let expand_k = 0.5 * length(angle_step);
	let min_dist = beam_raycast(ray_pos, ray_dir, expand_k);

	bvh_beam_textured.Store(pixel, min_dist);
}
