import shared::beam_combined_raycast::{ beam_raycast };

@group(3) @binding(0) var bvh_beam_textured: texture_storage_2d<r32float, write>;

struct CameraUniform {
	camera_transform: mat4x4<f32>,
	camera_view_size: vec2<f32>,
}
@group(0) @binding(0) var<uniform> camera: CameraUniform;

@compute
@workgroup_size(4, 4)
fn main(@builtin(global_invocation_id) global_invocation_id: vec3<u32>) {
	let texture_size = textureDimensions(bvh_beam_textured);
	let pixel = global_invocation_id.xy;
	if pixel.x >= texture_size.x || pixel.y >= texture_size.y { return; }

	let screen_pos = (vec2<f32>(pixel) + vec2<f32>(0.5)) / vec2<f32>(texture_size);
	let ray_pos = camera.camera_transform[3].xyz;
	let ray_dir = normalize((camera.camera_transform * vec4<f32>(
		(screen_pos.x - 0.5) * camera.camera_view_size.x * 2.0,
		(0.5 - screen_pos.y) * camera.camera_view_size.y * 2.0,
		-1.0,
		0.0,
	)).xyz);

	let angle_step = vec2<f32>(
		camera.camera_view_size.x * 2.0 / f32(texture_size.x),
		camera.camera_view_size.y * 2.0 / f32(texture_size.y),
	);
	let expand_k = 0.5 * length(angle_step);
	let min_dist = beam_raycast(ray_pos, ray_dir, expand_k);

	textureStore(bvh_beam_textured, vec2<i32>(pixel), vec4<f32>(min_dist, 0.0, 0.0, 0.0));
}
