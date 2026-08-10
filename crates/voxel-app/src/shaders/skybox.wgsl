#import bevy_render::view::View

struct VertexOutput {
	@builtin(position) clip_position: vec4<f32>,
	@location(0) screen_pos: vec2<f32>,
}

@vertex
fn vs_main(@builtin(vertex_index) vertex_index: u32) -> VertexOutput {
	var out: VertexOutput;
	out.screen_pos    = vec2<f32>(f32((vertex_index << 1u) & 2u), f32(vertex_index & 2u));
	out.clip_position = vec4<f32>(out.screen_pos * 2.0 - 1.0, 0.0, 1.0);
	return out;
}

@group(0) @binding(0) var<uniform> view: View;

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
	let clip = vec4<f32>(in.screen_pos * 2.0 - 1.0, 1.0, 1.0);
	let world_h = view.world_from_clip * clip;
	let ray_dir = normalize(world_h.xyz / world_h.w - view.world_position);

	let sun_dir = normalize(vec3<f32>(0.5, 1.0, 0.2));
	let t = ray_dir.y * 0.5 + 0.5;
	let bg = mix(vec3<f32>(0.15, 0.15, 0.18), vec3<f32>(0.05, 0.07, 0.12), t);
	let sun = max(dot(ray_dir, sun_dir), 0.0);
	let sun_color = vec3<f32>(1.0, 0.9, 0.6) * pow(sun, 64.0);
	return vec4<f32>(bg + sun_color, 1.0);
}
