@group(0) @binding(0)
var<uniform> screen_size: vec4<f32>;

@vertex
fn vs_main(@builtin(vertex_index) vi: u32) -> @builtin(position) vec4<f32> {
	let x = f32(i32(vi) / 2) * 4.0 - 1.0;
	let y = f32(i32(vi) % 2) * 4.0 - 1.0;
	return vec4<f32>(x, y, 0.0, 1.0);
}

@fragment
fn fs_main(@builtin(position) pos: vec4<f32>) -> @location(0) vec4<f32> {
	let screen_size = screen_size.xy;
	let p = abs(pos.xy - screen_size * 0.5);
	let thickness = 1.0;
	let length = 10.0;
	let gap = 2.0;

	let in_h = p.y <= thickness && p.x >= gap && p.x <= length;
	let in_v = p.x <= thickness && p.y >= gap && p.y <= length;

	if in_h || in_v {
		return vec4<f32>(1.0, 1.0, 1.0, 1.0);
	}
	discard;
}
