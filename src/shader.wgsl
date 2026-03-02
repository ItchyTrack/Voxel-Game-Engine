// Vertex shader
struct CameraUniform {
	view_proj: mat4x4<f32>,
};
@group(0) @binding(0) // 0.
var<uniform> camera: CameraUniform;

struct Matrix {
	matrix: mat4x4<f32>,
};
@group(1) @binding(0)
var<uniform> matrix: Matrix;

struct VertexInput {
	@location(0) position: vec3<f32>,
	@location(1) color: vec4<f32>,
}

struct VertexOutput {
	@builtin(position) clip_position: vec4<f32>,
	@location(0) color: vec4<f32>,
	@location(1) world_position: vec3<f32>,
}

@vertex
fn vs_main(
	vertex: VertexInput,
) -> VertexOutput {
	var out: VertexOutput;
	out.color = vertex.color;
	let world_pos = matrix.matrix * vec4<f32>(vertex.position, 1.0);
	out.world_position = world_pos.xyz;
	out.clip_position = camera.view_proj * world_pos;
	return out;
}

// Fragment shader
@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
	let normal = normalize(cross(dpdx(in.world_position), dpdy(in.world_position)));

	let light_dir = normalize(vec3<f32>(0.5, 1.0, 0.3));
	let ambient = 0.3;
	let diffuse = max(dot(normal, light_dir), 0.0);
	let lighting = ambient + (1.0 - ambient) * diffuse;

	return vec4<f32>(in.color.rgb * lighting, in.color.a);
}
