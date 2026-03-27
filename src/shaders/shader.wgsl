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
	position_x_y_z_orientation_size: u32,
	color: u32,
}
@group(2) @binding(0)
var<storage, read> vertex_data: array<VertexInput>;

struct VertexOutput {
	@builtin(position) clip_position: vec4<f32>,
	@location(0) color: vec4<f32>,
	@location(1) world_position: vec3<f32>,
}

@vertex
fn vs_main(
	// vertex: VertexInput,
	@builtin(vertex_index) vertex_index : u32,
) -> VertexOutput {
	const offsets_2d = array<vec2<f32>, 6>(
		vec2( 0.5,  0.5),
		vec2(-0.5,  0.5),
		vec2( 0.5, -0.5),
		vec2( 0.5, -0.5),
		vec2(-0.5,  0.5),
		vec2(-0.5, -0.5),
	);

	let vertex = vertex_data[vertex_index / 6u];
	let base = offsets_2d[vertex_index % 6u] * f32(vertex.position_x_y_z_orientation_size >> 27);
	var offset: vec3<f32>;
	switch((vertex.position_x_y_z_orientation_size >> 24) & 7) {
		case 0u: { offset = vec3( 1.0,			 base.x + 0.5,   base.y + 0.5); } // +X
		case 1u: { offset = vec3( 0.0,			-base.x + 0.5,   base.y + 0.5); } // -X
		case 2u: { offset = vec3( base.x + 0.5,  1.0, 			-base.y + 0.5); } // +Y
		case 3u: { offset = vec3( base.x + 0.5,  0.0, 			 base.y + 0.5); } // -Y
		case 4u: { offset = vec3( base.x + 0.5,  base.y + 0.5, 	 1.0		 ); } // +Z
		case 5u: { offset = vec3(-base.x + 0.5,  base.y + 0.5, 	 0.0		 ); } // -Z
		default: { offset = vec3(0.0); }
	}
	var out: VertexOutput;
	out.color = unpack4x8unorm(vertex.color);
	let world_pos = matrix.matrix * vec4<f32>(vec3<f32>(unpack4xI8(vertex.position_x_y_z_orientation_size).xyz) + offset, 1.0);
	out.world_position = world_pos.xyz;
	out.clip_position = camera.view_proj * world_pos;

	return out;
}

// Fragment shader
@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
	let normal = normalize(cross(dpdx(in.world_position), dpdy(in.world_position)));

	let light_dir = normalize(vec3<f32>(0.5, -1.0, -0.3));

	let ambient = 0.3;
	let diffuse = max(dot(normal, light_dir), 0.0);

	let lighting = ambient + (1.0 - ambient) * diffuse;

	return vec4<f32>(in.color.rgb * lighting, in.color.a);
}
