struct CameraUniform {
	view_proj: mat4x4<f32>,
};
@group(0) @binding(0)
var<uniform> camera: CameraUniform;

struct ModelUniform {
	model: mat4x4<f32>,
};
@group(1) @binding(0)
var<uniform> model: ModelUniform;

struct Face {
	packed_position_orientation_size: u32,
	color: u32,
};
@group(2) @binding(0)
var<storage, read> faces: array<Face>;

struct VertexOutput {
	@builtin(position) clip_position: vec4<f32>,
	@location(0) color: vec4<f32>,
	@location(1) world_position: vec3<f32>,
	@location(2) world_normal: vec3<f32>,
}

fn unpack_i8(value: u32, shift: u32) -> i32 {
	let byte = (value >> shift) & 0xFFu;
	return select(i32(byte), i32(byte) - 256, byte >= 128u);
}

@vertex
fn vs_main(@builtin(vertex_index) vertex_index: u32) -> VertexOutput {
	let offsets_2d = array<vec2<f32>, 6>(
		vec2( 0.5,  0.5),
		vec2(-0.5,  0.5),
		vec2( 0.5, -0.5),
		vec2( 0.5, -0.5),
		vec2(-0.5,  0.5),
		vec2(-0.5, -0.5),
	);

	let face = faces[vertex_index / 6u];
	let base = offsets_2d[vertex_index % 6u];
	let packed = face.packed_position_orientation_size;
	let orientation_size = (packed >> 24u) & 0xFFu;
	let orientation = orientation_size & 0x7u;
	let size_log2 = orientation_size >> 3u;
	let size = f32(1u << size_log2);
	let position = vec3<f32>(
		f32(unpack_i8(packed, 0u)),
		f32(unpack_i8(packed, 8u)),
		f32(unpack_i8(packed, 16u)),
	);
	var offset: vec3<f32>;
	var local_normal: vec3<f32>;
	switch(orientation) {
		case 0u: {
			offset = vec3(1.0,             base.x + 0.5,  base.y + 0.5);
			local_normal = vec3(1.0, 0.0, 0.0);
		}
		case 1u: {
			offset = vec3(0.0,            -base.x + 0.5,  base.y + 0.5);
			local_normal = vec3(-1.0, 0.0, 0.0);
		}
		case 2u: {
			offset = vec3(base.x + 0.5,    1.0,           -base.y + 0.5);
			local_normal = vec3(0.0, 1.0, 0.0);
		}
		case 3u: {
			offset = vec3(base.x + 0.5,    0.0,            base.y + 0.5);
			local_normal = vec3(0.0, -1.0, 0.0);
		}
		case 4u: {
			offset = vec3(base.x + 0.5,    base.y + 0.5,   1.0);
			local_normal = vec3(0.0, 0.0, 1.0);
		}
		default: {
			offset = vec3(-base.x + 0.5,   base.y + 0.5,   0.0);
			local_normal = vec3(0.0, 0.0, -1.0);
		}
	}

	var out: VertexOutput;
	out.color = unpack4x8unorm(face.color);
	let local = position + offset * size;
	let world = model.model * vec4(local, 1.0);
	out.world_position = world.xyz;
	out.world_normal = normalize((model.model * vec4(local_normal, 0.0)).xyz);
	out.clip_position = camera.view_proj * world;
	return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
	let normal = normalize(in.world_normal);
	let light_dir = normalize(vec3<f32>(0.5, 1.0, 0.2));
	let ambient = 0.3;
	let diffuse = max(dot(normal, light_dir), 0.0);
	let lighting = ambient + (1.0 - ambient) * diffuse;
	return vec4(in.color.rgb * lighting, in.color.a);
}
