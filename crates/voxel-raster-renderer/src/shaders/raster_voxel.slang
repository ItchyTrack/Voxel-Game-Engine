struct CameraUniform {
	float4x4 view_proj;
};
ConstantBuffer<CameraUniform> camera : register(b0, space0);

struct ModelUniform {
	float4x4 model;
	uint palette_offset;
	uint _padding0;
	uint _padding1;
	uint _padding2;
};
ConstantBuffer<ModelUniform> model : register(b0, space1);

struct Face {
	uint packed;
};
StructuredBuffer<Face> faces : register(t0, space2);
StructuredBuffer<uint> palette : register(t1, space2);

float4 unpack4x8unorm_slang(uint value) {
	return float4(
		float((value >> 0u) & 0xFFu) / 255.0,
		float((value >> 8u) & 0xFFu) / 255.0,
		float((value >> 16u) & 0xFFu) / 255.0,
		float((value >> 24u) & 0xFFu) / 255.0
	);
}

struct VertexOutput {
	float4 clip_position : SV_Position;
	float4 color : TEXCOORD0;
	float3 world_position : TEXCOORD1;
	float3 world_normal : TEXCOORD2;
};

[shader("vertex")]
VertexOutput vs_main(uint vertex_index : SV_VertexID) {
	let offsets_2d = float2[6](
		float2( 0.5,  0.5),
		float2(-0.5,  0.5),
		float2( 0.5, -0.5),
		float2( 0.5, -0.5),
		float2(-0.5,  0.5),
		float2(-0.5, -0.5),
	);

	let face = faces[vertex_index / 6u];
	let base = offsets_2d[vertex_index % 6u];
	let packed = face.packed;
	let orientation = (packed >> 18u) & 0x7u;
	let size_log4 = (packed >> 21u) & 0x3u;
	let size = float(1u << (size_log4 * 2u));
	let palette_index = (packed >> 23u) & 0xFFu;
	let position = float3(
		float(packed & 0x3Fu),
		float((packed >> 6u) & 0x3Fu),
		float((packed >> 12u) & 0x3Fu),
	);
	float3 offset;
	float3 local_normal;
	switch(orientation) {
		case 0u: {
			offset = float3(1.0,             base.x + 0.5,  base.y + 0.5);
			local_normal = float3(1.0, 0.0, 0.0);
			break;
		}
		case 1u: {
			offset = float3(0.0,            -base.x + 0.5,  base.y + 0.5);
			local_normal = float3(-1.0, 0.0, 0.0);
			break;
		}
		case 2u: {
			offset = float3(base.x + 0.5,    1.0,           -base.y + 0.5);
			local_normal = float3(0.0, 1.0, 0.0);
			break;
		}
		case 3u: {
			offset = float3(base.x + 0.5,    0.0,            base.y + 0.5);
			local_normal = float3(0.0, -1.0, 0.0);
			break;
		}
		case 4u: {
			offset = float3(base.x + 0.5,    base.y + 0.5,   1.0);
			local_normal = float3(0.0, 0.0, 1.0);
			break;
		}
		default: {
			offset = float3(-base.x + 0.5,   base.y + 0.5,   0.0);
			local_normal = float3(0.0, 0.0, -1.0);
			break;
		}
	}

	VertexOutput out;
	out.color = unpack4x8unorm_slang(palette[model.palette_offset + palette_index]);
	let local = position + offset * size;
	let world = mul(model.model, float4(local, 1.0));
	out.world_position = world.xyz;
	out.world_normal = normalize(mul(model.model, float4(local_normal, 0.0)).xyz);
	out.clip_position = mul(camera.view_proj, world);
	return out;
}

[shader("fragment")]
float4 fs_main(VertexOutput input) : SV_Target {
	let normal = normalize(input.world_normal);
	let light_dir = normalize(float3(0.5, 1.0, 0.2));
	let ambient = 0.3;
	let diffuse = max(dot(normal, light_dir), 0.0);
	let lighting = ambient + (1.0 - ambient) * diffuse;
	return float4(input.color.rgb * lighting, input.color.a);
}
