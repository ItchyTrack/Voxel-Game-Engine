#include "voxel_sampler_api.slang"

struct ViewUniform {
	float4x4 clip_from_world;
};
ConstantBuffer<ViewUniform> view : register(b0, space0);

struct ModelUniform {
	float4x4 model;
	uint voxel_type_id;
	uint voxel_data_offset;
	uint _padding0;
	uint _padding1;
};
StructuredBuffer<ModelUniform> models : register(t0, space1);

struct Face {
	uint packed;
	uint voxel_data_index;
};
StructuredBuffer<Face> faces : register(t0, space2);
StructuredBuffer<uint> voxel_data : register(t1, space2);

static const uint VOXEL_DATA_OFFSET_UNIT_BYTES = 16u;

uint voxel_sampler_u8(uint data_source, uint byte_offset) {
	let word = voxel_data[byte_offset / 4u];
	return (word >> ((byte_offset % 4u) * 8u)) & 0xFFu;
}

struct VertexOutput {
	float4 clip_position : SV_Position;
	float3 surface_position : TEXCOORD0;
	float3 world_position : TEXCOORD1;
	float3 world_normal : TEXCOORD2;
	nointerpolation uint voxel_type_id : TEXCOORD3;
	nointerpolation uint voxel_data_offset : TEXCOORD4;
	nointerpolation uint voxel_data_index : TEXCOORD5;
};

[shader("vertex")]
VertexOutput vs_main(uint vertex_index : SV_VertexID, uint instance_index : SV_InstanceID) {
	let offsets_2d = float2[6](
		float2( 0.5,  0.5),
		float2(-0.5,  0.5),
		float2( 0.5, -0.5),
		float2( 0.5, -0.5),
		float2(-0.5,  0.5),
		float2(-0.5, -0.5),
	);

	let model = models[instance_index];
	let face = faces[vertex_index / 6u];
	let base = offsets_2d[vertex_index % 6u];
	let packed = face.packed;
	let orientation = (packed >> 18u) & 0x7u;
	let size_log4 = (packed >> 21u) & 0x3u;
	let size = float(1u << (size_log4 * 2u));
	let position = float3(
		float(packed & 0x3Fu),
		float((packed >> 6u) & 0x3Fu),
		float((packed >> 12u) & 0x3Fu),
	);
	float3 offset;
	float3 local_normal;
	switch(orientation) {
		case 0u: {
			offset = float3(1.0,           base.x + 0.5,  base.y + 0.5);
			local_normal = float3(1.0, 0.0, 0.0);
			break;
		}
		case 1u: {
			offset = float3(0.0,           -base.x + 0.5, base.y + 0.5);
			local_normal = float3(-1.0, 0.0, 0.0);
			break;
		}
		case 2u: {
			offset = float3(base.x + 0.5,  1.0,           -base.y + 0.5);
			local_normal = float3(0.0, 1.0, 0.0);
			break;
		}
		case 3u: {
			offset = float3(base.x + 0.5,  0.0,            base.y + 0.5);
			local_normal = float3(0.0, -1.0, 0.0);
			break;
		}
		case 4u: {
			offset = float3(base.x + 0.5,  base.y + 0.5,   1.0);
			local_normal = float3(0.0, 0.0, 1.0);
			break;
		}
		default: {
			offset = float3(-base.x + 0.5, base.y + 0.5,   0.0);
			local_normal = float3(0.0, 0.0, -1.0);
			break;
		}
	}

	VertexOutput out;
	let local = position + offset * size;
	let world = mul(model.model, float4(local, 1.0));
	out.surface_position = offset;
	out.world_position = world.xyz;
	out.world_normal = normalize(mul(model.model, float4(local_normal, 0.0)).xyz);
	out.voxel_type_id = model.voxel_type_id;
	out.voxel_data_offset = model.voxel_data_offset;
	out.voxel_data_index = face.voxel_data_index;
	out.clip_position = mul(view.clip_from_world, world);
	return out;
}

[shader("fragment")]
float4 fs_main(VertexOutput input) : SV_Target {
	let node_data_offset = input.voxel_data_offset
		+ input.voxel_data_index * VOXEL_DATA_OFFSET_UNIT_BYTES;
	IVoxelSampler sampler = createDynamicObject<IVoxelSampler>(input.voxel_type_id);
	let voxel_color = sampler.color(
		0u,
		input.voxel_data_offset,
		node_data_offset,
		0u,
		input.surface_position,
	);

	let normal = normalize(input.world_normal);
	let light_dir = normalize(float3(0.5, 1.0, 0.2));
	let ambient = 0.25;
	let diffuse = max(dot(normal, light_dir), 0.0);
	let lighting = ambient + (1.0 - ambient) * diffuse;
	return float4(voxel_color.rgb * lighting, voxel_color.a);
}
