struct CameraUniform {
	float4x4 view_proj;
};
ConstantBuffer<CameraUniform> camera : register(b0, space0);

struct ModelUniform {
	float4x4 model;
};
StructuredBuffer<ModelUniform> models : register(t0, space1);

struct Vertex {
	float3 position;
	float _position_padding;
	float3 normal;
	uint color;
};
StructuredBuffer<Vertex> vertices : register(t0, space2);

struct VertexOutput {
	float4 clip_position : SV_Position;
	float4 color : TEXCOORD0;
	float3 world_normal : TEXCOORD1;
};

[shader("vertex")]
VertexOutput vs_main(uint vertex_index : SV_VertexID, uint instance_index : SV_InstanceID) {
	let vertex = vertices[vertex_index];
	let model = models[instance_index].model;
	let world = mul(model, float4(vertex.position, 1.0));
	VertexOutput out;
	out.clip_position = mul(camera.view_proj, world);
	out.world_normal = normalize(mul(model, float4(vertex.normal, 0.0)).xyz);
	out.color = unpackUnorm4x8ToFloat(vertex.color);
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
