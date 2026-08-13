struct ViewUniform {
	float4x4 clip_from_world;
};
ConstantBuffer<ViewUniform> view : register(b0, space0);

struct ModelUniform {
	float4x4 model;
};
StructuredBuffer<ModelUniform> models : register(t0, space1);

struct Triangle {
	uint packed_origin_xy;
	uint packed_origin_z_edges;
	uint color;
};
StructuredBuffer<Triangle> triangles : register(t0, space2);

struct VertexOutput {
	float4 clip_position : SV_Position;
	float4 color : TEXCOORD0;
	float3 world_normal : TEXCOORD1;
};

static const float3 MC33_VERTICES[13] = {
	float3(0.5, 0.0, 0.0), float3(1.0, 0.5, 0.0),
	float3(0.5, 1.0, 0.0), float3(0.0, 0.5, 0.0),
	float3(0.5, 0.0, 1.0), float3(1.0, 0.5, 1.0),
	float3(0.5, 1.0, 1.0), float3(0.0, 0.5, 1.0),
	float3(0.0, 0.0, 0.5), float3(1.0, 0.0, 0.5),
	float3(1.0, 1.0, 0.5), float3(0.0, 1.0, 0.5),
	float3(0.5, 0.5, 0.5),
};

float3 triangle_position(Triangle triangle, uint corner) {
	let origin = float3(
		triangle.packed_origin_xy & 0xffff,
		triangle.packed_origin_xy >> 16,
		triangle.packed_origin_z_edges & 0xffff
	);
	let packed_vertices = triangle.packed_origin_z_edges >> 16;
	let vertex = (packed_vertices >> (corner * 4)) & 0xf;
	return origin + MC33_VERTICES[vertex] + float3(0.5);
}

[shader("vertex")]
VertexOutput vs_main(uint vertex_index : SV_VertexID, uint instance_index : SV_InstanceID) {
	let triangle = triangles[vertex_index / 3];
	let p0 = triangle_position(triangle, 0);
	let p1 = triangle_position(triangle, 1);
	let p2 = triangle_position(triangle, 2);
	let corner = vertex_index % 3;
	let position = corner == 0 ? p0 : (corner == 1 ? p1 : p2);
	let normal = normalize(cross(p1 - p0, p2 - p0));
	let model = models[instance_index].model;
	let world = mul(model, float4(position, 1.0));
	VertexOutput out;
	out.clip_position = mul(view.clip_from_world, world);
	out.world_normal = normalize(mul(model, float4(normal, 0.0)).xyz);
	out.color = unpackUnorm4x8ToFloat(triangle.color);
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
