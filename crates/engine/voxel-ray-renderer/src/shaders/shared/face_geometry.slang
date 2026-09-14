#pragma once

#include "bvh/data.slang"
#include "helpers/quat.slang"

uint gi_canonical_normal(uint packed, uint axis) {
	return (packed & 0xffffc000u) | 1024u | (axis << 3u) | (packed & (1u << axis));
}

struct GiFaceGeometry {
	float3 center;
	float3 normal;
	float3 surface_pos;
	float4 rotation;
	float bias;
};

GiFaceGeometry gi_face_geometry(uint3 face) {
	let item = bvh_items[face.y];
	let coordinate = face.x >> 14u;
	let local_min = float3(coordinate & 63u, (coordinate >> 6u) & 63u, coordinate >> 12u);
	let axis = (face.x >> 3u) & 3u;
	let negative = (face.x & (1u << axis)) != 0u;
	float3 local_normal = float3(0.0);
	local_normal[axis] = negative ? -1.0 : 1.0;
	GiFaceGeometry geometry;
	geometry.surface_pos = float3(0.5);
	geometry.surface_pos[axis] = negative ? 0.0 : 1.0;
	geometry.rotation = float4(item.quat_x, item.quat_y, item.quat_z, item.quat_w);
	geometry.center = float3(item.pos_x, item.pos_y, item.pos_z)
		+ Quat::rotate(geometry.rotation, (local_min + geometry.surface_pos) * item.scale);
	geometry.normal = Quat::rotate(geometry.rotation, local_normal);
	geometry.bias = item.scale * 0.01;
	return geometry;
}
