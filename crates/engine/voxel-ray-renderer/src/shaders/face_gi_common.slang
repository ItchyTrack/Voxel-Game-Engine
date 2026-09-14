#pragma once

#include "bvh/data.slang"
#include "helpers/quat.slang"

struct GiParams {
	uint4 sizes;   // Table capacity, visible capacity, texture width, texture height.
	uint4 offsets; // Word offsets of table, face list, hits, and lighting.
};
ConstantBuffer<GiParams> gi_params : register(b2, space3);
Texture2D<uint4> intermediate_textured_read : register(t0, space3);
#ifdef FACE_GI_COMPUTE
RWStructuredBuffer<Atomic<uint>> gi_words : register(u1, space3);
#else
StructuredBuffer<uint> gi_words : register(t1, space3);
#endif

// Miss records keep this tag in word 1; contributor discovery copies it to word 3.
static const uint GI_UNKNOWN = 0xffffffffu;
static const uint GI_SKY = 0xfffffffeu;
static const uint GI_SAMPLE_COUNT = 8u;
static const uint GI_PROBE_LIMIT = 64u;

uint gi_load(uint offset) {
#ifdef FACE_GI_COMPUTE
	return gi_words[offset].load();
#else
	return gi_words[offset];
#endif
}

uint gi_hash(uint value) {
	value ^= value >> 16u;
	value *= 0x7feb352du;
	value ^= value >> 15u;
	value *= 0x846ca68bu;
	return value ^ (value >> 16u);
}

uint gi_canonical_normal(uint packed, uint axis) {
	return (packed & 0xffffc000u) | 1024u | (axis << 3u) | (packed & (1u << axis));
}

uint gi_primary_reference_count() {
	return gi_params.sizes.z * gi_params.sizes.w * 3u;
}

// Owners point to keys from an earlier dispatch, never to a half-written face.
uint3 gi_reference_face(uint reference) {
	let primary_count = gi_primary_reference_count();
	if (reference < primary_count) {
		let pixel = reference / 3u;
		let data = intermediate_textured_read.Load(int3(int2(pixel % gi_params.sizes.z, pixel / gi_params.sizes.z), 0));
		return uint3(gi_canonical_normal(data.x, reference % 3u), data.y, data.z);
	}
	let offset = gi_params.offsets.z + (reference - primary_count) * 4u;
	return uint3(gi_load(offset), gi_load(offset + 1u), gi_load(offset + 2u));
}

uint gi_face_hash(uint2 key) {
	return gi_hash(key.x ^ gi_hash(key.y));
}

uint gi_find_face(uint2 key) {
	let mask = gi_params.sizes.x - 1u;
	let start = gi_face_hash(key);
	for (uint probe = 0u; probe < GI_PROBE_LIMIT; probe++) {
		let slot = (start + probe) & mask;
		let owner = gi_load(gi_params.offsets.x + slot);
		if (owner == 0u) { break; }
		if (all(gi_reference_face(owner - 1u).xy == key)) { return slot; }
	}
	return GI_UNKNOWN;
}

uint3 gi_list_face(uint index) {
	let slot = gi_load(gi_params.offsets.y + index);
	return gi_reference_face(gi_load(gi_params.offsets.x + slot) - 1u);
}

float3 gi_load_rgb(uint offset) {
	return asfloat(uint3(gi_load(offset), gi_load(offset + 1u), gi_load(offset + 2u)));
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
