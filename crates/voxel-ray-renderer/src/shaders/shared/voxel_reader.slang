#pragma once

#include "voxel_sampler_api.slang"

StructuredBuffer<uint> residency_voxel_data_buf : register(t0, space2);
StructuredBuffer<uint> main_voxel_data_buf : register(t1, space2);

static const uint VOXELS_NODE_OFFSET_UNIT_BYTES = 16u;
static const uint VOXEL_DATA_SOURCE_RESIDENCY = 1u;
static const uint VOXEL_DATA_SOURCE_MAIN = 2u;

uint voxel_sampler_u32(uint data_source, uint byte_off) {
	if (data_source == VOXEL_DATA_SOURCE_MAIN) {
		return main_voxel_data_buf[byte_off / 4u];
	}
	return residency_voxel_data_buf[byte_off / 4u];
}

uint voxel_sampler_u8(uint data_source, uint byte_off) {
	return (voxel_sampler_u32(data_source, byte_off) >> ((byte_off % 4u) * 8u)) & 0xFFu;
}

uint voxel_reader_buffer_offset(uint voxels_base, uint voxel_data_index) {
	return voxels_base + voxel_data_index * VOXELS_NODE_OFFSET_UNIT_BYTES;
}

float4 voxel_sampler_color(uint voxel_type_id, uint data_source, uint voxels_base, uint voxel_data_index, uint voxel_index, float3 surface_pos) {
	let data_header_offset = voxels_base;
	let node_data_offset = voxel_reader_buffer_offset(voxels_base, voxel_data_index);
	IVoxelSampler sampler = createDynamicObject<IVoxelSampler>(voxel_type_id);
	return sampler.color(data_source, data_header_offset, node_data_offset, voxel_index, surface_pos);
}

float4 residency_voxel_sampler_color(uint voxel_type_id, uint voxels_base, uint voxel_data_index, uint voxel_index, float3 surface_pos) {
	return voxel_sampler_color(voxel_type_id, VOXEL_DATA_SOURCE_RESIDENCY, voxels_base, voxel_data_index, voxel_index, surface_pos);
}

float4 main_voxel_sampler_color(uint voxel_type_id, uint voxels_base, uint voxel_data_index, uint voxel_index, float3 surface_pos) {
	return voxel_sampler_color(voxel_type_id, VOXEL_DATA_SOURCE_MAIN, voxels_base, voxel_data_index, voxel_index, surface_pos);
}
