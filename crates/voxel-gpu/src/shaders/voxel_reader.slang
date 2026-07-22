#pragma once
StructuredBuffer<uint> residency_voxel_data_buf : register(t0, space2);
StructuredBuffer<uint> main_voxel_data_buf : register(t1, space2);

static const uint VOXELS_NODE_OFFSET_UNIT_BYTES = 16u;
static const uint VOXELS_COLOR_BYTES = 4u;

uint voxel_reader_u8(StructuredBuffer<uint> buffer, uint byte_off) {
	return (buffer[byte_off / 4u] >> ((byte_off % 4u) * 8u)) & 0xFFu;
}

float4 voxel_reader_color(StructuredBuffer<uint> buffer, uint voxels_base, uint voxel_data_index, uint voxel_index) {
	let off = voxels_base + voxel_data_index * VOXELS_NODE_OFFSET_UNIT_BYTES + voxel_index * VOXELS_COLOR_BYTES;
	return float4(
		float(voxel_reader_u8(buffer, off + 0u)) / 255.0,
		float(voxel_reader_u8(buffer, off + 1u)) / 255.0,
		float(voxel_reader_u8(buffer, off + 2u)) / 255.0,
		float(voxel_reader_u8(buffer, off + 3u)) / 255.0
	);
}

float4 residency_voxel_reader_color(uint voxels_base, uint voxel_data_index, uint voxel_index) {
	return voxel_reader_color(residency_voxel_data_buf, voxels_base, voxel_data_index, voxel_index);
}

float4 main_voxel_reader_color(uint voxels_base, uint voxel_data_index, uint voxel_index) {
	return voxel_reader_color(main_voxel_data_buf, voxels_base, voxel_data_index, voxel_index);
}
