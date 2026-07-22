#pragma once

StructuredBuffer<uint> residency_voxel_data_buf : register(t0, space2);
StructuredBuffer<uint> main_voxel_data_buf : register(t1, space2);

static const uint VOXELS_NODE_OFFSET_UNIT_BYTES = 16u;
static const uint VOXELS_COLOR_BYTES = 4u;

interface IVoxelDataSource {
	uint voxel_u32(uint byte_off);
}

uint voxel_reader_u32<T : IVoxelDataSource>(uint byte_off) {
	T data;
	return data.voxel_u32(byte_off);
}

uint voxel_reader_u8<T : IVoxelDataSource>(uint byte_off) {
	return (voxel_reader_u32<T>(byte_off) >> ((byte_off % 4u) * 8u)) & 0xFFu;
}

float4 voxel_reader_color<T : IVoxelDataSource>(uint voxels_base, uint voxel_data_index, uint voxel_index) {
	let off = voxels_base + voxel_data_index * VOXELS_NODE_OFFSET_UNIT_BYTES + voxel_index * VOXELS_COLOR_BYTES;
	return float4(
		float(voxel_reader_u8<T>(off + 0u)) / 255.0,
		float(voxel_reader_u8<T>(off + 1u)) / 255.0,
		float(voxel_reader_u8<T>(off + 2u)) / 255.0,
		float(voxel_reader_u8<T>(off + 3u)) / 255.0
	);
}

struct ResidencyVoxelDataSource : IVoxelDataSource {
	uint voxel_u32(uint byte_off) { return residency_voxel_data_buf[byte_off / 4u]; }
};

struct MainVoxelDataSource : IVoxelDataSource {
	uint voxel_u32(uint byte_off) { return main_voxel_data_buf[byte_off / 4u]; }
};

float4 residency_voxel_reader_color(uint voxels_base, uint voxel_data_index, uint voxel_index) {
	return voxel_reader_color<ResidencyVoxelDataSource>(voxels_base, voxel_data_index, voxel_index);
}

float4 main_voxel_reader_color(uint voxels_base, uint voxel_data_index, uint voxel_index) {
	return voxel_reader_color<MainVoxelDataSource>(voxels_base, voxel_data_index, voxel_index);
}
