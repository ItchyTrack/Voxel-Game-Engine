#pragma once

struct BasicVoxelSampler : IVoxelSampler {
	static const uint RAW_SIZE_BYTES = 4u;

	float4 color(uint data_source, uint buffer_offset, uint item_index, float3 surface_pos) {
		let raw_offset = buffer_offset + item_index * RAW_SIZE_BYTES;
		return float4(
			float(voxel_sampler_u8(data_source, raw_offset + 0u)) / 255.0,
			float(voxel_sampler_u8(data_source, raw_offset + 1u)) / 255.0,
			float(voxel_sampler_u8(data_source, raw_offset + 2u)) / 255.0,
			float(voxel_sampler_u8(data_source, raw_offset + 3u)) / 255.0
		);
	}
};
