#pragma once

uint voxel_sampler_u8(uint data_source, uint byte_off);

interface IVoxelSampler {
	float4 color(uint data_source, uint data_header_offset, uint node_data_offset, uint item_index, float3 surface_pos);
}
