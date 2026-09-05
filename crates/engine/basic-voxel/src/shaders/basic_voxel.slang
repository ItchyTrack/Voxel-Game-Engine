#pragma once

#include "voxel_sampler_api.slang"

struct BasicVoxelSampler : IVoxelSampler {
	static const uint RAW_SIZE_BYTES = 4u;
	static const uint PALETTE_NO_PALETTE = 255u;

	uint read_palette_index(uint data_source, uint node_data_offset, uint item_index, uint bits) {
		if (bits == 1u || bits == 2u || bits == 4u) {
			let bit_offset = item_index * bits;
			let packed = voxel_sampler_u8(data_source, node_data_offset + bit_offset / 8u);
			return (packed >> (bit_offset % 8u)) & ((1u << bits) - 1u);
		}
		if (bits == 8u) {
			return voxel_sampler_u8(data_source, node_data_offset + item_index);
		}
		return voxel_sampler_u8(data_source, node_data_offset + item_index * 2u)
			| (voxel_sampler_u8(data_source, node_data_offset + item_index * 2u + 1u) << 8u);
	}

	float4 read_color(uint data_source, uint raw_offset) {
		return float4(
			float(voxel_sampler_u8(data_source, raw_offset + 0u)) / 255.0,
			float(voxel_sampler_u8(data_source, raw_offset + 1u)) / 255.0,
			float(voxel_sampler_u8(data_source, raw_offset + 2u)) / 255.0,
			float(voxel_sampler_u8(data_source, raw_offset + 3u)) / 255.0
		);
	}

	float4 palette_color(uint data_source, uint data_header_offset, uint palette_index) {
		return read_color(data_source, data_header_offset + 1u + palette_index * 4u);
	}

	float4 color(uint data_source, uint data_header_offset, uint node_data_offset, uint item_index, float3 surface_pos) {
		let bits = voxel_sampler_u8(data_source, data_header_offset);
		if (bits == PALETTE_NO_PALETTE) {
			let raw_offset = node_data_offset + item_index * RAW_SIZE_BYTES;
			return read_color(data_source, raw_offset);
		}

		let palette_index = bits == 0u ? 0u : read_palette_index(data_source, node_data_offset, item_index, bits);
		return palette_color(data_source, data_header_offset, palette_index);
	}
};
