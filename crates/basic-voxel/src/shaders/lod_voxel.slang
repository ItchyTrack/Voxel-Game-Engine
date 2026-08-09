#pragma once

#include "../../../voxel-ray-renderer/src/shaders/shared/voxel_sampler_api.slang"

struct LodVoxelSampler : IVoxelSampler {
	static const uint RAW_SIZE_BYTES = 32u;
	static const uint PALETTE_NO_PALETTE = 255u;

	uint color_offset(uint node_data_offset, uint item_index, uint octant) {
		return node_data_offset + item_index * RAW_SIZE_BYTES + octant * 4u;
	}

	uint read_palette_index(uint data_source, uint node_data_offset, uint logical_index, uint bits) {
		if (bits == 1u || bits == 2u || bits == 4u) {
			let bit_offset = logical_index * bits;
			let packed = voxel_sampler_u8(data_source, node_data_offset + bit_offset / 8u);
			return (packed >> (bit_offset % 8u)) & ((1u << bits) - 1u);
		}
		if (bits == 8u) {
			return voxel_sampler_u8(data_source, node_data_offset + logical_index);
		}
		return voxel_sampler_u8(data_source, node_data_offset + logical_index * 2u)
			| (voxel_sampler_u8(data_source, node_data_offset + logical_index * 2u + 1u) << 8u);
	}

	bool is_null_color(uint data_source, uint raw_offset) {
		return voxel_sampler_u8(data_source, raw_offset + 0u) == 0u
			&& voxel_sampler_u8(data_source, raw_offset + 1u) == 0u
			&& voxel_sampler_u8(data_source, raw_offset + 2u) == 0u
			&& voxel_sampler_u8(data_source, raw_offset + 3u) == 0u;
	}

	float3 octant_center(uint octant) {
		return float3(
			(octant & 1u) != 0u ? 0.75 : 0.25,
			(octant & 2u) != 0u ? 0.75 : 0.25,
			(octant & 4u) != 0u ? 0.75 : 0.25
		);
	}

	float4 read_color(uint data_source, uint raw_offset) {
		return float4(
			float(voxel_sampler_u8(data_source, raw_offset + 0u)) / 255.0,
			float(voxel_sampler_u8(data_source, raw_offset + 1u)) / 255.0,
			float(voxel_sampler_u8(data_source, raw_offset + 2u)) / 255.0,
			float(voxel_sampler_u8(data_source, raw_offset + 3u)) / 255.0
		);
	}

	uint palette_color_offset(uint data_header_offset, uint palette_index) {
		return data_header_offset + 1u + palette_index * 4u;
	}

	bool palette_is_null_color(uint data_source, uint data_header_offset, uint palette_index) {
		return is_null_color(data_source, palette_color_offset(data_header_offset, palette_index));
	}

	float4 palette_color(uint data_source, uint data_header_offset, uint palette_index) {
		return read_color(data_source, palette_color_offset(data_header_offset, palette_index));
	}

	float4 raw_color(uint data_source, uint node_data_offset, uint item_index, float3 surface_pos, uint octant) {
		let selected_offset = color_offset(node_data_offset, item_index, octant);
		if (!is_null_color(data_source, selected_offset)) {
			return read_color(data_source, selected_offset);
		}

		uint best_octant = 8u;
		float best_distance = 1e38;
		for (uint candidate = 0u; candidate < 8u; candidate++) {
			let candidate_offset = color_offset(node_data_offset, item_index, candidate);
			if (is_null_color(data_source, candidate_offset)) { continue; }
			let delta = surface_pos - octant_center(candidate);
			let distance = dot(delta, delta);
			if (distance < best_distance) {
				best_distance = distance;
				best_octant = candidate;
			}
		}

		if (best_octant == 8u) {
			return float4(0.0);
		}
		return read_color(data_source, color_offset(node_data_offset, item_index, best_octant));
	}

	float4 color(uint data_source, uint data_header_offset, uint node_data_offset, uint item_index, float3 surface_pos) {
		let octant = (surface_pos.x >= 0.5 ? 1u : 0u)
			+ (surface_pos.y >= 0.5 ? 2u : 0u)
			+ (surface_pos.z >= 0.5 ? 4u : 0u);

		let bits = voxel_sampler_u8(data_source, data_header_offset);
		if (bits == PALETTE_NO_PALETTE) {
			return raw_color(data_source, node_data_offset, item_index, surface_pos, octant);
		}

		let logical_index = item_index * 8u + octant;
		let palette_index = bits == 0u ? 0u : read_palette_index(data_source, node_data_offset, logical_index, bits);
		if (!palette_is_null_color(data_source, data_header_offset, palette_index)) {
			return palette_color(data_source, data_header_offset, palette_index);
		}

		uint best_index = 0u;
		bool found = false;
		float best_distance = 1e38;
		for (uint candidate = 0u; candidate < 8u; candidate++) {
			let candidate_index = bits == 0u ? 0u : read_palette_index(data_source, node_data_offset, item_index * 8u + candidate, bits);
			if (palette_is_null_color(data_source, data_header_offset, candidate_index)) { continue; }
			let delta = surface_pos - octant_center(candidate);
			let distance = dot(delta, delta);
			if (distance < best_distance) {
				best_distance = distance;
				best_index = candidate_index;
				found = true;
			}
		}

		if (!found) {
			return float4(0.0);
		}
		return palette_color(data_source, data_header_offset, best_index);
	}
};
