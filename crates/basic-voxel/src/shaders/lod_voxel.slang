#pragma once

struct LodVoxelSampler : IVoxelSampler {
	static const uint RAW_SIZE_BYTES = 32u;

	uint color_offset(uint buffer_offset, uint item_index, uint octant) {
		return buffer_offset + item_index * RAW_SIZE_BYTES + octant * 4u;
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

	float4 color(uint data_source, uint buffer_offset, uint item_index, float3 surface_pos) {
		let octant = (surface_pos.x >= 0.5 ? 1u : 0u)
			+ (surface_pos.y >= 0.5 ? 2u : 0u)
			+ (surface_pos.z >= 0.5 ? 4u : 0u);
		let selected_offset = color_offset(buffer_offset, item_index, octant);
		if (!is_null_color(data_source, selected_offset)) {
			return read_color(data_source, selected_offset);
		}

		uint best_octant = 8u;
		float best_distance = 1e38;
		for (uint candidate = 0u; candidate < 8u; candidate++) {
			let candidate_offset = color_offset(buffer_offset, item_index, candidate);
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
		return read_color(data_source, color_offset(buffer_offset, item_index, best_octant));
	}
};
