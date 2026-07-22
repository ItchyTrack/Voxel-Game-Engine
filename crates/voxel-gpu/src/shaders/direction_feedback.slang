#pragma once
RWStructuredBuffer<Atomic<uint>> bvh_item_direction_masks : register(u2, space1);

// Call to make sure the bvh item internals are loaded.
void record_bvh_item_directions(uint item_index, float3 local_dir) {
	let mask = ((local_dir.x >= 0.0) ? (1u << 1u) : (1u << 0u))
		| ((local_dir.y >= 0.0) ? (1u << 3u) : (1u << 2u))
		| ((local_dir.z >= 0.0) ? (1u << 5u) : (1u << 4u));
	let word_index = item_index >> 2u;
	let shift = (item_index & 3u) * 8u;
	bvh_item_direction_masks[word_index].or(mask << shift);
}
