@group(1) @binding(2) var<storage, read_write> bvh_item_direction_masks: array<atomic<u32>>;

// Call to make sure the bvh item internals are loaded.
fn record_bvh_item_directions(item_index: u32, local_dir: vec3<f32>) {
	let mask = select(1u << 0u, 1u << 1u, local_dir.x >= 0.0)
		| select(1u << 2u, 1u << 3u, local_dir.y >= 0.0)
		| select(1u << 4u, 1u << 5u, local_dir.z >= 0.0);
	let word_index = item_index >> 2u;
	let shift = (item_index & 3u) * 8u;
	atomicOr(&bvh_item_direction_masks[word_index], mask << shift);
}
