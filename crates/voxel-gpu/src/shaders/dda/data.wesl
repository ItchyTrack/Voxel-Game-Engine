const DDA_LOG_SIZE:   u32 = 2u;
const DDA_SIZE:       u32 = 4u;
const DDA_SLOT_BYTES: u32 = 4u;

@group(2) @binding(0) var<storage, read> grid_tree_buf: array<u32>;

fn dda_u8(byte_offset: u32) -> u32 {
	return (grid_tree_buf[byte_offset / 4u] >> ((byte_offset % 4u) * 8u)) & 0xFFu;
}
fn dda_u16(byte_offset: u32) -> u32 {
	return (grid_tree_buf[byte_offset / 4u] >> ((byte_offset % 4u) * 8u)) & 0xFFFFu;
}
fn dda_u32(byte_offset: u32) -> u32 {
	return grid_tree_buf[byte_offset / 4u];
}
fn dda_root_header(tree_base: u32) -> u32 {
	return dda_u8(tree_base);
}
fn dda_nodes_base(tree_base: u32) -> u32 {
	return tree_base + 4u;
}
fn dda_node_byte_off(nodes_base: u32, slot_index: u32) -> u32 {
	return nodes_base + slot_index * DDA_SLOT_BYTES;
}
fn dda_parent_offset(node_byte_offset: u32) -> u32 {
	return dda_u16(node_byte_offset);
}
fn dda_voxel_data_offset(node_byte_offset: u32) -> u32 {
	return dda_u16(node_byte_offset + 2u);
}
fn dda_bitmap_has(low_bits: u32, high_bits: u32, bitmap_status: u32, cell_index: u32) -> bool {
	if cell_index < 32u {
		if (bitmap_status & 1) == 0 { return false; }
		return (low_bits & (1u << cell_index)) != 0u;
	} else {
		if (bitmap_status & 2) == 0 { return false; }
		return (high_bits & (1u << (cell_index - 32u))) != 0u;
	}
}
fn dda_data_index(low_bits: u32, high_bits: u32, bitmap_status: u32, cell_index: u32) -> u32 {
	let mask = (1u << (cell_index & 31u)) - 1u;
	if cell_index >= 32u {
		return (bitmap_status & 1) * countOneBits(low_bits) + (bitmap_status >> 1) * countOneBits(high_bits & mask);
	} else {
		return (bitmap_status & 1) * countOneBits(low_bits & mask);
	}
}
fn dda_node_entry(node_byte_offset: u32, bitmap_status: u32, data_index: u32, node_size: u32) -> u32 {
	let byte_start = node_byte_offset + 8u + 4u * u32(bitmap_status == 3);
	if node_size == 16u {
		return dda_u8(byte_start + data_index);
	}
	return dda_u16(byte_start + data_index * 2u);
}
fn dda_node_size(depth: u32) -> u32  { return 1u << (DDA_LOG_SIZE * (depth + 1u)); }
fn dda_get_child_contents_index(contents_pos: vec3<u32>) -> u32 {
	return contents_pos.x + contents_pos.y * DDA_SIZE + contents_pos.z * DDA_SIZE * DDA_SIZE;
}

struct DDAHit {
	normal: u32,
	voxel_data_index: u32,
	voxel_index: u32,
	dist: f32,
}
