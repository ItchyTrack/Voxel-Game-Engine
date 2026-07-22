#pragma once
static const uint DDA_LOG_SIZE = 2u;
static const uint DDA_SIZE = 4u;
static const uint DDA_SLOT_BYTES = 4u;

StructuredBuffer<uint> residency_grid_tree_buf : register(t0, space2);
StructuredBuffer<uint> main_grid_tree_buf : register(t1, space2);

interface IDDADataSource {
	uint dda_u32(uint byte_offset);
}

struct ResidencyDDADataSource : IDDADataSource {
	uint dda_u32(uint byte_offset) { return residency_grid_tree_buf[byte_offset / 4u]; }
};

struct MainDDADataSource : IDDADataSource {
	uint dda_u32(uint byte_offset) { return main_grid_tree_buf[byte_offset / 4u]; }
};

uint dda_u32<T : IDDADataSource>(uint byte_offset) {
	T data;
	return data.dda_u32(byte_offset);
}

uint dda_u8<T : IDDADataSource>(uint byte_offset) {
	return (dda_u32<T>(byte_offset) >> ((byte_offset % 4u) * 8u)) & 0xFFu;
}

uint dda_u16<T : IDDADataSource>(uint byte_offset) {
	return (dda_u32<T>(byte_offset) >> ((byte_offset % 4u) * 8u)) & 0xFFFFu;
}

uint dda_parent_offset<T : IDDADataSource>(uint node_byte_offset) {
	return dda_u16<T>(node_byte_offset);
}

uint dda_voxel_data_offset<T : IDDADataSource>(uint node_byte_offset) {
	return dda_u16<T>(node_byte_offset + 2u);
}

uint dda_node_entry<T : IDDADataSource>(uint node_byte_offset, uint bitmap_status, uint data_index, uint node_size) {
	uint byte_start = node_byte_offset + 8u + 4u * ((bitmap_status == 3u) ? 1u : 0u);
	if ((node_size == 16u)) { return dda_u8<T>(byte_start + data_index); }
	return dda_u16<T>(byte_start + data_index * 2u);
}

uint dda_nodes_base(uint tree_base) {
	return tree_base + 4u;
}

uint dda_node_byte_off(uint nodes_base, uint slot_index) {
	return nodes_base + slot_index * DDA_SLOT_BYTES;
}

bool dda_bitmap_has(uint low_bits, uint high_bits, uint bitmap_status, uint cell_index) {
	if (cell_index < 32u) {
		if ((bitmap_status & 1u) == 0u) { return false; }
		return (low_bits & (1u << cell_index)) != 0u;
	} else {
		if ((bitmap_status & 2u) == 0u) { return false; }
		return (high_bits & (1u << (cell_index - 32u))) != 0u;
	}
}

uint dda_data_index(uint low_bits, uint high_bits, uint bitmap_status, uint cell_index) {
	uint mask = (1u << (cell_index & 31u)) - 1u;
	if (cell_index >= 32u) {
		return (bitmap_status & 1u) * countbits(low_bits) + (bitmap_status >> 1u) * countbits(high_bits & mask);
	} else {
		return (bitmap_status & 1u) * countbits(low_bits & mask);
	}
}

uint dda_node_size(uint depth) {
	return 1u << (DDA_LOG_SIZE * (depth + 1u));
}

uint dda_get_child_contents_index(uint3 contents_pos) {
	return contents_pos.x + contents_pos.y * DDA_SIZE + contents_pos.z * DDA_SIZE * DDA_SIZE;
}

struct DDAHit {
	uint normal;
	uint voxel_data_index;
	uint voxel_index;
	float dist;
};
