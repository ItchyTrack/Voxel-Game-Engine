import package::dda::data::{ DDA_SLOT_BYTES };

@group(2) @binding(__BINDING__) var<storage, read> __BUFFER__: array<u32>;

fn __PREFIX___dda_u8(byte_offset: u32) -> u32 {
	return (__BUFFER__[byte_offset / 4u] >> ((byte_offset % 4u) * 8u)) & 0xFFu;
}
fn __PREFIX___dda_u16(byte_offset: u32) -> u32 {
	return (__BUFFER__[byte_offset / 4u] >> ((byte_offset % 4u) * 8u)) & 0xFFFFu;
}
fn __PREFIX___dda_u32(byte_offset: u32) -> u32 {
	return __BUFFER__[byte_offset / 4u];
}
fn __PREFIX___dda_parent_offset(node_byte_offset: u32) -> u32 {
	return __PREFIX___dda_u16(node_byte_offset);
}
fn __PREFIX___dda_voxel_data_offset(node_byte_offset: u32) -> u32 {
	return __PREFIX___dda_u16(node_byte_offset + 2u);
}
fn __PREFIX___dda_node_entry(node_byte_offset: u32, bitmap_status: u32, data_index: u32, node_size: u32) -> u32 {
	let byte_start = node_byte_offset + 8u + 4u * u32(bitmap_status == 3);
	if node_size == 16u { return __PREFIX___dda_u8(byte_start + data_index); }
	return __PREFIX___dda_u16(byte_start + data_index * 2u);
}
