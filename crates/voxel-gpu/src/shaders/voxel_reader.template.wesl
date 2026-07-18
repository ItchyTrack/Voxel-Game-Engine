const VOXELS_HEADER_SIZE: u32 = 1u;
const VOXELS_NODE_INDEX_STEP: u32 = 4u;
const VOXELS_COLOR_BYTES: u32 = 4u;

@group(2) @binding(__BINDING__) var<storage, read> __BUFFER__: array<u32>;

fn __PREFIX___voxel_reader_u8(byte_off: u32) -> u32 {
	return (__BUFFER__[byte_off / 4u] >> ((byte_off % 4u) * 8u)) & 0xFFu;
}
fn __PREFIX___voxel_reader_palette_index(voxels_base: u32, voxel_data_index: u32, voxel_index: u32) -> u32 {
	let palette_size = __PREFIX___voxel_reader_u8(voxels_base);
	return __PREFIX___voxel_reader_u8(voxels_base + VOXELS_HEADER_SIZE + palette_size * 4 + voxel_data_index * VOXELS_NODE_INDEX_STEP + voxel_index);
}
fn __PREFIX___voxel_reader_palette_color(voxels_base: u32, voxel_data_index: u32, voxel_index: u32) -> vec4<f32> {
	let palette_size = __PREFIX___voxel_reader_u8(voxels_base);
	let palette_index = __PREFIX___voxel_reader_u8(voxels_base + VOXELS_HEADER_SIZE + palette_size * VOXELS_COLOR_BYTES + voxel_data_index * VOXELS_NODE_INDEX_STEP + voxel_index);
	let off = voxels_base + VOXELS_HEADER_SIZE + palette_index * VOXELS_COLOR_BYTES;
	return vec4<f32>(
		f32(__PREFIX___voxel_reader_u8(off + 0u)) / 255.0,
		f32(__PREFIX___voxel_reader_u8(off + 1u)) / 255.0,
		f32(__PREFIX___voxel_reader_u8(off + 2u)) / 255.0,
		f32(__PREFIX___voxel_reader_u8(off + 3u)) / 255.0,
	);
}
