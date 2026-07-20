const VOXELS_NODE_OFFSET_UNIT_BYTES: u32 = 16u;
const VOXELS_COLOR_BYTES: u32 = 4u;

@group(2) @binding(__BINDING__) var<storage, read> __BUFFER__: array<u32>;

fn __PREFIX___voxel_reader_u8(byte_off: u32) -> u32 {
	return (__BUFFER__[byte_off / 4u] >> ((byte_off % 4u) * 8u)) & 0xFFu;
}
fn __PREFIX___voxel_reader_color(voxels_base: u32, voxel_data_index: u32, voxel_index: u32) -> vec4<f32> {
	let off = voxels_base + voxel_data_index * VOXELS_NODE_OFFSET_UNIT_BYTES + voxel_index * VOXELS_COLOR_BYTES;
	return vec4<f32>(
		f32(__PREFIX___voxel_reader_u8(off + 0u)) / 255.0,
		f32(__PREFIX___voxel_reader_u8(off + 1u)) / 255.0,
		f32(__PREFIX___voxel_reader_u8(off + 2u)) / 255.0,
		f32(__PREFIX___voxel_reader_u8(off + 3u)) / 255.0,
	);
}
