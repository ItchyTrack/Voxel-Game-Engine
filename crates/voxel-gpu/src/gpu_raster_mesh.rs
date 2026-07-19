use std::collections::HashMap;

use bevy::math::IVec3;
use voxel_data::voxel_grid_tree::VoxelGridTree;
use voxel_data::voxels::{Voxel, VoxelPalette, VoxelTypeInfo};

use crate::voxel_color::VoxelColorReaders;

#[repr(transparent)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct MeshFace {
	pub packed: u32,
}

impl MeshFace {
	fn new(position: [u32; 3], size: u32, palette_index: u8, orientation: u8) -> Self {
		debug_assert!(position.iter().all(|&coord| coord < 64), "mesh face position out of 6-bit range: {position:?}");
		debug_assert!(orientation < 8, "mesh face orientation out of range: {orientation}");

		let size_log2 = size.ilog2();
		debug_assert_eq!(1u32 << size_log2, size, "mesh face size must be a power of two: {size}");
		debug_assert_eq!(size_log2 % 2, 0, "mesh face size must be a power of four: {size}");
		let size_log4 = size_log2 / 2;
		debug_assert!(size_log4 < 4, "mesh face size out of 2-bit range: {size}");

		Self {
			packed: (position[0] & 0x3F)
				| ((position[1] & 0x3F) << 6)
				| ((position[2] & 0x3F) << 12)
				| (u32::from(orientation & 0x7) << 18)
				| ((size_log4 & 0x3) << 21)
				| (u32::from(palette_index) << 23),
		}
	}
}

fn nearest_palette_index(color: [u8; 4], palette: &[[u8; 4]]) -> u8 {
	palette
		.iter()
		.map(|candidate| {
			let dr = i32::from(color[0]) - i32::from(candidate[0]);
			let dg = i32::from(color[1]) - i32::from(candidate[1]);
			let db = i32::from(color[2]) - i32::from(candidate[2]);
			let da = i32::from(color[3]) - i32::from(candidate[3]);
			(dr * dr + dg * dg + db * db + da * da) as u32
		})
		.enumerate()
		.min_by_key(|(_, distance)| *distance)
		.map(|(index, _)| index as u8)
		.unwrap_or(0)
}

fn palette_index_for_color(
	color: [u8; 4],
	palette_map: &mut HashMap<[u8; 4], u8>,
	palette_vec: &mut Vec<[u8; 4]>,
	palette_overflowed: &mut bool,
) -> u8 {
	if let Some(index) = palette_map.get(&color).copied() {
		return index;
	}

	if palette_vec.len() < usize::from(u8::MAX) + 1 {
		let index = palette_vec.len() as u8;
		palette_vec.push(color);
		palette_map.insert(color, index);
		return index;
	}

	if !*palette_overflowed {
		log::warn!("raster mesh palette exceeded 256 colors; approximating extra colors with nearest palette entry");
		*palette_overflowed = true;
	}
	nearest_palette_index(color, palette_vec)
}

pub fn make_gpu_raster_mesh(grid_tree: &VoxelGridTree, palette: &VoxelPalette, voxel_type: VoxelTypeInfo, color_readers: &VoxelColorReaders) -> (Vec<u8>, Vec<u8>, u32) {
	let mut faces = Vec::new();
	let mut palette_vec: Vec<[u8; 4]> = Vec::new();
	let mut palette_map: HashMap<[u8; 4], u8> = HashMap::new();
	let mut palette_overflowed = false;

	for (pos, size, voxel_id) in grid_tree.iter() {
		let Some(raw) = palette.raw(voxel_id) else { continue; };
		let voxel = Voxel::new(voxel_type.id, raw.to_vec());
		let Some(color) = color_readers.color(&voxel) else { continue; };
		let size_u32 = size as u32;
		let size_i32 = size as i32;
		let position = [pos.x as u32, pos.y as u32, pos.z as u32];
		let palette_index = palette_index_for_color(color, &mut palette_map, &mut palette_vec, &mut palette_overflowed);

		if !grid_tree.is_area_filled(&(pos + bevy::math::U16Vec3::X * size), IVec3::new(1, size_i32, size_i32)) {
			faces.push(MeshFace::new(position, size_u32, palette_index, 0));
		}
		if pos.x == 0 || !grid_tree.is_area_filled(&(pos - bevy::math::U16Vec3::X), IVec3::new(1, size_i32, size_i32)) {
			faces.push(MeshFace::new(position, size_u32, palette_index, 1));
		}
		if !grid_tree.is_area_filled(&(pos + bevy::math::U16Vec3::Y * size), IVec3::new(size_i32, 1, size_i32)) {
			faces.push(MeshFace::new(position, size_u32, palette_index, 2));
		}
		if pos.y == 0 || !grid_tree.is_area_filled(&(pos - bevy::math::U16Vec3::Y), IVec3::new(size_i32, 1, size_i32)) {
			faces.push(MeshFace::new(position, size_u32, palette_index, 3));
		}
		if !grid_tree.is_area_filled(&(pos + bevy::math::U16Vec3::Z * size), IVec3::new(size_i32, size_i32, 1)) {
			faces.push(MeshFace::new(position, size_u32, palette_index, 4));
		}
		if pos.z == 0 || !grid_tree.is_area_filled(&(pos - bevy::math::U16Vec3::Z), IVec3::new(size_i32, size_i32, 1)) {
			faces.push(MeshFace::new(position, size_u32, palette_index, 5));
		}
	}

	let face_count = faces.len() as u32;
	(
		bytemuck::cast_slice(&faces).to_vec(),
		bytemuck::cast_slice(&palette_vec).to_vec(),
		face_count,
	)
}

#[cfg(test)]
mod tests {
	use super::MeshFace;

	#[test]
	fn mesh_face_packs_fields_into_one_u32() {
		let face = MeshFace::new([63, 17, 5], 16, 200, 6);
		assert_eq!(face.packed & 0x3F, 63);
		assert_eq!((face.packed >> 6) & 0x3F, 17);
		assert_eq!((face.packed >> 12) & 0x3F, 5);
		assert_eq!((face.packed >> 18) & 0x7, 6);
		assert_eq!((face.packed >> 21) & 0x3, 2);
		assert_eq!((face.packed >> 23) & 0xFF, 200);
	}
}
