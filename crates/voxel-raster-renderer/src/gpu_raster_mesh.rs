use std::collections::HashMap;

use voxel_data::voxel_grid_tree::VoxelGridTree;
use voxel_data::voxels::VoxelTypeInfo;

use voxel_gpu::voxel_color::VoxelGpuDataReaders;

#[repr(transparent)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct MeshFace {
	pub packed: u32,
}

const TILE_EXTENT: usize = 64;
const OCCUPANCY_ROW_COUNT: usize = TILE_EXTENT * TILE_EXTENT;

struct TileOccupancy {
	// One x-axis bit row for every (y, z) coordinate in the tile.
	rows: Box<[u64]>,
}

impl TileOccupancy {
	fn new() -> Self {
		Self { rows: vec![0; OCCUPANCY_ROW_COUNT].into_boxed_slice() }
	}

	#[inline]
	fn row_index(y: usize, z: usize) -> usize { z * TILE_EXTENT + y }

	#[inline]
	fn x_mask(x: usize, size: usize) -> u64 {
		if size == TILE_EXTENT { u64::MAX } else { ((1u64 << size) - 1) << x }
	}

	fn fill(&mut self, position: [u32; 3], size: u32) {
		let [x, y, z] = position.map(|coordinate| coordinate as usize);
		let size = size as usize;
		debug_assert!(x + size <= TILE_EXTENT && y + size <= TILE_EXTENT && z + size <= TILE_EXTENT);
		let mask = Self::x_mask(x, size);
		for z in z..z + size {
			for y in y..y + size {
				self.rows[Self::row_index(y, z)] |= mask;
			}
		}
	}

	fn face_is_filled(&self, position: [u32; 3], size: u32, orientation: u8) -> bool {
		let [x, y, z] = position.map(|coordinate| coordinate as usize);
		let size = size as usize;
		match orientation {
			0 => {
				let neighbor_x = x + size;
				neighbor_x < TILE_EXTENT && self.x_plane_is_filled(neighbor_x, y, z, size)
			}
			1 => x > 0 && self.x_plane_is_filled(x - 1, y, z, size),
			2 => {
				let neighbor_y = y + size;
				neighbor_y < TILE_EXTENT && self.y_plane_is_filled(x, neighbor_y, z, size)
			}
			3 => y > 0 && self.y_plane_is_filled(x, y - 1, z, size),
			4 => {
				let neighbor_z = z + size;
				neighbor_z < TILE_EXTENT && self.z_plane_is_filled(x, y, neighbor_z, size)
			}
			5 => z > 0 && self.z_plane_is_filled(x, y, z - 1, size),
			_ => unreachable!("raster face orientation is out of range"),
		}
	}

	#[inline]
	fn x_plane_is_filled(&self, x: usize, y: usize, z: usize, size: usize) -> bool {
		let bit = 1u64 << x;
		(z..z + size).all(|z| (y..y + size).all(|y| self.rows[Self::row_index(y, z)] & bit != 0))
	}

	#[inline]
	fn y_plane_is_filled(&self, x: usize, y: usize, z: usize, size: usize) -> bool {
		let mask = Self::x_mask(x, size);
		(z..z + size).all(|z| self.rows[Self::row_index(y, z)] & mask == mask)
	}

	#[inline]
	fn z_plane_is_filled(&self, x: usize, y: usize, z: usize, size: usize) -> bool {
		let mask = Self::x_mask(x, size);
		(y..y + size).all(|y| self.rows[Self::row_index(y, z)] & mask == mask)
	}
}

#[derive(Clone, Copy)]
struct RasterLeaf {
	position: [u32; 3],
	size: u32,
	palette_index: u8,
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

pub fn make_gpu_raster_mesh(grid_tree: &VoxelGridTree, _voxel_type: VoxelTypeInfo, _gpu_data_readers: &VoxelGpuDataReaders) -> (Vec<u8>, Vec<u8>, u32) {
	let mut faces = Vec::new();
	let mut palette_vec: Vec<[u8; 4]> = Vec::new();
	let mut palette_map: HashMap<[u8; 4], u8> = HashMap::new();
	let mut palette_overflowed = false;
	let mut occupancy = TileOccupancy::new();
	let mut leaves = Vec::new();

	// Stage occupancy in one tree traversal. Face generation can then use cache-friendly
	// bit tests instead of recursively searching the grid tree six times per leaf.
	for (pos, size, _voxel_ref) in grid_tree.iter() {
		let color = [255, 255, 255, 255];
		let position = [pos.x as u32, pos.y as u32, pos.z as u32];
		let size = size as u32;
		let palette_index = palette_index_for_color(color, &mut palette_map, &mut palette_vec, &mut palette_overflowed);
		occupancy.fill(position, size);
		leaves.push(RasterLeaf { position, size, palette_index });
	}

	for leaf in leaves {
		for orientation in 0..6 {
			if !occupancy.face_is_filled(leaf.position, leaf.size, orientation) {
				faces.push(MeshFace::new(leaf.position, leaf.size, leaf.palette_index, orientation));
			}
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
