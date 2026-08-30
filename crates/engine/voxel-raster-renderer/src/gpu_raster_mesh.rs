use voxel_data::voxel_grid_tree::VoxelGridTree;
use voxel_data::voxels::VoxelTypeInfo;

use voxel_gpu::voxel_color::{VoxelGpuDataReaders, VoxelGpuNodeEntry};

use crate::gpu_data::VOXEL_DATA_ALIGNMENT;

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct MeshFace {
	pub packed: u32,
	pub voxel_data_index: u32,
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
	voxel_data_index: u32,
}

impl MeshFace {
	fn new(position: [u32; 3], size: u32, orientation: u8, voxel_data_index: u32) -> Self {
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
				| ((size_log4 & 0x3) << 21),
			voxel_data_index,
		}
	}
}

fn align_voxel_data(data: &mut Vec<u8>) {
	data.resize(data.len().max(1).next_multiple_of(VOXEL_DATA_ALIGNMENT as usize), 0);
}

pub fn make_gpu_raster_mesh(grid_tree: &VoxelGridTree, voxel_type: VoxelTypeInfo, gpu_data_readers: &VoxelGpuDataReaders) -> (Vec<u8>, Vec<u8>, u32) {
	let voxel_refs = grid_tree.iter().map(|(_, _, voxel)| voxel).collect::<Vec<_>>();
	let mut voxel_data = Vec::new();
	let encoder = gpu_data_readers
		.create_encoder(voxel_type.id, &voxel_refs, &mut voxel_data)
		.expect("raster mesh voxel type must have registered GPU data");
	align_voxel_data(&mut voxel_data);

	let mut faces = Vec::new();
	let mut occupancy = TileOccupancy::new();
	let mut leaves = Vec::new();

	// Stage occupancy and one encoded GPU-data node per leaf in one tree traversal.
	// Face generation can then use cache-friendly bit tests instead of recursively
	// searching the grid tree six times per leaf.
	for (pos, size, voxel_ref) in grid_tree.iter() {
		let voxel_data_index = u32::try_from(voxel_data.len() / VOXEL_DATA_ALIGNMENT as usize)
			.expect("raster voxel data offset exceeds u32");
		encoder.write_node(&[VoxelGpuNodeEntry::Data(voxel_ref)], &mut voxel_data);
		align_voxel_data(&mut voxel_data);

		let position = [pos.x as u32, pos.y as u32, pos.z as u32];
		let size = size as u32;
		occupancy.fill(position, size);
		leaves.push(RasterLeaf { position, size, voxel_data_index });
	}

	for leaf in leaves {
		for orientation in 0..6 {
			if !occupancy.face_is_filled(leaf.position, leaf.size, orientation) {
				faces.push(MeshFace::new(leaf.position, leaf.size, orientation, leaf.voxel_data_index));
			}
		}
	}

	let face_count = faces.len() as u32;
	(bytemuck::cast_slice(&faces).to_vec(), voxel_data, face_count)
}
