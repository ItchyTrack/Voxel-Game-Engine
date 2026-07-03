use bevy::math::IVec3;
use voxel_data::voxels::VoxelPalette;
use voxel_data::voxel_grid_tree::VoxelGridTree;

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct MeshFace {
	pub position: [i8; 3],
	pub orientation_size: u8,
	pub color: [u8; 4],
}

impl MeshFace {
	fn new(position: [u32; 3], size: u32, color: [u8; 4], orientation: u8) -> Self {
		let size_log2 = size.ilog2() as u8;
		debug_assert_eq!(1u32 << size_log2, size);
		Self {
			position: [
				i8::try_from(position[0]).expect("mesh face x out of range"),
				i8::try_from(position[1]).expect("mesh face y out of range"),
				i8::try_from(position[2]).expect("mesh face z out of range"),
			],
			orientation_size: orientation | (size_log2 << 3),
			color,
		}
	}
}

pub fn make_gpu_raster_mesh(grid_tree: &VoxelGridTree, palette: &VoxelPalette) -> (Vec<u8>, u32) {
	let mut faces = Vec::new();

	for (pos, size, voxel_id) in grid_tree.iter() {
		let Some(voxel) = palette.voxel(voxel_id) else { continue; };
		let size_u32 = size as u32;
		let size_i32 = size as i32;
		let position = [pos.x as u32, pos.y as u32, pos.z as u32];

		if !grid_tree.is_area_filled(&(pos + bevy::math::U16Vec3::X * (size as u16)), IVec3::new(1, size_i32, size_i32)) {
			faces.push(MeshFace::new(position, size_u32, voxel.color, 0));
		}
		if pos.x == 0 || !grid_tree.is_area_filled(&(pos - bevy::math::U16Vec3::X), IVec3::new(1, size_i32, size_i32)) {
			faces.push(MeshFace::new(position, size_u32, voxel.color, 1));
		}
		if !grid_tree.is_area_filled(&(pos + bevy::math::U16Vec3::Y * (size as u16)), IVec3::new(size_i32, 1, size_i32)) {
			faces.push(MeshFace::new(position, size_u32, voxel.color, 2));
		}
		if pos.y == 0 || !grid_tree.is_area_filled(&(pos - bevy::math::U16Vec3::Y), IVec3::new(size_i32, 1, size_i32)) {
			faces.push(MeshFace::new(position, size_u32, voxel.color, 3));
		}
		if !grid_tree.is_area_filled(&(pos + bevy::math::U16Vec3::Z * (size as u16)), IVec3::new(size_i32, size_i32, 1)) {
			faces.push(MeshFace::new(position, size_u32, voxel.color, 4));
		}
		if pos.z == 0 || !grid_tree.is_area_filled(&(pos - bevy::math::U16Vec3::Z), IVec3::new(size_i32, size_i32, 1)) {
			faces.push(MeshFace::new(position, size_u32, voxel.color, 5));
		}
	}

	let face_count = faces.len() as u32;
	(bytemuck::cast_slice(&faces).to_vec(), face_count)
}
