use std::collections::HashMap;

use bevy::math::IVec3;
use tile_data::NonZeroChunkRegion;
use voxel_data::compressed_voxels::CompressedVoxels;
use voxel_data::voxels::{VoxelTypeId, Voxels};
use voxel_sources::edit::{GridGeneration, GridEdit};

struct StoredChunk {
	voxels: CompressedVoxels,
	// These 2 should be used to make sure active requests finish correctly.
	owned_chunk: bool,
	users_of_chunk: u32,
}

#[derive(Default)]
pub struct GridStore {
	grid_generation: GridGeneration,
	chunks: HashMap<IVec3, StoredChunk>,
}

impl GridStore {
	pub fn current_grid_generation(&self) -> GridGeneration {
		self.grid_generation
	}

	pub fn available_area(&self) -> Option<NonZeroChunkRegion> {
		let mut iter = self.chunks.keys().copied();
		let first = iter.next()?;
		let (mut min, mut max) = (first, first);
		for chunk in iter {
			min = min.min(chunk);
			max = max.max(chunk);
		}
		NonZeroChunkRegion::from_min_max(min, max)
	}

	pub fn contains_chunk(&self, chunk: IVec3) -> bool {
		self.chunks.contains_key(&chunk)
	}

	pub fn load_chunk(&self, chunk: IVec3) -> Option<Voxels> {
		self.chunks.get(&chunk)?.voxels.decompress().ok()
	}

	pub fn has_any_in_region(&self, min: IVec3, size: IVec3) -> bool {
		(0..size.z).any(|z| (0..size.y).any(|y| (0..size.x).any(|x| {
			self.chunks.contains_key(&(min + IVec3::new(x, y, z)))
		})))
	}

	pub fn voxel_type_id(&self) -> Option<VoxelTypeId> {
		self.chunks.values().find_map(|chunk| chunk.voxels.decompress().ok().map(|voxels| voxels.voxel_type_id()))
	}

	pub fn voxel_type_id_in_region(&self, min: IVec3, size: IVec3) -> Option<VoxelTypeId> {
		for z in 0..size.z {
			for y in 0..size.y {
				for x in 0..size.x {
					let chunk = min + IVec3::new(x, y, z);
					if let Some(voxels) = self.load_chunk(chunk) {
						return Some(voxels.voxel_type_id());
					}
				}
			}
		}
		None
	}

	pub fn load_lod_region(&self, min: IVec3, size: IVec3, lod: f32) -> Option<Voxels> {
		if lod <= 0.0 {
			let mut out: Option<Voxels> = None;
			for z in 0..size.z {
				for y in 0..size.y {
					for x in 0..size.x {
						let chunk = min + IVec3::new(x, y, z);
						let Some(voxels) = self.load_chunk(chunk) else { continue };
						out.get_or_insert_with(|| Voxels::new_with_type(voxels.voxel_type_info()))
							.merge_from(&voxels, IVec3::new(x, y, z) * tile_data::CHUNK_SIZE as i32);
					}
				}
			}
			return out.filter(|out| !out.is_empty());
		}
		let _ = (min, size, lod);
		None
	}

	fn apply_edit(&mut self, grid_generation: GridGeneration, edit: &dyn GridEdit) {
		// TODO: apply edit
	}

	pub fn forget_chunk(&mut self, chunk: IVec3) { self.chunks.remove(&chunk); }
}
