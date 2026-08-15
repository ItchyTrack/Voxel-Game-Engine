use std::collections::HashMap;

use bevy::math::IVec3;
use tile_data::ChunkRegion;
use voxel_data::compressed_voxels::CompressedVoxels;
use voxel_data::voxels::{VoxelTypeId, Voxels};

struct StoredChunk {
	voxels: CompressedVoxels,
}

#[derive(Default)]
pub struct GridStore {
	chunks: HashMap<IVec3, StoredChunk>,
	generations: HashMap<IVec3, u64>,
}

impl GridStore {
	pub fn available_area(&self) -> Option<ChunkRegion> {
		let mut iter = self.generations.keys().copied();
		let first = iter.next()?;
		let (mut min, mut max) = (first, first);
		for chunk in iter {
			min = min.min(chunk);
			max = max.max(chunk);
		}
		ChunkRegion::from_min_max_inclusive(min, max)
	}

	pub fn contains_chunk(&self, chunk: IVec3) -> bool {
		self.chunks.contains_key(&chunk) || self.generations.contains_key(&chunk)
	}

	pub fn load_chunk(&self, chunk: IVec3) -> Option<Voxels> {
		self.chunks.get(&chunk)?.voxels.decompress().ok()
	}

	pub fn chunk_generation(&self, chunk: IVec3) -> u64 {
		self.generations.get(&chunk).copied().unwrap_or(0)
	}

	pub fn has_any_in_region(&self, min: IVec3, size: IVec3) -> bool {
		(0..size.z).any(|z| (0..size.y).any(|y| (0..size.x).any(|x| {
			self.generations.contains_key(&(min + IVec3::new(x, y, z)))
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
							.merge_from(&voxels, IVec3::new(x, y, z) * tile_data::CHUNK_SIZE);
					}
				}
			}
			return out.filter(|out| !out.is_empty());
		}
		let _ = (min, size, lod);
		None
	}

	pub fn save_chunk(&mut self, chunk: IVec3, generation: u64, voxels: &Voxels) -> bool {
		if generation < self.chunk_generation(chunk) { return false; }
		self.generations.insert(chunk, generation);
		if voxels.is_empty() {
			self.chunks.remove(&chunk);
		} else if let Ok(compressed) = CompressedVoxels::new(voxels, 0) {
			self.chunks.insert(chunk, StoredChunk { voxels: compressed });
		}
		true
	}

	pub(crate) fn take_chunk(&mut self, chunk: IVec3) -> Option<(u64, Option<CompressedVoxels>)> {
		let generation = self.generations.remove(&chunk)?;
		let voxels = self.chunks.remove(&chunk).map(|chunk| chunk.voxels);
		Some((generation, voxels))
	}

	pub fn forget_chunk(&mut self, chunk: IVec3) {
		self.chunks.remove(&chunk);
		self.generations.remove(&chunk);
	}
}

#[cfg(test)]
mod tests {
	use super::*;
	use bevy::math::U16Vec3;
	use voxel_data::voxels::{Voxel, VoxelTypeId, VoxelTypeInfo};
	use tile_data::CHUNK_SIZE;

	fn test_type_info() -> VoxelTypeInfo {
		VoxelTypeInfo { id: VoxelTypeId(7), size_bytes: 4 }
	}

	fn voxel(c: u8) -> Voxel {
		Voxel::new(test_type_info().id, [c, c, c, 255])
	}

	#[test]
	fn load_chunk_preserves_chunk_local_coordinates() {
		let mut store = GridStore::default();
		let mut voxels = Voxels::new_with_type(test_type_info());
		voxels.add_voxel(U16Vec3::new(0, 0, 0), voxel(7).get_ref());
		store.save_chunk(IVec3::new(5, 0, 0), 0, &voxels);

		let loaded = store.load_chunk(IVec3::new(5, 0, 0)).expect("chunk should load");
		assert_eq!(loaded.voxel(&U16Vec3::new(0, 0, 0)), Some(voxel(7).get_ref()));
	}

	#[test]
	fn load_lod_region_offsets_output_by_region_min_chunk() {
		let mut store = GridStore::default();
		let mut voxels = Voxels::new_with_type(test_type_info());
		voxels.add_voxel(U16Vec3::new(0, 0, 0), voxel(9).get_ref());
		store.save_chunk(IVec3::new(5, 0, 0), 0, &voxels);

		let loaded = store.load_lod_region(IVec3::new(4, 0, 0), IVec3::new(2, 1, 1), 0.0).expect("lod region should load");
		assert_eq!(loaded.voxel(&U16Vec3::new(CHUNK_SIZE as u16, 0, 0)), Some(voxel(9).get_ref()));
	}

	#[test]
	fn load_lod_region_lod1_is_not_generated_by_content_store() {
		let mut store = GridStore::default();
		let mut voxels = Voxels::new_with_type(test_type_info());
		voxels.add_voxel(U16Vec3::new(0, 0, 0), voxel(12).get_ref());
		store.save_chunk(IVec3::new(5, 0, 0), 0, &voxels);

		assert!(store.load_lod_region(IVec3::new(4, 0, 0), IVec3::new(2, 1, 1), 1.0).is_none());
	}

	#[test]
	fn load_lod_region_tracks_negative_chunk_offsets_for_lod0() {
		let mut store = GridStore::default();
		let mut voxels = Voxels::new_with_type(test_type_info());
		voxels.add_voxel(U16Vec3::new(0, 0, 0), voxel(3).get_ref());
		store.save_chunk(IVec3::new(-2, 1, 0), 0, &voxels);

		let loaded = store.load_lod_region(IVec3::new(-3, 1, 0), IVec3::new(2, 1, 1), 0.0).expect("lod region should load");
		let positions: Vec<_> = loaded.grid_tree().iter().map(|(pos, _, _)| pos).collect();
		assert!(positions.contains(&U16Vec3::new(CHUNK_SIZE as u16, 0, 0)), "positions={positions:?}");
	}
}
