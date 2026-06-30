use std::collections::HashMap;

use bevy::math::IVec3;
use voxel_data::compressed_voxels::CompressedVoxels;
use voxel_data::voxels::Voxels;

use crate::lod_downsample::downsample_region;

#[derive(Default)]
pub struct GridStore {
	chunks: HashMap<IVec3, CompressedVoxels>,
}

impl GridStore {
	pub fn available_area(&self) -> Option<(IVec3, IVec3)> {
		let mut iter = self.chunks.keys().copied();
		let first = iter.next()?;
		let (mut min, mut max) = (first, first);
		for chunk in iter {
			min = min.min(chunk);
			max = max.max(chunk);
		}
		Some((min, max - min + IVec3::ONE))
	}

	pub fn contains_chunk(&self, chunk: IVec3) -> bool {
		self.chunks.contains_key(&chunk)
	}

	pub fn load_chunk(&self, chunk: IVec3) -> Option<Voxels> {
		self.chunks.get(&chunk)?.decompress().ok()
	}

	pub fn has_any_in_region(&self, min: IVec3, size: IVec3) -> bool {
		(0..size.z).any(|z| (0..size.y).any(|y| (0..size.x).any(|x| {
			self.chunks.contains_key(&(min + IVec3::new(x, y, z)))
		})))
	}

	pub fn load_lod_region(&self, min: IVec3, size: IVec3, lod: f32) -> Option<Voxels> {
		if lod <= 0.0 {
			let mut out = Voxels::new();
			for z in 0..size.z {
				for y in 0..size.y {
					for x in 0..size.x {
						let chunk = min + IVec3::new(x, y, z);
						let Some(voxels) = self.load_chunk(chunk) else { continue };
						out.merge_from(&voxels, IVec3::new(x, y, z) * voxel_streaming::CHUNK_SIZE);
					}
				}
			}
			return (!out.is_empty()).then_some(out);
		}
		let region = downsample_region(min, size, lod, |chunk| self.load_chunk(chunk));
		(!region.is_empty()).then_some(region)
	}

	pub fn save_chunk(&mut self, chunk: IVec3, voxels: &Voxels) {
		if voxels.is_empty() {
			self.chunks.remove(&chunk);
		} else if let Ok(compressed) = CompressedVoxels::new(voxels, 0) {
			self.chunks.insert(chunk, compressed);
		}
	}

	pub fn forget_chunk(&mut self, chunk: IVec3) {
		self.chunks.remove(&chunk);
	}
}

#[cfg(test)]
mod tests {
	use super::*;
	use bevy::math::U16Vec3;
	use voxel_data::voxels::Voxel;
	use voxel_streaming::CHUNK_SIZE;

	fn voxel(c: u8) -> Voxel {
		Voxel { color: [c, c, c, 255], mass: 1 }
	}

	#[test]
	fn load_chunk_preserves_chunk_local_coordinates() {
		let mut store = GridStore::default();
		let mut voxels = Voxels::new();
		voxels.add_voxel(U16Vec3::new(0, 0, 0), voxel(7));
		store.save_chunk(IVec3::new(5, 0, 0), &voxels);

		let loaded = store.load_chunk(IVec3::new(5, 0, 0)).expect("chunk should load");
		assert_eq!(loaded.voxel(&U16Vec3::new(0, 0, 0)), Some(&voxel(7)));
	}

	#[test]
	fn load_lod_region_offsets_output_by_region_min_chunk() {
		let mut store = GridStore::default();
		let mut voxels = Voxels::new();
		voxels.add_voxel(U16Vec3::new(0, 0, 0), voxel(9));
		store.save_chunk(IVec3::new(5, 0, 0), &voxels);

		let loaded = store.load_lod_region(IVec3::new(4, 0, 0), IVec3::new(2, 1, 1), 0.0).expect("lod region should load");
		assert_eq!(loaded.voxel(&U16Vec3::new(CHUNK_SIZE as u16, 0, 0)), Some(&voxel(9)));
	}

	#[test]
	fn load_lod_region_lod1_keeps_voxel_position_relative_to_tile_min() {
		let mut store = GridStore::default();
		let mut voxels = Voxels::new();
		voxels.add_voxel(U16Vec3::new(0, 0, 0), voxel(12));
		store.save_chunk(IVec3::new(5, 0, 0), &voxels);

		let loaded = store.load_lod_region(IVec3::new(4, 0, 0), IVec3::new(2, 1, 1), 1.0).expect("lod region should load");
		let positions: Vec<_> = loaded.grid_tree().iter().map(|(pos, _, _)| pos).collect();
		assert!(positions.contains(&U16Vec3::new((CHUNK_SIZE / 2) as u16, 0, 0)), "positions={positions:?}");
	}

	#[test]
	fn load_lod_region_tracks_negative_chunk_offsets_correctly() {
		let mut store = GridStore::default();
		let mut voxels = Voxels::new();
		voxels.add_voxel(U16Vec3::new(0, 0, 0), voxel(3));
		store.save_chunk(IVec3::new(-2, 1, 0), &voxels);

		let loaded = store.load_lod_region(IVec3::new(-3, 1, 0), IVec3::new(2, 1, 1), 1.0).expect("lod region should load");
		let positions: Vec<_> = loaded.grid_tree().iter().map(|(pos, _, _)| pos).collect();
		assert!(positions.contains(&U16Vec3::new((CHUNK_SIZE / 2) as u16, 0, 0)), "positions={positions:?}");
	}
}
