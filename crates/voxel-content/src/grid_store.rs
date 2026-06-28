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
