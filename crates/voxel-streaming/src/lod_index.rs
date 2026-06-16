use std::collections::{HashMap, HashSet};

use bevy::math::IVec3;

use crate::loader::LodKey;

#[derive(Debug, Default, Clone)]
pub(crate) struct LodIndex {
	bins: HashMap<(u8, IVec3), Vec<LodKey>>,
	max_lod: u8,
}

impl LodIndex {
	pub(crate) fn insert(&mut self, key: LodKey) {
		self.max_lod = self.max_lod.max(key.lod);
		for bin in bins_intersecting(key) {
			let keys = self.bins.entry((key.lod, bin)).or_default();
			if !keys.contains(&key) {
				keys.push(key);
			}
		}
	}

	pub(crate) fn remove(&mut self, key: LodKey) {
		for bin in bins_intersecting(key) {
			let Some(keys) = self.bins.get_mut(&(key.lod, bin)) else { continue };
			keys.retain(|candidate| *candidate != key);
			if keys.is_empty() {
				self.bins.remove(&(key.lod, bin));
			}
		}
		if key.lod == self.max_lod && !self.bins.keys().any(|(lod, _)| *lod == self.max_lod) {
			self.max_lod = self.bins.keys().map(|(lod, _)| *lod).max().unwrap_or(0);
		}
	}

	pub(crate) fn lods_covering_chunk(&self, chunk: IVec3) -> Vec<LodKey> {
		let mut seen = HashSet::new();
		let mut out = Vec::new();
		for lod in 1..=self.max_lod {
			let bin = align_to_lod_bin(chunk, lod);
			let Some(keys) = self.bins.get(&(lod, bin)) else { continue };
			for &key in keys {
				if contains_chunk(key, chunk) && seen.insert(key) {
					out.push(key);
				}
			}
		}
		out
	}

	pub(crate) fn clear(&mut self) {
		self.bins.clear();
		self.max_lod = 0;
	}

	pub(crate) fn is_empty(&self) -> bool {
		self.bins.is_empty()
	}
}

fn bins_intersecting(key: LodKey) -> impl Iterator<Item = IVec3> {
	let bin_size = lod_bin_size(key.lod);
	let min = align_to_lod_bin(key.min, key.lod);
	let max = align_to_lod_bin(key.min + key.size - IVec3::ONE, key.lod);
	let mut bins = Vec::new();
	for x in (min.x..=max.x).step_by(bin_size as usize) {
		for y in (min.y..=max.y).step_by(bin_size as usize) {
			for z in (min.z..=max.z).step_by(bin_size as usize) {
				bins.push(IVec3::new(x, y, z));
			}
		}
	}
	bins.into_iter()
}

fn contains_chunk(key: LodKey, chunk: IVec3) -> bool {
	chunk.cmpge(key.min).all() && chunk.cmplt(key.min + key.size).all()
}

fn align_to_lod_bin(chunk: IVec3, lod: u8) -> IVec3 {
	let size = IVec3::splat(lod_bin_size(lod));
	chunk.div_euclid(size) * size
}

fn lod_bin_size(lod: u8) -> i32 {
	1i32 << lod
}

#[cfg(test)]
mod tests {
	use super::*;

	fn key(lod: u8, min: IVec3, size: IVec3) -> LodKey {
		LodKey { lod, min, size }
	}

	#[test]
	fn finds_lod_by_dirty_chunk_bin() {
		let mut index = LodIndex::default();
		let lod = key(2, IVec3::new(4, 0, 0), IVec3::splat(4));
		index.insert(lod);

		assert_eq!(index.lods_covering_chunk(IVec3::new(6, 1, 1)), vec![lod]);
		assert!(index.lods_covering_chunk(IVec3::new(8, 1, 1)).is_empty());
	}

	#[test]
	fn supports_lods_spanning_multiple_bins() {
		let mut index = LodIndex::default();
		let lod = key(1, IVec3::new(0, 0, 0), IVec3::new(4, 2, 2));
		index.insert(lod);

		assert_eq!(index.lods_covering_chunk(IVec3::new(3, 1, 1)), vec![lod]);
	}

	#[test]
	fn remove_drops_lod_from_lookup() {
		let mut index = LodIndex::default();
		let lod = key(1, IVec3::ZERO, IVec3::splat(2));
		index.insert(lod);
		index.remove(lod);

		assert!(index.lods_covering_chunk(IVec3::ZERO).is_empty());
		assert!(index.is_empty());
	}
}
