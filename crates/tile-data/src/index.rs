use std::collections::{HashMap, HashSet};
use std::hash::Hash;

use bevy::math::IVec3;

pub trait TileIndexKey: Copy + Eq + Hash {
	fn min(self) -> IVec3;
	fn size(self) -> IVec3;
}

#[derive(Debug, Clone)]
pub struct TileIndex<K> {
	bins: HashMap<(u8, IVec3), Vec<K>>,
	max_lod: u8,
}

impl<K> Default for TileIndex<K> {
	fn default() -> Self {
		Self { bins: HashMap::new(), max_lod: 0 }
	}
}

impl<K: TileIndexKey> TileIndex<K> {
	pub fn insert(&mut self, key: K) {
		let lod = bin_lod(key.size());
		self.max_lod = self.max_lod.max(lod);
		for_each_bin_intersecting_key(key, lod, |bin| {
			let keys = self.bins.entry((lod, bin)).or_default();
			if !keys.contains(&key) {
				keys.push(key);
			}
		});
	}

	pub fn remove(&mut self, key: K) {
		let lod = bin_lod(key.size());
		for_each_bin_intersecting_key(key, lod, |bin| {
			let Some(keys) = self.bins.get_mut(&(lod, bin)) else { return };
			keys.retain(|candidate| *candidate != key);
			if keys.is_empty() {
				self.bins.remove(&(lod, bin));
			}
		});
		if lod == self.max_lod && !self.bins.keys().any(|(lod, _)| *lod == self.max_lod) {
			self.max_lod = self.bins.keys().map(|(lod, _)| *lod).max().unwrap_or(0);
		}
	}

	pub fn for_each_overlapping(&self, min: IVec3, size: IVec3, mut f: impl FnMut(K)) {
		let area_max = min + size;
		let mut seen = HashSet::new();
		for lod in 0..=self.max_lod {
			for_each_bin_intersecting_region(min, size, lod, |bin| {
				let Some(keys) = self.bins.get(&(lod, bin)) else { return };
				for &key in keys {
					let key_min = key.min();
					let key_max = key_min + key.size();
					if key_min.cmplt(area_max).all() && min.cmplt(key_max).all() && seen.insert(key) {
						f(key);
					}
				}
			});
		}
	}

	pub fn keys_covering_point(&self, point: IVec3) -> Vec<K> {
		let mut out = Vec::new();
		let mut seen = HashSet::new();
		for lod in 0..=self.max_lod {
			let bin = align_to_lod_bin(point, lod);
			let Some(keys) = self.bins.get(&(lod, bin)) else { continue };
			for &key in keys {
				let key_min = key.min();
				if point.cmpge(key_min).all() && point.cmplt(key_min + key.size()).all() && seen.insert(key) {
					out.push(key);
				}
			}
		}
		out
	}

	pub fn is_empty(&self) -> bool {
		self.bins.is_empty()
	}
}

fn for_each_bin_intersecting_key<K: TileIndexKey>(key: K, lod: u8, f: impl FnMut(IVec3)) {
	for_each_bin_intersecting_region(key.min(), key.size(), lod, f);
}

fn bin_lod(size: IVec3) -> u8 {
	let max_size = size.max_element().max(1) as u32;
	(u32::BITS - (max_size - 1).leading_zeros()).min(i32::BITS - 2) as u8
}

fn for_each_bin_intersecting_region(min: IVec3, size: IVec3, lod: u8, mut f: impl FnMut(IVec3)) {
	let bin_size = lod_bin_size(lod);
	let max_inclusive = min + size - IVec3::ONE;
	let start = align_to_lod_bin(min, lod);
	let end = align_to_lod_bin(max_inclusive, lod);
	for x in (start.x..=end.x).step_by(bin_size as usize) {
		for y in (start.y..=end.y).step_by(bin_size as usize) {
			for z in (start.z..=end.z).step_by(bin_size as usize) {
				f(IVec3::new(x, y, z));
			}
		}
	}
}

fn align_to_lod_bin(pos: IVec3, lod: u8) -> IVec3 {
	let size = IVec3::splat(lod_bin_size(lod));
	pos.div_euclid(size) * size
}

fn lod_bin_size(lod: u8) -> i32 {
	1i32 << lod
}
