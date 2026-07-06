use crate::{LodKey, TileIndex, TileIndexKey};

impl TileIndexKey for LodKey {
	fn lod(self) -> u8 { self.lod }
	fn min(self) -> bevy::math::IVec3 { self.min }
	fn size(self) -> bevy::math::IVec3 { self.size }
}

#[derive(Debug, Default, Clone)]
pub(crate) struct LodIndex(TileIndex<LodKey>);

impl LodIndex {
	pub(crate) fn insert(&mut self, key: LodKey) {
		self.0.insert(key);
	}

	pub(crate) fn remove(&mut self, key: LodKey) {
		self.0.remove(key);
	}

	pub(crate) fn lods_covering_chunk(&self, chunk: bevy::math::IVec3) -> Vec<LodKey> {
		self.0.keys_covering_point(chunk, 1)
	}

	#[cfg(test)]
	pub(crate) fn is_empty(&self) -> bool {
		self.0.is_empty()
	}
}

#[cfg(test)]
mod tests {
	use bevy::math::IVec3;

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
