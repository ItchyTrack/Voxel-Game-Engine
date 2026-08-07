use crate::{TileIndex, TileIndexKey, TileKey};

impl TileIndexKey for TileKey {
	fn lod(self) -> u8 { self.lod }
	fn min(self) -> bevy::math::IVec3 { self.min }
	fn size(self) -> bevy::math::IVec3 { self.size }
}

#[derive(Debug, Default, Clone)]
pub(crate) struct TileStateIndex(TileIndex<TileKey>);

impl TileStateIndex {
	pub(crate) fn insert(&mut self, key: TileKey) { self.0.insert(key); }
	pub(crate) fn remove(&mut self, key: TileKey) { self.0.remove(key); }
	pub(crate) fn tiles_covering_chunk(&self, chunk: bevy::math::IVec3) -> Vec<TileKey> {
		self.0.keys_covering_point(chunk, 0)
	}
}

#[cfg(test)]
mod tests {
	use bevy::math::IVec3;
	use super::*;
	use crate::TileClassId;

	fn key(lod: u8, min: IVec3, size: IVec3) -> TileKey {
		TileKey { lod, min, size, class: TileClassId(0) }
	}

	#[test]
	fn finds_tiles_by_dirty_chunk_bin() {
		let mut index = TileStateIndex::default();
		let tile = key(2, IVec3::new(4, 0, 0), IVec3::splat(4));
		index.insert(tile);
		assert_eq!(index.tiles_covering_chunk(IVec3::new(6, 1, 1)), vec![tile]);
		assert!(index.tiles_covering_chunk(IVec3::new(8, 1, 1)).is_empty());
	}

	#[test]
	fn supports_tiles_spanning_multiple_bins() {
		let mut index = TileStateIndex::default();
		let tile = key(1, IVec3::new(0, 0, 0), IVec3::new(4, 2, 2));
		index.insert(tile);
		assert_eq!(index.tiles_covering_chunk(IVec3::new(3, 1, 1)), vec![tile]);
	}
}
