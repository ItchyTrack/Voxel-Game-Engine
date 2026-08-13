use std::collections::{HashMap, HashSet};

use bevy::math::IVec3;
use tile_data::{TileIndex, TileIndexKey, ChunkRegion};

use crate::TileKey;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct TileDependency {
	pub(crate) area: ChunkRegion,
	pub(crate) edit_index: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct IndexedTileDependency {
	tile: TileKey,
	dependency: TileDependency,
}

impl TileIndexKey for IndexedTileDependency {
	fn min(self) -> IVec3 { self.dependency.area.min() }
	fn size(self) -> IVec3 { self.dependency.area.size().as_ivec3() }
}

#[derive(Debug, Default)]
pub(crate) struct TileDependencyIndex {
	index: TileIndex<IndexedTileDependency>,
	by_tile: HashMap<TileKey, Vec<TileDependency>>,
}

impl TileDependencyIndex {
	pub(crate) fn replace(&mut self, key: TileKey, dependencies: HashSet<TileDependency>) {
		self.remove(key);
		let dependencies: Vec<_> = dependencies.into_iter().collect();
		for dependency in &dependencies {
			self.index.insert(IndexedTileDependency { tile: key, dependency: *dependency });
		}
		self.by_tile.insert(key, dependencies);
	}

	pub(crate) fn remove(&mut self, key: TileKey) {
		let Some(dependencies) = self.by_tile.remove(&key) else { return };
		for dependency in dependencies {
			self.index.remove(IndexedTileDependency { tile: key, dependency });
		}
	}

	pub(crate) fn stale_tiles_for_chunk(&self, chunk: IVec3, edit_index: u64) -> impl Iterator<Item = TileKey> {
		self.index
			.keys_covering_point(chunk)
			.into_iter()
			.filter(move |dependency| dependency.dependency.edit_index < edit_index)
			.map(|dependency| dependency.tile)
	}
}
