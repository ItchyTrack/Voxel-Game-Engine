use std::collections::{HashMap, HashSet};

use rustc_hash::FxHashSet;
use tile_data::{NonZeroChunkRegion, TileIndex, TileIndexKey};

use crate::TileKey;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct TileDependency {
	pub(crate) region: NonZeroChunkRegion,
	pub(crate) generation: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct IndexedTileDependency {
	tile: TileKey,
	dependency: TileDependency,
}

impl TileIndexKey for IndexedTileDependency {
	fn region(self) -> NonZeroChunkRegion { self.dependency.region }
}

#[derive(Debug, Default)]
pub(crate) struct TileDependencyIndex {
	index: TileIndex<IndexedTileDependency>,
	by_tile: HashMap<TileKey, Vec<TileDependency>>,
}

impl TileDependencyIndex {
	pub(crate) fn set(&mut self, key: TileKey, dependencies: HashSet<TileDependency>) {
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

	pub(crate) fn tiles_using_region(&self, region: NonZeroChunkRegion) -> impl Iterator<Item = TileKey> {
		let mut stale = FxHashSet::default();
		self.index.for_each_overlapping(region, |dependency| { stale.insert(dependency.tile); });
		stale.into_iter()
	}
}
