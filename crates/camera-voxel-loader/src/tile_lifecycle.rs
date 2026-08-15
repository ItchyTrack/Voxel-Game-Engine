use std::collections::{hash_map::Entry, HashMap, HashSet};

use bevy::prelude::*;
use tile_data::TileIndex;
use voxel_data::grid::GridId;

use crate::{coverage::Coverage, types::TileKey};

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum TileResolution {
	Requested,
	Empty,
	Tile(Entity),
}

#[derive(Debug, PartialEq, Eq)]
pub(crate) enum ResolvedTile {
	Empty,
	Tile(Entity),
}

impl TileResolution {
	pub(crate) fn is_requested(&self) -> bool { matches!(self, Self::Requested) }
	#[cfg(test)]
	pub(crate) fn is_visible(&self) -> bool { matches!(self, Self::Tile(_)) }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct TileEntry {
	pub(crate) resolution: TileResolution,
}

#[derive(Debug, Default)]
pub(crate) struct TileLifecycle {
	desired: HashSet<TileKey>,
	desired_index: HashMap<GridId, TileIndex<TileKey>>,
	entries: HashMap<TileKey, TileEntry>,
	coverage: Coverage,
}

impl TileLifecycle {
	/// Applies one complete desired-set change using caller-owned output buffers. Releases are
	/// finalized only after every added key is registered, so an early addition cannot release a
	/// key added later in the same batch.
	pub(crate) fn apply_delta(
		&mut self, added: &[TileKey], removed: &[TileKey], acquire: &mut Vec<TileKey>, release: &mut Vec<TileKey>,
	) {
		acquire.clear();
		release.clear();

		for &key in added {
			if !self.desired.insert(key) {
				panic!("Dont add already desired tiles.");
			}
			self.desired_index.entry(key.grid).or_default().insert(key);
			if let Entry::Vacant(entry) = self.entries.entry(key) {
				entry.insert(TileEntry { resolution: TileResolution::Requested });
				acquire.push(key);
			}
		}

		// Establish all pending coverage before resolved reactivations satisfy dependencies. This
		// makes the entire added slice visible to coverage before it can emit a release candidate.
		for &key in added {
			self.coverage.set_wanted(key);
		}
		for &key in added {
			if !self.entries[&key].resolution.is_requested() {
				release.extend(self.coverage.set_resolved(key));
			}
		}

		for &key in removed {
			if !self.desired.remove(&key) {
				continue;
			}
			if let Some(index) = self.desired_index.get_mut(&key.grid) {
				index.remove(key);
				if index.is_empty() {
					self.desired_index.remove(&key.grid);
				}
			}
			if !self.entries.contains_key(&key) {
				continue;
			}

			release.extend(self.coverage.set_unwanted(key));
		}

		self.remove_releasable(release);
	}

	#[must_use = "released tiles must be removed from streaming state"]
	pub(crate) fn resolve(&mut self, key: TileKey, resolved: ResolvedTile) -> Vec<TileKey> {
		if !self.entries.contains_key(&key) {
			return Vec::new();
		}
				let resolution = match resolved {
			ResolvedTile::Empty => TileResolution::Empty,
			ResolvedTile::Tile(entity) => TileResolution::Tile(entity),
		};
		self.replace_resolution(key, resolution);

		let mut release = self.coverage.set_resolved(key);
		if !self.desired.contains(&key) {
			release.extend(self.coverage.set_unwanted(key));
		}
		self.remove_releasable(&mut release);
		release
	}

	pub(crate) fn contains_desired(&self, key: TileKey) -> bool { self.desired.contains(&key) }
	pub(crate) fn contains_source(&self, key: TileKey) -> bool { self.entries.contains_key(&key) }
	#[cfg(test)]
	pub(crate) fn entry(&self, key: TileKey) -> Option<&TileEntry> { self.entries.get(&key) }
	pub(crate) fn entries(&self) -> impl Iterator<Item = (TileKey, &TileEntry)> { self.entries.iter().map(|(&key, entry)| (key, entry)) }
	#[cfg(test)]
	pub(crate) fn desired(&self) -> impl Iterator<Item = TileKey> + '_ { self.desired.iter().copied() }
	#[cfg(test)]
	pub(crate) fn desired_set(&self) -> &HashSet<TileKey> { &self.desired }

	pub(crate) fn desired_in_area(&self, grid: GridId, region: ChunkRegion, out: &mut Vec<TileKey>) {
		out.clear();
		if let Some(index) = self.desired_index.get(&grid) {
			index.for_each_overlapping(region, |key| out.push(key));
		}
	}

	pub(crate) fn tiles_to_render(&self) -> impl Iterator<Item = Entity> + '_ {
		self.entries.values().filter_map(|entry| match &entry.resolution {
			TileResolution::Tile(entity) => Some(*entity),
			TileResolution::Requested | TileResolution::Empty => None,
		})
	}
	pub(crate) fn coverage_debug_tiles(&self) -> Vec<(TileKey, bool, bool)> { self.coverage.debug_tiles() }

	fn replace_resolution(&mut self, key: TileKey, next: TileResolution) {
		self.entries.get_mut(&key).unwrap().resolution = next;
	}

	fn remove_releasable(&mut self, keys: &mut Vec<TileKey>) {
		keys.sort_by_key(|key| (key.grid.to_bits(), key.lod, key.min().x, key.min().y, key.min().z));
		keys.dedup();
		keys.retain(|key| {
			if self.desired.contains(key) {
				return false;
			}
			self.entries.remove(key).is_some()
		});
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	fn grid() -> Entity { Entity::from_bits(1) }
	fn tile(lod: u8, min: IVec3) -> TileKey { TileKey::new(grid(), voxel_streaming::TileClassId(0), lod, min) }
	fn apply(lifecycle: &mut TileLifecycle, added: &[TileKey], removed: &[TileKey]) -> (Vec<TileKey>, Vec<TileKey>) {
		let mut acquire = Vec::new();
		let mut release = Vec::new();
		lifecycle.apply_delta(added, removed, &mut acquire, &mut release);
		(acquire, release)
	}

	#[test]
	fn lifecycle_acquires_resolves_and_releases_through_small_api() {
		let key = tile(1, IVec3::ZERO);
		let entity = Entity::from_bits(2);
		let mut lifecycle = TileLifecycle::default();

		let (acquire, release) = apply(&mut lifecycle, &[key], &[]);
		assert_eq!(acquire, vec![key]);
		assert!(release.is_empty());
		assert_eq!(lifecycle.entry(key), Some(&TileEntry { resolution: TileResolution::Requested }));

		assert!(lifecycle.resolve(key, ResolvedTile::Tile(entity)).is_empty());
		assert!(lifecycle.tiles_to_render().any(|candidate| candidate == entity));

		assert_eq!(apply(&mut lifecycle, &[], &[key]).1, vec![key]);
		assert!(!lifecycle.contains_source(key));
		assert!(!lifecycle.tiles_to_render().any(|candidate| candidate == entity));
	}

	#[test]
	fn reactivating_retiring_empty_source_cannot_be_released_by_former_replacements() {
		let old = tile(1, IVec3::ZERO);
		let mut replacements = Vec::new();
		for x in 0..2 {
			for y in 0..2 {
				for z in 0..2 {
					replacements.push(tile(0, IVec3::new(x, y, z)));
				}
			}
		}
		let mut lifecycle = TileLifecycle::default();
		apply(&mut lifecycle, &[old], &[]);
		let _ = lifecycle.resolve(old, ResolvedTile::Empty);
		apply(&mut lifecycle, &replacements, &[]);
		assert!(apply(&mut lifecycle, &[], &[old]).1.is_empty());
		assert!(!lifecycle.contains_desired(old));

		apply(&mut lifecycle, &[old], &[]);
		assert_eq!(lifecycle.entry(old), Some(&TileEntry { resolution: TileResolution::Empty }));
		for replacement in replacements {
			assert_eq!(apply(&mut lifecycle, &[], &[replacement]).1, vec![replacement]);
		}
		assert!(lifecycle.contains_desired(old));
		assert!(lifecycle.contains_source(old));
	}

}
