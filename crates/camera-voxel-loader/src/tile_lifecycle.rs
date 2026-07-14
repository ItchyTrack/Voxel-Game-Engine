use std::collections::{hash_map::Entry, HashMap, HashSet};

use bevy::prelude::*;
use voxel_data::grid::GridId;
use voxel_streaming::TileIndex;

use crate::{coverage::Coverage, types::TileKey};

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum TileResolution {
	Requested,
	Empty,
	Chunk(HashSet<Entity>),
	Lod(Entity),
}

#[derive(Debug, PartialEq, Eq)]
pub(crate) enum ResolvedTile {
	Empty,
	Chunk(HashSet<Entity>),
	Lod(Entity),
}

impl TileResolution {
	pub(crate) fn is_requested(&self) -> bool { matches!(self, Self::Requested) }
	#[cfg(test)]
	pub(crate) fn is_visible(&self) -> bool { matches!(self, Self::Chunk(entities) if !entities.is_empty()) || matches!(self, Self::Lod(_)) }
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
	subgrid_render_refs: HashMap<Entity, usize>,
	lods_to_render: HashSet<Entity>,
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
		debug_assert!(matches!((&resolved, key.is_chunk()), (ResolvedTile::Empty, _) | (ResolvedTile::Chunk(_), true) | (ResolvedTile::Lod(_), false)));
		let resolution = match resolved {
			ResolvedTile::Empty => TileResolution::Empty,
			ResolvedTile::Chunk(entities) => TileResolution::Chunk(entities),
			ResolvedTile::Lod(entity) => TileResolution::Lod(entity),
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

	pub(crate) fn desired_in_area(&self, grid: GridId, min: IVec3, size: IVec3, max_lod: u8, out: &mut Vec<TileKey>) {
		out.clear();
		if let Some(index) = self.desired_index.get(&grid) {
			index.for_each_overlapping(min, size, max_lod, None, |key| out.push(key));
		}
	}

	pub(crate) fn subgrids_to_render(&self) -> impl Iterator<Item = Entity> + '_ { self.subgrid_render_refs.keys().copied() }
	pub(crate) fn lods_to_render(&self) -> &HashSet<Entity> { &self.lods_to_render }
	pub(crate) fn coverage_debug_tiles(&self) -> Vec<(TileKey, bool, bool)> { self.coverage.debug_tiles() }

	fn replace_resolution(&mut self, key: TileKey, next: TileResolution) {
		let previous = std::mem::replace(&mut self.entries.get_mut(&key).unwrap().resolution, TileResolution::Requested);
		match (&previous, &next) {
			(TileResolution::Chunk(old), TileResolution::Chunk(new)) => {
				for &entity in old.difference(new) { self.remove_subgrid_ref(entity); }
				for &entity in new.difference(old) { self.add_subgrid_ref(entity); }
			}
			(TileResolution::Lod(old), TileResolution::Lod(new)) if old == new => {}
			_ => {
				self.detach_resolution(&previous);
				self.attach_resolution(&next);
			}
		}
		self.entries.get_mut(&key).unwrap().resolution = next;
	}

	fn remove_releasable(&mut self, keys: &mut Vec<TileKey>) {
		keys.sort_by_key(|key| (key.grid.to_bits(), key.lod, key.min.x, key.min.y, key.min.z));
		keys.dedup();
		keys.retain(|key| {
			if self.desired.contains(key) {
				return false;
			}
			let Some(entry) = self.entries.remove(key) else { return false };
			self.detach_resolution(&entry.resolution);
			true
		});
	}

	fn attach_resolution(&mut self, resolution: &TileResolution) {
		match resolution {
			TileResolution::Chunk(entities) => {
				for &entity in entities { self.add_subgrid_ref(entity); }
			}
			TileResolution::Lod(entity) => { self.lods_to_render.insert(*entity); }
			TileResolution::Requested | TileResolution::Empty => {}
		}
	}

	fn detach_resolution(&mut self, resolution: &TileResolution) {
		match resolution {
			TileResolution::Chunk(entities) => {
				for &entity in entities { self.remove_subgrid_ref(entity); }
			}
			TileResolution::Lod(entity) => { self.lods_to_render.remove(entity); }
			TileResolution::Requested | TileResolution::Empty => {}
		}
	}

	fn add_subgrid_ref(&mut self, entity: Entity) { *self.subgrid_render_refs.entry(entity).or_default() += 1; }

	fn remove_subgrid_ref(&mut self, entity: Entity) {
		if let Some(count) = self.subgrid_render_refs.get_mut(&entity) {
			*count = count.saturating_sub(1);
			if *count == 0 { self.subgrid_render_refs.remove(&entity); }
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	fn grid() -> Entity { Entity::from_bits(1) }
	fn tile(lod: u8, min: IVec3) -> TileKey { TileKey { grid: grid(), lod, min } }
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

		assert!(lifecycle.resolve(key, ResolvedTile::Lod(entity)).is_empty());
		assert!(lifecycle.lods_to_render().contains(&entity));

		assert_eq!(apply(&mut lifecycle, &[], &[key]).1, vec![key]);
		assert!(!lifecycle.contains_source(key));
		assert!(!lifecycle.lods_to_render().contains(&entity));
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

	#[test]
	fn shared_chunk_entity_reference_survives_until_last_tile_releases_it() {
		let first = tile(0, IVec3::ZERO);
		let second = tile(0, IVec3::X);
		let entity = Entity::from_bits(2);
		let entities = HashSet::from([entity]);
		let mut lifecycle = TileLifecycle::default();
		apply(&mut lifecycle, &[first, second], &[]);
		let _ = lifecycle.resolve(first, ResolvedTile::Chunk(entities.clone()));
		let _ = lifecycle.resolve(second, ResolvedTile::Chunk(entities));
		assert_eq!(lifecycle.subgrids_to_render().collect::<Vec<_>>(), vec![entity]);

		apply(&mut lifecycle, &[], &[first]);
		assert_eq!(lifecycle.subgrids_to_render().collect::<Vec<_>>(), vec![entity]);
		apply(&mut lifecycle, &[], &[second]);
		assert!(lifecycle.subgrids_to_render().next().is_none());
	}
}
