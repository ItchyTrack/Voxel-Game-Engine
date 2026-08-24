use std::collections::{HashMap, HashSet};

use voxel_data::grid_tree::NonZeroVoxelRegion;

use crate::{types::TileKey, unresolved_tile_index::UnresolvedTileIndex};

/// Tracks pending coverage and the loaded tiles retained until that coverage resolves.
#[derive(Clone, Debug, Default)]
pub(crate) struct Coverage {
	pending: UnresolvedTileIndex,
	replacements_by_source: HashMap<TileKey, HashSet<TileKey>>,
	sources_by_replacement: HashMap<TileKey, HashSet<TileKey>>,
}

impl Coverage {
	pub(crate) fn debug_tiles(&self) -> Vec<(TileKey, bool, bool)> {
		let mut roles: HashMap<TileKey, (bool, bool)> = HashMap::new();
		for key in self.pending.keys() {
			roles.entry(key).or_default().0 = true;
		}
		for &key in self.replacements_by_source.keys() {
			roles.entry(key).or_default().1 = true;
		}
		let mut tiles: Vec<_> = roles.into_iter().map(|(key, (pending, retained))| (key, pending, retained)).collect();
		tiles.sort_by_key(|(key, _, _)| {
			let min = key.region.min();
			(key.grid.to_bits(), key.lod, min.x, min.y, min.z)
		});
		tiles
	}

	/// Registers coverage that is now wanted. If the tile was previously being retained, reverse
	/// that handoff so its former replacements can be removed once this tile is available again.
	pub(crate) fn set_wanted(&mut self, key: TileKey) {
		if self.pending.contains(key) {
			return;
		}
		self.pending.insert(key);

		if let Some(replacements) = self.detach_source(key) {
			for replacement in replacements {
				self.add_dependency(replacement, key);
			}
		}
		let overlapping_sources: Vec<_> = self
			.replacements_by_source
			.keys()
			.copied()
			.filter(|source| tiles_overlap(*source, key))
			.collect();
		for source in overlapping_sources {
			self.add_dependency(source, key);
		}
	}

	/// Marks wanted coverage as resolved (visible or empty) and returns retained tiles that it now safely replaces.
	#[must_use = "tiles returned by Coverage must be removed from loader, render, and streaming state"]
	pub(crate) fn set_resolved(&mut self, key: TileKey) -> Vec<TileKey> {
		self.pending.remove(key);
		self.apply_satisfied(key)
	}

	/// Removes a tile from the wanted set and returns tiles whose requests or coverage can now be
	/// removed without opening a hole.
	#[must_use = "tiles returned by Coverage must be removed from loader, render, and streaming state"]
	pub(crate) fn set_unwanted(&mut self, key: TileKey) -> Vec<TileKey> {
		if self.pending.remove(key) {
			let mut removable = self.apply_satisfied(key);
			if !self.replacements_by_source.contains_key(&key) {
				removable.push(key);
			}
			return self.only_unwanted(removable);
		}

		if self.replacements_by_source.contains_key(&key) {
			return Vec::new();
		}

		let mut replacements = HashSet::new();
		if let Some(region) = NonZeroVoxelRegion::new(key.region.min(), key.region.size()) {
			self.pending.for_each_in_region(key.grid, key.class, region, u8::MAX, Some(key.lod), |candidate| {
				replacements.insert(candidate);
			});
		}
		if replacements.is_empty() {
			vec![key]
		} else {
			for replacement in replacements {
				self.add_dependency(key, replacement);
			}
			Vec::new()
		}
	}

	fn add_dependency(&mut self, source: TileKey, replacement: TileKey) {
		if source == replacement {
			return;
		}
		if self.replacements_by_source.entry(source).or_default().insert(replacement) {
			self.sources_by_replacement.entry(replacement).or_default().insert(source);
		}
	}

	fn apply_satisfied(&mut self, key: TileKey) -> Vec<TileKey> {
		let mut satisfied = HashSet::from([key]);
		let mut pending = vec![key];
		let mut removable = Vec::new();

		while let Some(replacement) = pending.pop() {
			let Some(sources) = self.sources_by_replacement.get(&replacement) else { continue };
			for source in sources {
				if satisfied.contains(source) {
					continue;
				}
				let Some(replacements) = self.replacements_by_source.get(source) else { continue };
				if replacements.iter().all(|replacement| satisfied.contains(replacement)) {
					satisfied.insert(*source);
					pending.push(*source);
					removable.push(*source);
				}
			}
		}

		for replacement in &satisfied {
			let Some(sources) = self.sources_by_replacement.remove(replacement) else { continue };
			for source in sources {
				if let Some(replacements) = self.replacements_by_source.get_mut(&source) {
					replacements.remove(replacement);
				}
			}
		}
		for source in &removable {
			self.replacements_by_source.remove(source);
		}
		self.only_unwanted(removable)
	}

	fn detach_source(&mut self, source: TileKey) -> Option<HashSet<TileKey>> {
		let replacements = self.replacements_by_source.remove(&source)?;
		for replacement in &replacements {
			let Some(sources) = self.sources_by_replacement.get_mut(replacement) else { continue };
			sources.remove(&source);
			if sources.is_empty() {
				self.sources_by_replacement.remove(replacement);
			}
		}
		Some(replacements)
	}

	fn only_unwanted(&self, tiles: Vec<TileKey>) -> Vec<TileKey> {
		let mut seen = HashSet::new();
		tiles.into_iter().filter(|key| !self.pending.contains(*key) && seen.insert(*key)).collect()
	}
}

fn tiles_overlap(a: TileKey, b: TileKey) -> bool {
	a.grid == b.grid
		&& a.region.min().cmplt(b.region.min() + b.region.size().as_ivec3()).all()
		&& b.region.min().cmplt(a.region.min() + a.region.size().as_ivec3()).all()
}

#[cfg(test)]
mod tests {
	use std::collections::HashSet;

	use bevy::prelude::*;
use tile_data::TileClassId;

	use super::Coverage;
	use crate::types::TileKey;

	fn grid() -> Entity { Entity::from_bits(1) }
	fn tile(lod: u8, min: IVec3) -> TileKey { TileKey::new(grid(), TileClassId(0), lod, min) }
	fn children(parent: TileKey) -> Vec<TileKey> {
		let child_lod = parent.lod - 1;
		let child_size = 1 << child_lod;
		let mut children = Vec::new();
		for x in 0..2 {
			for y in 0..2 {
				for z in 0..2 {
					children.push(tile(child_lod, parent.region.min() + IVec3::new(x, y, z) * child_size));
				}
			}
		}
		children
	}

	#[test]
	fn already_loaded_coarse_tile_does_not_leave_a_fine_tile_waiting() {
		let mut coverage = Coverage::default();
		let old = tile(0, IVec3::ZERO);
		let replacement = tile(1, IVec3::ZERO);
		coverage.set_wanted(old);
		assert!(coverage.set_resolved(old).is_empty());
		let neighbor = tile(1, IVec3::new(2, 0, 0));
		coverage.set_wanted(replacement);
		coverage.set_wanted(neighbor);
		assert!(coverage.set_resolved(replacement).is_empty());
		assert_eq!(coverage.pending.keys().into_iter().collect::<HashSet<_>>(), HashSet::from([neighbor]));

		assert_eq!(coverage.set_unwanted(old), vec![old]);
	}

	#[test]
	fn finer_coverage_removes_coarse_coverage_only_after_every_child_loads() {
		let mut coverage = Coverage::default();
		let old = tile(1, IVec3::ZERO);
		let replacements = children(old);
		coverage.set_wanted(old);
		assert!(coverage.set_resolved(old).is_empty());
		for replacement in &replacements {
			coverage.set_wanted(*replacement);
		}
		assert!(coverage.set_unwanted(old).is_empty());

		for replacement in &replacements[..7] {
			assert!(coverage.set_resolved(*replacement).is_empty());
		}
		assert_eq!(coverage.set_resolved(replacements[7]), vec![old]);
	}

	#[test]
	fn wanting_old_coverage_again_makes_pending_replacements_removable() {
		let mut coverage = Coverage::default();
		let old = tile(1, IVec3::ZERO);
		let replacements = children(old);
		coverage.set_wanted(old);
		assert!(coverage.set_resolved(old).is_empty());
		for replacement in &replacements {
			coverage.set_wanted(*replacement);
		}
		assert!(coverage.set_unwanted(old).is_empty());

		coverage.set_wanted(old);
		assert!(coverage.set_resolved(old).is_empty());
		let removable: HashSet<_> = replacements.iter().flat_map(|replacement| coverage.set_unwanted(*replacement)).collect();
		assert_eq!(removable, replacements.into_iter().collect());
	}

	#[test]
	fn unwanted_pending_coverage_releases_it_and_the_coverage_waiting_on_it() {
		let mut coverage = Coverage::default();
		let old = tile(1, IVec3::ZERO);
		let new = tile(2, IVec3::ZERO);
		coverage.set_wanted(old);
		assert!(coverage.set_resolved(old).is_empty());
		coverage.set_wanted(new);
		assert!(coverage.set_unwanted(old).is_empty());

		assert_eq!(coverage.set_unwanted(new).into_iter().collect::<HashSet<_>>(), HashSet::from([old, new]));
	}
}
