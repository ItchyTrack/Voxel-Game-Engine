use std::collections::{HashMap, HashSet};

use crate::types::TileKey;

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct DependencyRecord {
	pub(crate) source: TileKey,
	pub(crate) replacements: HashSet<TileKey>,
}

impl DependencyRecord {
	pub(crate) fn new(source: TileKey, replacements: HashSet<TileKey>) -> Self {
		Self { source, replacements }
	}
}

#[derive(Clone, Debug, Default)]
pub(crate) struct ReplacementGraph {
	records: HashMap<TileKey, HashSet<TileKey>>,
	waiters_by_replacement: HashMap<TileKey, HashSet<TileKey>>,
}

impl ReplacementGraph {
	pub(crate) fn add_record(&mut self, record: DependencyRecord) {
		let DependencyRecord { source, replacements } = record;
		assert!(!replacements.is_empty(), "replacement dependency records must have at least one replacement");
		assert!(!self.records.contains_key(&source), "replacement dependency record already exists");

		for replacement in &replacements {
			self.waiters_by_replacement.entry(*replacement).or_default().insert(source);
		}
		self.records.insert(source, replacements);
	}

	pub(crate) fn cancel_record(&mut self, source: TileKey) {
		self.detach_record(source);
	}

	pub(crate) fn remove_source(&mut self, source: TileKey) {
		let replacements = self.detach_record(source);
		let Some(waiters) = self.waiters_by_replacement.remove(&source) else {
			return;
		};
		let Some(replacements) = replacements else {
			debug_assert!(false, "replacement graph removed waited-on source without a dependency record");
			return;
		};

		let mut additions = Vec::new();
		for waiter in waiters {
			let Some(waiter_replacements) = self.records.get_mut(&waiter) else {
				continue;
			};
			waiter_replacements.remove(&source);
			for replacement in &replacements {
				if *replacement == waiter {
					continue;
				}
				if waiter_replacements.insert(*replacement) {
					additions.push((*replacement, waiter));
				}
			}
			debug_assert!(
				!waiter_replacements.is_empty(),
				"replacement graph rewrite removed the last dependency for a waiter; source={source:?}, waiter={waiter:?}"
			);
		}
		for (replacement, waiter) in additions {
			self.waiters_by_replacement.entry(replacement).or_default().insert(waiter);
		}
	}

	pub(crate) fn apply_satisfied(&mut self, source: TileKey) -> Vec<TileKey> {
		let Some(waiters) = self.waiters_by_replacement.remove(&source) else {
			return Vec::new();
		};

		let mut ready = Vec::new();
		for waiter in waiters {
			let Some(replacements) = self.records.get_mut(&waiter) else {
				continue;
			};
			replacements.remove(&source);
			if replacements.is_empty() {
				ready.push(waiter);
			}
		}
		for source in &ready {
			self.records.remove(source);
		}
		ready
	}


	fn detach_record(&mut self, source: TileKey) -> Option<HashSet<TileKey>> {
		let replacements = self.records.remove(&source)?;
		for replacement in &replacements {
			let Some(waiters) = self.waiters_by_replacement.get_mut(replacement) else {
				continue;
			};
			waiters.remove(&source);
			if waiters.is_empty() {
				self.waiters_by_replacement.remove(replacement);
			}
		}
		Some(replacements)
	}
}

#[cfg(test)]
mod tests {
	use std::collections::HashSet;

	use bevy::prelude::*;

	use super::{DependencyRecord, ReplacementGraph};
	use crate::types::TileKey;

	fn grid() -> Entity { Entity::from_bits(1) }
	fn tile(lod: u8, min: IVec3) -> TileKey { TileKey { grid: grid(), lod, min } }
	fn set<const N: usize>(tiles: [TileKey; N]) -> HashSet<TileKey> { HashSet::from(tiles) }

	#[test]
	fn cancel_record_only_cancels_the_source_record() {
		let mut graph = ReplacementGraph::default();
		let old = tile(2, IVec3::ZERO);
		let mid = tile(1, IVec3::ZERO);
		let new = tile(0, IVec3::ZERO);
		graph.add_record(DependencyRecord::new(old, set([mid])));
		graph.add_record(DependencyRecord::new(mid, set([new])));

		graph.cancel_record(mid);
		assert!(graph.apply_satisfied(new).is_empty());
		assert_eq!(graph.apply_satisfied(mid), vec![old]);
	}

	#[test]
	fn remove_source_passes_waiters_through_nested_replacements() {
		let mut graph = ReplacementGraph::default();
		let old = tile(2, IVec3::ZERO);
		let mid = tile(1, IVec3::ZERO);
		let new = tile(0, IVec3::ZERO);
		graph.add_record(DependencyRecord::new(old, set([mid])));
		graph.add_record(DependencyRecord::new(mid, set([new])));

		graph.remove_source(mid);
		assert_eq!(graph.apply_satisfied(new), vec![old]);
	}
}
