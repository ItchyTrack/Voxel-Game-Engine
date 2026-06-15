use std::collections::{HashMap, HashSet};

use crate::coverage::CoverageSource;

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct DependencyRecord {
	pub(crate) source: CoverageSource,
	pub(crate) replacements: HashSet<CoverageSource>,
}

impl DependencyRecord {
	pub(crate) fn new(source: CoverageSource, replacements: impl IntoIterator<Item = CoverageSource>) -> Self {
		Self { source, replacements: replacements.into_iter().filter(|replacement| *replacement != source).collect() }
	}
}

#[derive(Clone, Debug, Default)]
pub(crate) struct ReplacementGraph {
	records: HashMap<CoverageSource, DependencyRecord>,
}

impl ReplacementGraph {
	pub(crate) fn add_record(&mut self, record: DependencyRecord) {
		assert!(!record.replacements.is_empty(), "replacement dependency records must have at least one replacement");
		assert!(!self.records.contains_key(&record.source), "replacement dependency record already exists");
		self.records.insert(record.source, record);
	}

	pub(crate) fn cancel_record(&mut self, source: CoverageSource) {
		self.records.remove(&source);
	}

	pub(crate) fn remove_source(&mut self, source: CoverageSource) {
		self.records.remove(&source);
		for record in self.records.values_mut() {
			record.replacements.remove(&source);
		}
	}

	pub(crate) fn apply_satisfied(&mut self, source: CoverageSource) -> Vec<CoverageSource> {
		let ready: Vec<_> = self
			.records
			.iter_mut()
			.filter_map(|(&retiring, record)| {
				record.replacements.remove(&source);
				record.replacements.is_empty().then_some(retiring)
			})
			.collect();
		for source in &ready {
			self.records.remove(source);
		}
		ready
	}
}

#[cfg(test)]
mod tests {
	use bevy::prelude::*;

	use super::*;
	use crate::types::{ChunkKey, TileKey};

	fn grid() -> Entity { Entity::from_bits(1) }
	fn tile(lod: u8, min: IVec3) -> CoverageSource { CoverageSource::Tile(TileKey { grid: grid(), lod, min }) }
	fn chunk(chunk: IVec3) -> CoverageSource { CoverageSource::Chunk(ChunkKey { grid: grid(), chunk }) }

	#[test]
	fn unresolved_replacements_block_removal_until_all_are_satisfied() {
		let mut graph = ReplacementGraph::default();
		let old = tile(2, IVec3::ZERO);
		let a = tile(1, IVec3::ZERO);
		let b = tile(1, IVec3::new(2, 0, 0));
		graph.add_record(DependencyRecord::new(old, [a, b]));
		assert!(graph.apply_satisfied(a).is_empty());
		assert_eq!(graph.apply_satisfied(b), vec![old]);
	}

	#[test]
	fn removals_do_not_propagate_transitively() {
		let mut graph = ReplacementGraph::default();
		let old = tile(2, IVec3::ZERO);
		let mid = tile(1, IVec3::ZERO);
		let new = chunk(IVec3::ZERO);
		graph.add_record(DependencyRecord::new(old, [mid]));
		graph.add_record(DependencyRecord::new(mid, [new]));
		assert_eq!(graph.apply_satisfied(new), vec![mid]);
		assert_eq!(graph.apply_satisfied(mid), vec![old]);
	}

	#[test]
	fn cancel_record_allows_same_source_to_retire_again() {
		let mut graph = ReplacementGraph::default();
		let old = tile(2, IVec3::ZERO);
		let replacement = tile(1, IVec3::ZERO);
		graph.add_record(DependencyRecord::new(old, [replacement]));
		graph.cancel_record(old);
		graph.add_record(DependencyRecord::new(old, [replacement]));
		assert_eq!(graph.apply_satisfied(replacement), vec![old]);
	}

	#[test]
	fn one_satisfied_source_can_unlock_multiple_records() {
		let mut graph = ReplacementGraph::default();
		let old_a = tile(2, IVec3::ZERO);
		let old_b = tile(2, IVec3::new(4, 0, 0));
		let replacement = tile(1, IVec3::ZERO);
		graph.add_record(DependencyRecord::new(old_a, [replacement]));
		graph.add_record(DependencyRecord::new(old_b, [replacement]));
		let mut removed = graph.apply_satisfied(replacement);
		removed.sort_by_key(|source| match source { CoverageSource::Chunk(chunk) => chunk.chunk.x, CoverageSource::Tile(tile) => tile.min.x });
		assert_eq!(removed, vec![old_a, old_b]);
	}

	#[test]
	#[should_panic(expected = "replacement dependency records must have at least one replacement")]
	fn empty_replacement_sets_are_invalid() {
		ReplacementGraph::default().add_record(DependencyRecord::new(tile(2, IVec3::ZERO), []));
	}

	#[test]
	#[should_panic(expected = "replacement dependency record already exists")]
	fn duplicate_records_are_invalid() {
		let mut graph = ReplacementGraph::default();
		let old = tile(2, IVec3::ZERO);
		let replacement = tile(1, IVec3::ZERO);
		graph.add_record(DependencyRecord::new(old, [replacement]));
		graph.add_record(DependencyRecord::new(old, [replacement]));
	}
}
