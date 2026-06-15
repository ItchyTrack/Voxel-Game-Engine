use std::collections::{HashMap, HashSet};

use crate::coverage::CoverageSource;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct SourceVersion {
	pub(crate) source: CoverageSource,
	pub(crate) generation: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct DependencyRecord {
	pub(crate) source: SourceVersion,
	pub(crate) replacements: HashSet<SourceVersion>,
}

impl DependencyRecord {
	pub(crate) fn new(source: SourceVersion, replacements: impl IntoIterator<Item = SourceVersion>) -> Self {
		Self { source, replacements: replacements.into_iter().filter(|replacement| *replacement != source).collect() }
	}
}

/// Tracks replacement dependencies between chunk/tile coverage sources.
///
/// Records are only created for sources that are waiting to retire. Replacement
/// sources are removed from dependency records only when the loader reports that
/// exact [`SourceVersion`] visible/empty. When a record has no remaining
/// replacements, the record's source can be removed.
#[derive(Clone, Debug, Default)]
pub(crate) struct ReplacementGraph {
	current_generations: HashMap<CoverageSource, u64>,
	records: HashMap<SourceVersion, DependencyRecord>,
}

impl ReplacementGraph {
	pub(crate) fn current(&mut self, source: CoverageSource) -> SourceVersion {
		let generation = *self.current_generations.entry(source).or_insert(0);
		SourceVersion { source, generation }
	}

	pub(crate) fn next_generation(&mut self, source: CoverageSource) -> SourceVersion {
		let generation = self.current_generations.entry(source).and_modify(|generation| *generation += 1).or_insert(0);
		SourceVersion { source, generation: *generation }
	}

	pub(crate) fn add_record(&mut self, record: DependencyRecord) {
		assert!(!record.replacements.is_empty(), "replacement dependency records must have at least one replacement");
		assert!(!self.records.contains_key(&record.source), "replacement dependency record already exists");
		self.records.insert(record.source, record);
	}

	/// Applies a visible/empty replacement and returns every retiring source that can
	/// now be removed.
	pub(crate) fn apply_satisfied(&mut self, source: SourceVersion) -> Vec<SourceVersion> {
		let ready = self.remove_replacement(source);
		ready
			.into_iter()
			.filter(|source| self.records.remove(source).is_some())
			.collect()
	}

	fn remove_replacement(&mut self, replacement: SourceVersion) -> Vec<SourceVersion> {
		self.records
			.iter_mut()
			.filter_map(|(&source, record)| {
				record.replacements.remove(&replacement);
				record.replacements.is_empty().then_some(source)
			})
			.collect()
	}
}

#[cfg(test)]
mod tests {
	use bevy::prelude::*;

	use super::*;
	use crate::types::{ChunkKey, TileKey};

	fn grid() -> Entity {
		Entity::from_bits(1)
	}

	fn tile(lod: u8, min: IVec3) -> CoverageSource {
		CoverageSource::Tile(TileKey { grid: grid(), lod, min })
	}

	fn chunk(chunk: IVec3) -> CoverageSource {
		CoverageSource::Chunk(ChunkKey { grid: grid(), chunk })
	}

	#[test]
	fn unresolved_replacements_block_removal_until_all_are_satisfied() {
		let mut graph = ReplacementGraph::default();
		let old = graph.current(tile(2, IVec3::ZERO));
		let a = graph.current(tile(1, IVec3::ZERO));
		let b = graph.current(tile(1, IVec3::new(2, 0, 0)));

		graph.add_record(DependencyRecord::new(old, [a, b]));

		assert!(graph.apply_satisfied(a).is_empty());
		assert_eq!(graph.apply_satisfied(b), vec![old]);
	}

	#[test]
	fn removals_do_not_propagate_transitively() {
		let mut graph = ReplacementGraph::default();
		let old = graph.current(tile(2, IVec3::ZERO));
		let mid = graph.current(tile(1, IVec3::ZERO));
		let new = graph.current(chunk(IVec3::ZERO));

		graph.add_record(DependencyRecord::new(old, [mid]));
		graph.add_record(DependencyRecord::new(mid, [new]));

		assert_eq!(graph.apply_satisfied(new), vec![mid]);
		assert_eq!(graph.apply_satisfied(mid), vec![old]);
	}

	#[test]
	fn later_generation_does_not_satisfy_older_dependency() {
		let mut graph = ReplacementGraph::default();
		let old = graph.current(tile(2, IVec3::ZERO));
		let replacement_gen0 = graph.current(tile(1, IVec3::ZERO));
		graph.add_record(DependencyRecord::new(old, [replacement_gen0]));

		let replacement_gen1 = graph.next_generation(tile(1, IVec3::ZERO));

		assert_ne!(replacement_gen0, replacement_gen1);
		assert!(graph.apply_satisfied(replacement_gen1).is_empty());
		assert_eq!(graph.apply_satisfied(replacement_gen0), vec![old]);
	}

	#[test]
	fn one_satisfied_source_can_unlock_multiple_records() {
		let mut graph = ReplacementGraph::default();
		let old_a = graph.current(tile(2, IVec3::ZERO));
		let old_b = graph.current(tile(2, IVec3::new(4, 0, 0)));
		let replacement = graph.current(tile(1, IVec3::ZERO));

		graph.add_record(DependencyRecord::new(old_a, [replacement]));
		graph.add_record(DependencyRecord::new(old_b, [replacement]));

		let mut removed = graph.apply_satisfied(replacement);
		removed.sort_by_key(|version| match version.source {
			CoverageSource::Chunk(chunk) => chunk.chunk.x,
			CoverageSource::Tile(tile) => tile.min.x,
		});
		assert_eq!(removed, vec![old_a, old_b]);
	}

	#[test]
	#[should_panic(expected = "replacement dependency records must have at least one replacement")]
	fn empty_replacement_sets_are_invalid() {
		let mut graph = ReplacementGraph::default();
		let old = graph.current(tile(2, IVec3::ZERO));

		graph.add_record(DependencyRecord::new(old, []));
	}

	#[test]
	#[should_panic(expected = "replacement dependency record already exists")]
	fn duplicate_records_are_invalid() {
		let mut graph = ReplacementGraph::default();
		let old = graph.current(tile(2, IVec3::ZERO));
		let replacement = graph.current(tile(1, IVec3::ZERO));

		graph.add_record(DependencyRecord::new(old, [replacement]));
		graph.add_record(DependencyRecord::new(old, [replacement]));
	}
}
