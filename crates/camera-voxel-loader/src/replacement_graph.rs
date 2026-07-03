use std::collections::{HashMap, HashSet};

use crate::types::TileKey;

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct DependencyRecord {
	pub(crate) source: TileKey,
	pub(crate) replacements: HashSet<TileKey>,
}

impl DependencyRecord {
	pub(crate) fn new(source: TileKey, replacements: impl IntoIterator<Item = TileKey>) -> Self {
		Self { source, replacements: replacements.into_iter().filter(|replacement| *replacement != source).collect() }
	}
}

#[derive(Clone, Debug, Default)]
pub(crate) struct ReplacementGraph {
	records: HashMap<TileKey, HashSet<TileKey>>,
}

impl ReplacementGraph {
	pub(crate) fn add_record(&mut self, record: DependencyRecord) {
		assert!(!record.replacements.is_empty(), "replacement dependency records must have at least one replacement");
		assert!(!self.records.contains_key(&record.source), "replacement dependency record already exists");
		self.records.insert(record.source, record.replacements);
	}

	pub(crate) fn cancel_record(&mut self, source: TileKey) {
		self.records.remove(&source);
	}

	pub(crate) fn remove_source(&mut self, source: TileKey) {
		self.records.remove(&source);
		for replacements in self.records.values_mut() {
			replacements.remove(&source);
		}
	}

	pub(crate) fn apply_satisfied(&mut self, source: TileKey) -> Vec<TileKey> {
		let ready: Vec<_> = self
			.records
			.iter_mut()
			.filter_map(|(&retiring, replacements)| {
				replacements.remove(&source);
				replacements.is_empty().then_some(retiring)
			})
			.collect();
		for source in &ready {
			self.records.remove(source);
		}
		ready
	}
}
