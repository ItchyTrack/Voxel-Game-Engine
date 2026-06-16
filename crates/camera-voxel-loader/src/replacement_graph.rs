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
