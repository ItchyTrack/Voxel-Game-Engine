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
		self.remove_record(source);
	}

	pub(crate) fn remove_source(&mut self, source: TileKey) {
		self.remove_record(source);

		let Some(waiters) = self.waiters_by_replacement.remove(&source) else {
			return;
		};
		for waiter in waiters {
			if let Some(replacements) = self.records.get_mut(&waiter) {
				replacements.remove(&source);
			}
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

	fn remove_record(&mut self, source: TileKey) {
		let Some(replacements) = self.records.remove(&source) else {
			return;
		};

		for replacement in replacements {
			let Some(waiters) = self.waiters_by_replacement.get_mut(&replacement) else {
				continue;
			};
			waiters.remove(&source);
			if waiters.is_empty() {
				self.waiters_by_replacement.remove(&replacement);
			}
		}
	}
}
