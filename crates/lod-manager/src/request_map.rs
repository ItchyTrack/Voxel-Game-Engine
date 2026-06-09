use std::collections::{HashMap, HashSet};

use bevy::prelude::*;
use voxel_data::grid::GridId;

use crate::request_tree::{add_key_to_tree, remove_key_from_tree, LoadedLodEvent, LodRequest, LodRequestGridTree};
use crate::{LodDestination, LodKey};

/// Component owned by a LOD policy source, such as a camera.
///
/// It is both the public request API and the manager-facing delta queue:
/// - `trees` stores the current requested LOD per chunk; unrequested chunks are absent.
/// - `added_delta` records newly added/changed requests for the manager.
/// - `loaded_delta` records GPU LODs that finished loading for this requester.
#[derive(Component, Default, Debug)]
pub struct LodRequestMap {
	trees: HashMap<GridId, LodRequestGridTree>,
	requests: HashMap<LodKey, LodRequest>,
	added_delta: Vec<LodRequest>,
	removed_delta: Vec<LodKey>,
	loaded_delta: Vec<LoadedLodEvent>,
}

impl LodRequestMap {
	pub fn new() -> Self {
		Self::default()
	}

	/// Request a LOD and ask the manager to spawn/cache it as a GPU uploadable entity.
	pub fn request_gpu(&mut self, key: LodKey, priority: f32) {
		self.request(key, priority, LodDestination::Gpu);
	}

	/// Request a LOD without requiring a GPU entity. Useful for warm-up/cache holders.
	pub fn request_cached(&mut self, key: LodKey, priority: f32) {
		self.request(key, priority, LodDestination::CacheOnly);
	}

	pub fn request(&mut self, key: LodKey, priority: f32, destination: LodDestination) {
		let request = LodRequest { key, priority, destination };
		if self.requests.get(&key).is_some_and(|old| *old == request) { return; }

		self.trees.entry(key.grid).or_insert_with(LodRequestGridTree::new);
		if let Some(tree) = self.trees.get_mut(&key.grid) {
			add_key_to_tree(tree, key);
		}
		self.requests.insert(key, request);
		self.added_delta.push(request);
	}

	pub fn release(&mut self, key: LodKey) {
		if self.requests.remove(&key).is_none() { return; }
		if let Some(tree) = self.trees.get_mut(&key.grid) {
			remove_key_from_tree(tree, key);
		}
		self.removed_delta.push(key);
	}

	/// Replace all current requests with `new_requests`, producing minimal deltas.
	pub fn replace_all(&mut self, new_requests: impl IntoIterator<Item = LodRequest>) {
		let mut seen = HashSet::new();
		for request in new_requests {
			seen.insert(request.key);
			self.request(request.key, request.priority, request.destination);
		}
		let stale: Vec<_> = self.requests.keys().copied().filter(|key| !seen.contains(key)).collect();
		for key in stale {
			self.release(key);
		}
	}

	pub fn contains(&self, key: &LodKey) -> bool {
		self.requests.contains_key(key)
	}

	pub fn get(&self, key: &LodKey) -> Option<&LodRequest> {
		self.requests.get(key)
	}

	pub fn tree(&self, grid: GridId) -> Option<&LodRequestGridTree> {
		self.trees.get(&grid)
	}

	pub fn iter(&self) -> impl Iterator<Item = &LodRequest> {
		self.requests.values()
	}

	pub fn added_delta(&self) -> &[LodRequest] {
		&self.added_delta
	}

	pub fn removed_delta(&self) -> &[LodKey] {
		&self.removed_delta
	}

	pub fn loaded_delta(&self) -> &[LoadedLodEvent] {
		&self.loaded_delta
	}

	pub fn drain_added_delta(&mut self) -> Vec<LodRequest> {
		std::mem::take(&mut self.added_delta)
	}

	pub fn drain_removed_delta(&mut self) -> Vec<LodKey> {
		std::mem::take(&mut self.removed_delta)
	}

	pub fn drain_loaded_delta(&mut self) -> Vec<LoadedLodEvent> {
		std::mem::take(&mut self.loaded_delta)
	}

	pub(crate) fn push_loaded(&mut self, event: LoadedLodEvent) {
		self.loaded_delta.push(event);
	}
}
