use std::collections::HashMap;

use bevy::prelude::*;

use crate::LodKey;

/// Component attached to every GPU LOD entity spawned by the manager.
///
/// Systems that are no longer requesting this LOD but still need it temporarily
/// should increment this counter before changing their request map, then decrement
/// it when the old LOD can be dropped.
#[derive(Component, Default, Debug, Clone, Copy)]
pub struct LodRetainCount(pub u32);

impl LodRetainCount {
	pub fn retain(&mut self) {
		self.0 = self.0.saturating_add(1);
	}

	pub fn release(&mut self) {
		self.0 = self.0.saturating_sub(1);
	}

	pub fn is_retained(&self) -> bool {
		self.0 > 0
	}
}

/// Manager cache of loaded GPU LOD entities.
#[derive(Resource, Default)]
pub struct LoadedLods {
	by_key: HashMap<LodKey, Entity>,
}

impl LoadedLods {
	pub fn get(&self, key: &LodKey) -> Option<Entity> {
		self.by_key.get(key).copied()
	}

	pub fn contains(&self, key: &LodKey) -> bool {
		self.by_key.contains_key(key)
	}

	pub fn iter(&self) -> impl Iterator<Item = (&LodKey, &Entity)> {
		self.by_key.iter()
	}

	pub(crate) fn insert(&mut self, key: LodKey, entity: Entity) -> Option<Entity> {
		self.by_key.insert(key, entity)
	}

	pub(crate) fn remove(&mut self, key: &LodKey) -> Option<Entity> {
		self.by_key.remove(key)
	}

	pub(crate) fn keys(&self) -> impl Iterator<Item = &LodKey> {
		self.by_key.keys()
	}
}
