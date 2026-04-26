use std::any::{Any, TypeId};

use crate::world::sparse_set::SparseSet;
use super::entity_component_system::EntityId;

trait ComponentStorageDyn {
	fn as_any(&self) -> &dyn Any;
	fn as_any_mut(&mut self) -> &mut dyn Any;
	fn remove(&mut self, entity_id: EntityId);
	fn held_type_id(&self) -> TypeId;
	fn type_name(&self) -> &'static str;
}

struct ComponentStorageImpl<T> {
	storage: SparseSet<EntityId, T>,
	type_id: TypeId,
}

impl<T: 'static> ComponentStorageDyn for ComponentStorageImpl<T> {
	fn as_any(&self) -> &dyn Any {
		self
	}

	fn as_any_mut(&mut self) -> &mut dyn Any {
		self
	}

	fn remove(&mut self, entity_id: EntityId) {
		self.storage.remove(&entity_id);
	}

	fn held_type_id(&self) -> TypeId {
		TypeId::of::<T>()
	}

	fn type_name(&self) -> &'static str {
		std::any::type_name::<T>()
	}
}

pub struct ComponentStorage {
	internal: Box<dyn ComponentStorageDyn>,
}

impl ComponentStorage {
	pub fn new<T: 'static>() -> Self {
		Self {
			internal: Box::new(ComponentStorageImpl::<T> {
				storage: SparseSet::new(),
				type_id: TypeId::of::<T>(),
			}),
		}
	}

	pub fn get<T: 'static>(&self) -> &SparseSet<EntityId, T> {
		assert!(TypeId::of::<T>() == self.internal.held_type_id());
		self.internal
			.as_any()
			.downcast_ref::<ComponentStorageImpl<T>>()
			.map(|s| &s.storage).unwrap()
	}

	pub fn get_mut<T: 'static>(&mut self) -> &mut SparseSet<EntityId, T> {
		assert!(TypeId::of::<T>() == self.internal.held_type_id());
		self.internal
			.as_any_mut()
			.downcast_mut::<ComponentStorageImpl<T>>()
			.map(|s| &mut s.storage).unwrap()
	}

	pub fn remove(&mut self, entity_id: EntityId) {
		self.internal.remove(entity_id);
	}

	pub fn held_type_id(&self) -> TypeId {
		self.internal.held_type_id()
	}

	pub fn type_name(&self) -> &'static str {
		self.internal.type_name()
	}
}