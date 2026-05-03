use std::collections::HashMap;
use std::any::TypeId;

use super::{component_storage::ComponentStorage, system::System, message_queue::MessageQueue};

// Yes, I know this is slow and I do not care. There are so many more things for me to work on.

#[derive(Copy, Clone, Hash, PartialEq, Eq, Debug)]
pub struct EntityId(u32);

pub struct EntityComponentSystem {
	component_sets: HashMap<TypeId, ComponentStorage>,
	message_queues: HashMap<TypeId, MessageQueue>,
	systems: HashMap<String, System>,
	next_entity_id: EntityId,
}

impl EntityComponentSystem {
	pub fn new() -> Self {
		Self {
			component_sets: HashMap::new(),
			message_queues: HashMap::new(),
			systems: HashMap::new(),
			next_entity_id: EntityId(0),
		}
	}

	pub fn add_entity(&mut self) -> EntityId {
		self.next_entity_id.0 += 1;
		return EntityId(self.next_entity_id.0 - 1);
	}

	pub fn remove_entity(&mut self, entity_id: EntityId) {
		for storage in self.component_sets.values_mut() {
			storage.remove(entity_id);
		}
	}

	pub fn add_component_to_entity<T: 'static>(&mut self, entity_id: EntityId, component: T) {
		self.component_sets
			.entry(TypeId::of::<T>())
			.or_insert_with(|| ComponentStorage::new::<T>())
			.get_mut::<T>()
			.insert(entity_id, component);
	}

	pub fn remove_component_from_entity<T: 'static>(&mut self, entity_id: EntityId) {
		if let Some(component_set) = self.component_sets.get_mut(&TypeId::of::<T>()) {
			component_set.get_mut::<T>().remove(&entity_id);
		}
	}

	pub fn get_component<T: 'static>(&self, entity_id: EntityId) -> Option<&T> {
		self.component_sets.get(&TypeId::of::<T>())?.get().get(&entity_id)
	}

	pub fn get_component_mut<T: 'static>(&mut self, entity_id: EntityId) -> Option<&mut T> {
		self.component_sets.get_mut(&TypeId::of::<T>())?.get_mut().get_mut(&entity_id)
	}

	pub fn run_on_single_component<A: 'static, F: Fn(EntityId, &A)>(&self, entity_id: EntityId, f: F){
		if let Some(component_set) = self.component_sets.get(&TypeId::of::<A>()) {
			if let Some(component) = component_set.get().get(&entity_id) {
				f(entity_id, component);
			}
		}
	}

	pub fn run_on_single_component_mut<A: 'static, F: Fn(EntityId, &mut A)>(&mut self, entity_id: EntityId, f: F){
		if let Some(component_set) = self.component_sets.get_mut(&TypeId::of::<A>()) {
			if let Some(component) = component_set.get_mut().get_mut(&entity_id) {
				f(entity_id, component);
			}
		}
	}

	pub fn run_on_single_component_pair<A: 'static, B: 'static, F: Fn(EntityId, &A, &B)>(&self, entity_id: EntityId, f: F){
		if let Some(component_set_a) = self.component_sets.get(&TypeId::of::<A>()) {
			if let Some(component_set_b) = self.component_sets.get(&TypeId::of::<B>()) {
				if let Some(component_a) = component_set_a.get().get(&entity_id) {
					if let Some(component_b) = component_set_b.get().get(&entity_id) {
						f(entity_id, component_a, component_b);
					}
				}
			}
		}
	}

	pub fn run_on_single_component_pair_mut<A: 'static, B: 'static, F: Fn(EntityId, &mut A, &mut B)>(&mut self, entity_id: EntityId, f: F){
		if let [Some(component_set_a), Some(component_set_b)] = self.component_sets.get_disjoint_mut([&TypeId::of::<A>(), &TypeId::of::<B>()]) {
			if let Some(component_a) = component_set_a.get_mut().get_mut(&entity_id) {
				if let Some(component_b) = component_set_b.get_mut().get_mut(&entity_id) {
					f(entity_id, component_a, component_b);
				}
			}
		}
	}

	pub fn run_on_single_component_triple<A: 'static, B: 'static, C: 'static, F: Fn(EntityId, &A, &B, &C)>(&self, entity_id: EntityId, f: F){
		if let Some(component_set_a) = self.component_sets.get(&TypeId::of::<A>()) {
			if let Some(component_set_b) = self.component_sets.get(&TypeId::of::<B>()) {
				if let Some(component_set_c) = self.component_sets.get(&TypeId::of::<C>()) {
					if let Some(component_a) = component_set_a.get().get(&entity_id) {
						if let Some(component_b) = component_set_b.get().get(&entity_id) {
							if let Some(component_c) = component_set_c.get().get(&entity_id) {
								f(entity_id, component_a, component_b, component_c);
							}
						}
					}
				}
			}
		}
	}

	pub fn run_on_single_component_triple_mut<A: 'static, B: 'static, C: 'static, F: Fn(EntityId, &mut A, &mut B, &mut C)>(&mut self, entity_id: EntityId, f: F){
		if let [Some(component_set_a), Some(component_set_b), Some(component_set_c)] = self.component_sets.get_disjoint_mut([&TypeId::of::<A>(), &TypeId::of::<B>(), &TypeId::of::<C>()]) {
			if let Some(component_a) = component_set_a.get_mut().get_mut(&entity_id) {
				if let Some(component_b) = component_set_b.get_mut().get_mut(&entity_id) {
					if let Some(component_c) = component_set_c.get_mut().get_mut(&entity_id) {
						f(entity_id, component_a, component_b, component_c);
					}
				}
			}
		}
	}

	pub fn run_on_components<A: 'static, F: Fn(EntityId, &A)>(&self, f: F){
		if let Some(component_set) = self.component_sets.get(&TypeId::of::<A>()) {
			for (entity_id, component) in component_set.get() {
				f(*entity_id, component);
			}
		}
	}

	pub fn run_on_components_mut<A: 'static, F: FnMut(EntityId, &mut A)>(&mut self, f: &mut F){
		if let Some(component_set) = self.component_sets.get_mut(&TypeId::of::<A>()) {
			for (entity_id, component) in component_set.get_mut() {
				f(*entity_id, component);
			}
		}
	}

	pub fn run_on_components_pair<A: 'static, B: 'static, F: Fn(EntityId, &A, &B)>(&self, f: &F){
		if let Some(component_set_a) = self.component_sets.get(&TypeId::of::<A>()) {
			if let Some(component_set_b) = self.component_sets.get(&TypeId::of::<B>()) {
				let component_set_b = component_set_b.get();
				for (entity_id, component_a) in component_set_a.get() {
					if let Some(component_b) = component_set_b.get(entity_id) {
						f(*entity_id, component_a, component_b);
					}
				}
			}
		}
	}

	pub fn run_on_components_pair_mut<A: 'static, B: 'static, F: FnMut(EntityId, &mut A, &mut B)>(&mut self, f: &mut F){
		if let [Some(component_set_a), Some(component_set_b)] = self.component_sets.get_disjoint_mut([&TypeId::of::<A>(), &TypeId::of::<B>()]) {
			let component_set_b = component_set_b.get_mut();
			for (entity_id, component_a) in component_set_a.get_mut() {
				if let Some(component_b) = component_set_b.get_mut(entity_id) {
					f(*entity_id, component_a, component_b);
				}
			}
		}
	}

	pub fn run_on_components_tripl<A: 'static, B: 'static, C: 'static, F: Fn(EntityId, &A, &B, &C)>(&self, f: F){
		if let Some(component_set_a) = self.component_sets.get(&TypeId::of::<A>()) {
			if let Some(component_set_b) = self.component_sets.get(&TypeId::of::<B>()) {
				if let Some(component_set_c) = self.component_sets.get(&TypeId::of::<C>()) {
					let component_set_b = component_set_b.get();
					let component_set_c = component_set_c.get();
					for (entity_id, component_a) in component_set_a.get() {
						if let Some(component_b) = component_set_b.get(entity_id) {
							if let Some(component_c) = component_set_c.get(entity_id) {
								f(*entity_id, component_a, component_b, component_c);
							}
						}
					}
				}
			}
		}
	}

	pub fn run_on_components_tripl_mut<A: 'static, B: 'static, C: 'static, F: FnMut(EntityId, &mut A, &mut B, &mut C)>(&mut self, f: &mut F){
		if let [Some(component_set_a), Some(component_set_b), Some(component_set_c)] = self.component_sets.get_disjoint_mut([&TypeId::of::<A>(), &TypeId::of::<B>(), &TypeId::of::<C>()]) {
			let component_set_b = component_set_b.get_mut();
			let component_set_c = component_set_c.get_mut();
			for (entity_id, component_a) in component_set_a.get_mut() {
				if let Some(component_b) = component_set_b.get_mut(entity_id) {
					if let Some(component_c) = component_set_c.get_mut(entity_id) {
						f(*entity_id, component_a, component_b, component_c);
					}
				}
			}
		}
	}

	pub fn add_system<F>(&mut self, name: impl Into<String>, func: F)
	where
		F: FnMut(&mut EntityComponentSystem) + 'static,
	{
		let name = name.into();
		self.systems.insert(name.clone(), System::new(name, func));
	}

	pub fn run_system(&mut self, name: &str) {
		if let Some(mut system) = self.systems.remove(name) {
			system.run(self);
			self.systems.insert(system.name().to_string(), system);
		}
	}

	pub fn post_message<T: 'static>(&mut self, message: T) {
		self.message_queues
			.entry(TypeId::of::<T>())
			.or_insert_with(|| MessageQueue::new::<T>())
			.push(message);
	}

	pub fn handle_messages<T: 'static, F: FnMut(&mut Self, T)>(&mut self, mut f: F) {
		let Some(mut queue) = self.message_queues.remove(&TypeId::of::<T>()) else { return };
		while let Some(msg) = queue.pop::<T>() {
			f(self, msg);
		}
		self.message_queues.entry(TypeId::of::<T>()).or_insert(queue);
	}

	pub fn peek_messages<T: 'static, F: Fn(&T)>(&self, f: F) {
		if let Some(queue) = self.message_queues.get(&TypeId::of::<T>()) {
			for msg in queue.iter::<T>() {
				f(msg);
			}
		}
	}

	pub fn clear_messages<T: 'static>(&mut self) {
		if let Some(queue) = self.message_queues.get_mut(&TypeId::of::<T>()) {
			queue.clear();
		}
	}
}
