use std::collections::HashMap;
use std::any::TypeId;

use super::{component_storage::ComponentStorage};

// Yes, I know this is slow and I do not care. There are so many more things for me to work on.

pub struct EntityComponentSystem {
	component_sets: HashMap<TypeId, ComponentStorage>,
	next_entity_id: u32,
}

impl EntityComponentSystem {
	pub fn new() -> Self {
		Self {
			component_sets: HashMap::new(),
			next_entity_id: 0,
		}
	}

	pub fn add_entity(&mut self) -> u32 {
		self.next_entity_id += 1;
		return self.next_entity_id - 1;
	}

	pub fn remove_entity(&mut self, entity_id: u32) {
		for storage in self.component_sets.values_mut() {
			storage.remove(entity_id);
		}
	}

	pub fn add_component_to_entity<T: 'static>(&mut self, entity_id: u32, component: T) {
		let component_set = self.component_sets.entry(TypeId::of::<T>()).or_insert_with(|| ComponentStorage::new::<T>());
		component_set.get_mut::<T>().insert(entity_id, component);
	}

	pub fn remove_component_from_entity<T: 'static>(&mut self, entity_id: u32) {
		if let Some(component_set) = self.component_sets.get_mut(&TypeId::of::<T>()) {
			component_set.get_mut::<T>().remove(&entity_id);
		}
	}

	pub fn get_component<T: 'static>(&self, entity_id: u32) -> Option<&T> {
		self.component_sets.get(&TypeId::of::<T>())?.get().get(&entity_id)
	}

	pub fn get_component_mut<T: 'static>(&mut self, entity_id: u32) -> Option<&mut T> {
		self.component_sets.get_mut(&TypeId::of::<T>())?.get_mut().get_mut(&entity_id)
	}

	pub fn run_on_single_component<A: 'static, F: Fn(u32, &A)>(&self, entity_id: u32, f: F){
		if let Some(component_set) = self.component_sets.get(&TypeId::of::<A>()) {
			if let Some(component) = component_set.get().get(&entity_id) {
				f(entity_id, component);
			}
		}
	}

	pub fn run_on_single_component_mut<A: 'static, F: Fn(u32, &mut A)>(&mut self, entity_id: u32, f: F){
		if let Some(component_set) = self.component_sets.get_mut(&TypeId::of::<A>()) {
			if let Some(component) = component_set.get_mut().get_mut(&entity_id) {
				f(entity_id, component);
			}
		}
	}

	pub fn run_on_single_component_pair<A: 'static, B: 'static, F: Fn(u32, &A, &B)>(&self, entity_id: u32, f: F){
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

	pub fn run_on_single_component_pair_mut<A: 'static, B: 'static, F: Fn(u32, &mut A, &mut B)>(&mut self, entity_id: u32, f: F){
		if let [Some(component_set_a), Some(component_set_b)] = self.component_sets.get_disjoint_mut([&TypeId::of::<A>(), &TypeId::of::<B>()]) {
			if let Some(component_a) = component_set_a.get_mut().get_mut(&entity_id) {
				if let Some(component_b) = component_set_b.get_mut().get_mut(&entity_id) {
					f(entity_id, component_a, component_b);
				}
			}
		}
	}

	pub fn run_on_components<A: 'static, F: Fn(u32, &A)>(&self, f: F){
		if let Some(component_set) = self.component_sets.get(&TypeId::of::<A>()) {
			for (entity_id, component) in component_set.get() {
				f(*entity_id, component);
			}
		}
	}

	pub fn run_on_components_mut<A: 'static, F: Fn(u32, &mut A)>(&mut self, f: F){
		if let Some(component_set) = self.component_sets.get_mut(&TypeId::of::<A>()) {
			for (entity_id, component) in component_set.get_mut() {
				f(*entity_id, component);
			}
		}
	}

	pub fn run_on_components_pair<A: 'static, B: 'static, F: Fn(u32, &A, &B)>(&self, f: F){
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

	pub fn run_on_components_pair_mut<A: 'static, B: 'static, F: Fn(u32, &mut A, &mut B)>(&mut self, f: F){
		if let [Some(component_set_a), Some(component_set_b)] = self.component_sets.get_disjoint_mut([&TypeId::of::<A>(), &TypeId::of::<B>()]) {
			let component_set_b = component_set_b.get_mut();
			for (entity_id, component_a) in component_set_a.get_mut() {
				if let Some(component_b) = component_set_b.get_mut(entity_id) {
					f(*entity_id, component_a, component_b);
				}
			}
		}
	}
}
