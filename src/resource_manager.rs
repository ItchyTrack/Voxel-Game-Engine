use std::collections::HashMap;

use parry3d::bounding_volume::Aabb;

use crate::{physics::physics_body::PhysicsBodyId};

#[derive(Eq, Hash, PartialEq)]
pub struct UUID(pub String);

#[derive(Eq, Hash, PartialEq)]
pub struct ResourceUUID(pub UUID);

pub trait ResourceInfo {
	fn deserialize(&mut self, raw: &String);
	fn serialize(&mut self) -> String;
	fn set_empty(&mut self) -> bool; // returns if it was successfully set empty
}

pub struct NetworkState {
	// todo
}

impl NetworkState {
	pub fn new() -> Self {
		Self {

		}
	}
}

// the file location will be its UUID
pub struct Resource {
	on_disk: bool,
	loaded: bool,
	directly_requested: bool,
	dependents_count: u32,
	dependencies: Vec<String>, // I dont know if we need this
	info: Box<dyn ResourceInfo>,
	network_state: NetworkState,
}

impl Resource {
	pub fn new(on_disk: bool, loaded: bool, dependencies: Vec<String>, info: Box<dyn ResourceInfo>) -> Self {
		Self {
			on_disk,
			loaded,
			directly_requested: false,
			dependents_count: 0,
			dependencies,
			info,
			network_state: NetworkState::new(),
		}
	}
	pub fn on_disk(&self) -> bool {
		self.on_disk
	}
	pub fn loaded(&self) -> bool {
		self.loaded
	}
	pub fn directly_requested(&self) -> bool {
		self.directly_requested
	}
	pub fn dependencies(&self) -> &Vec<String> {
		&self.dependencies
	}
	pub fn info(&self) -> &dyn ResourceInfo {
		self.info.as_ref()
	}
	// pub fn info_type<T: 'static>(&self) -> &T {
	// 	self.info

	// 		.downcast_ref::<ComponentStorageImpl<T>>()
	// 		.map(|s| &s.storage).unwrap()
	// }
	pub fn network_state(&self) -> &NetworkState {
		&self.network_state
	}
}

pub struct ResourceManager {
	resources: HashMap<ResourceUUID, Resource>,
	save_folder: Option<String>,
}

impl ResourceManager {
	pub fn new() -> Self {
		Self {
			resources: HashMap::new(),
			save_folder: None,
		}
	}
	pub fn save_folder(&self) -> Option<&String> {
		self.save_folder.as_ref()
	}
	pub fn set_save_folder(&mut self, save_folder: Option<String>) {
		if self.save_folder == save_folder { return; }
		if self.save_folder.is_some() {
			for (_uuid, resource) in &mut self.resources {
				resource.on_disk = false;
				if !resource.loaded {
					println!("Warning: Save folder was moved but all resources were not loaded. This may break the save!");
				}
			}
		}
		self.save_folder = save_folder;
		if let Some(save_folder) = self.save_folder.as_ref() {
			println!("Save folder set to {}", save_folder);
		} else {
			println!("Save folder set to NONE");
		}
	}
	pub fn sync_whole_disk(&self) {
		println!("We are not saving anything yet!");
	}
	pub fn sync_disk(&self, _uuid: ResourceUUID) {
		println!("We are not saving anything yet!");
	}
	pub fn reload_all_resource(&self) {
		println!("We are not saving anything yet!");
	}
	pub fn reload_resource(&self, _uuid: ResourceUUID) {
		println!("We are not saving anything yet!");
	}
	pub fn resource(&self, uuid: &ResourceUUID) -> Option<&Resource> {
		self.resources.get(uuid)
	}
	pub fn resources(&self) -> &HashMap<ResourceUUID, Resource> {
		&self.resources
	}
}

pub struct PhysicsBodyResource {
	physics_body_id: Option<PhysicsBodyId>,
	bounds: Option<Aabb>,
	grid_resources: Vec<(Aabb, ResourceUUID)>,
	constraints: Vec<ResourceUUID>, // unused
}

impl PhysicsBodyResource {
	pub fn new() -> Self {
		Self {
			physics_body_id: None,
			grid_resources: Vec::new(),
			constraints: Vec::new(),
			bounds: None,
		}
	}
	pub fn physics_body_id(&self) -> Option<PhysicsBodyId> {
		self.physics_body_id
	}
	pub fn bounds(&self) -> Option<Aabb> {
		self.bounds
	}
	pub fn grid_resources(&self) ->  &Vec<(Aabb, ResourceUUID)> {
		&self.grid_resources
	}
	pub fn constraints(&self) ->  &Vec<ResourceUUID> {
		&self.constraints
	}
}

impl ResourceInfo for PhysicsBodyResource {
	fn deserialize(&mut self, _raw: &String) {
		unimplemented!()
	}
	fn serialize(&mut self) -> String {
		unimplemented!()
	}
	fn set_empty(&mut self) -> bool {
		unimplemented!()
	}
}
