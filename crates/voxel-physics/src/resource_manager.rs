use std::{collections::HashMap};

use uuid::Uuid;

use super::physics_body_resource::{GridResource, PhysicsBodyResource};

#[derive(Clone, Eq, Hash, PartialEq)]
pub struct ResourceUUID(pub Uuid);

impl ResourceUUID {
	pub fn generate() -> Self {
		Self(Uuid::new_v4())
	}
}

pub trait ResourceInfo {
	fn deserialize(&mut self, raw: &String);
	fn serialize(&mut self) -> String;
	fn set_empty(&mut self) -> bool; // returns if it was successfully set empty
}

pub enum ResourceInfoType {
	PhysicsBody { physics_body_resource: PhysicsBodyResource },
	Grid { grid_resource: GridResource },
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
	info: ResourceInfoType,
	network_state: NetworkState,
}

impl Resource {
	pub fn new(on_disk: bool, loaded: bool, dependencies: Vec<String>, info: ResourceInfoType) -> Self {
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
	pub fn info(&self) -> &ResourceInfoType {
		&self.info
	}
	pub fn info_mut(&mut self) -> &mut ResourceInfoType {
		&mut self.info
	}
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
	pub fn create_resource(&mut self, uuid: ResourceUUID, on_disk: bool, loaded: bool, dependencies: Vec<String>, resource_info: ResourceInfoType) -> Option<&mut Resource> {
		match self.resources.entry(uuid) {
			std::collections::hash_map::Entry::Occupied(_occupied_entry) => {
				println!("ERROR: created resource with existing uuid");
				return None;
			},
			std::collections::hash_map::Entry::Vacant(vacant_entry) => {
				return Some(vacant_entry.insert(Resource::new(on_disk, loaded, dependencies, resource_info)));
			},
		}
	}
	pub fn create_resource_blank(&mut self, uuid: ResourceUUID, resource_info: ResourceInfoType) -> Option<&mut Resource> {
		self.create_resource(uuid, false, true, vec![], resource_info)
	}
}
