use std::collections::HashMap;

pub trait ResourceInfo { }

// the file location will be its UUID
pub struct Resource {
	on_disk: bool,
	loaded: bool,
	dependencies: Vec<String>, // I dont know if we need this
	info: Box<dyn ResourceInfo>,
}

impl Resource {
	fn new(on_disk: bool, loaded: bool, dependencies: Vec<String>, info: Box<dyn ResourceInfo>) -> Self {
		Self {
			on_disk,
			loaded,
			dependencies,
			info,
		}
	}
	pub fn on_disk(&self) -> bool {
		self.on_disk
	}
	pub fn loaded(&self) -> bool {
		self.loaded
	}
	pub fn dependencies(&self) -> &Vec<String> {
		&self.dependencies
	}
	pub fn info(&self) -> &dyn ResourceInfo {
		self.info.as_ref()
	}
}

pub struct ResourceManager {
	resources: HashMap<String, Resource>,
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
	pub fn sync_disk(&self, _uuid: String) {
		println!("We are not saving anything yet!");
	}
	pub fn reload_all_resource(&self) {
		println!("We are not saving anything yet!");
	}
	pub fn reload_resource(&self, _uuid: String) {
		println!("We are not saving anything yet!");
	}
	pub fn resource(&self, uuid: &String) -> Option<&Resource> {
		self.resources.get(uuid)
	}
	pub fn resources(&self) -> &HashMap<String, Resource> {
		&self.resources
	}
}
