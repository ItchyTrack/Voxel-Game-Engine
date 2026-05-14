use parry3d::bounding_volume::Aabb;

use super::{physics_body::PhysicsBodyId, grid::GridId};
use super::resource_manager::{ResourceInfo, ResourceUUID};

// ------- PhysicsBodyResource -------
pub struct PhysicsBodyResource {
	physics_body_id: Option<PhysicsBodyId>,
	physics_body_uuid: ResourceUUID,
	bounds: Option<Aabb>,
	grid_resources: Vec<(Aabb, ResourceUUID)>,
	constraints: Vec<ResourceUUID>, // unused
}

impl PhysicsBodyResource {
	pub fn new(physics_body_uuid: ResourceUUID) -> Self {
		Self {
			physics_body_id: None,
			physics_body_uuid,
			grid_resources: Vec::new(),
			constraints: Vec::new(),
			bounds: None,
		}
	}
	pub fn physics_body_id(&self) -> Option<PhysicsBodyId> { self.physics_body_id }
	pub fn set_physics_body_id(&mut self, physics_body_id: Option<PhysicsBodyId>) {
		self.physics_body_id = physics_body_id;
	}
	pub fn bounds(&self) -> Option<Aabb> { self.bounds }
	pub fn grid_resources(&self) ->  &Vec<(Aabb, ResourceUUID)> { &self.grid_resources }
	pub fn constraints(&self) ->  &Vec<ResourceUUID> { &self.constraints }
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

// ------- GridResource -------
pub struct GridResource {
	physics_body_uuid: ResourceUUID,
	physics_body_grid_uuid: ResourceUUID,
	physics_body_grid_id: Option<GridId>,
	bounds: Option<Aabb>,
	sub_grid_resources: Vec<(Aabb, ResourceUUID)>,
}

impl GridResource {
	pub fn new(physics_body_grid_uuid: ResourceUUID, physics_body_uuid: ResourceUUID) -> Self {
		Self {
			physics_body_uuid,
			physics_body_grid_uuid,
			physics_body_grid_id: None,
			sub_grid_resources: Vec::new(),
			bounds: None,
		}
	}
	pub fn physics_body_uuid(&self) -> &ResourceUUID { &self.physics_body_uuid }
	pub fn set_physics_body_grid_id(&mut self, physics_body_grid_id: Option<GridId>) {
		self.physics_body_grid_id = physics_body_grid_id;
	}
	pub fn physics_body_grid_id(&self) -> Option<GridId> { self.physics_body_grid_id }
	pub fn bounds(&self) -> Option<Aabb> { self.bounds }
	pub fn sub_grid_resources(&self) ->  &Vec<(Aabb, ResourceUUID)> { &self.sub_grid_resources}
}

impl ResourceInfo for GridResource {
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
