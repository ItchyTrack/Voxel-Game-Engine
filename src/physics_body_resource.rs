use parry3d::bounding_volume::Aabb;

use crate::{physics::physics_body::{PhysicsBodyGridId, PhysicsBodyId, SubGridId}, resource_manager::{ResourceInfo, ResourceInfoType, ResourceUUID}, state::{State, Task}};

// ------- SubGridGpuState -------
#[derive(Clone)]
pub struct SubGridGpuUploadingState {
	lod_level: f32,
}

pub struct SubGridGpuState {
	on_gpu: bool,
	lod_level: f32,
	currently_uploading: Option<SubGridGpuUploadingState>, // we be replaced with this when upload is done
}

impl SubGridGpuState {
	pub fn new() -> Self {
		Self {
			on_gpu: false,
			lod_level: 0.0,
			currently_uploading: None,
		}
	}
	pub fn on_gpu(&self) -> bool { self.on_gpu }
	pub fn lod_level(&self) -> f32 { self.lod_level }
	pub fn currently_uploading(&self) -> &Option<SubGridGpuUploadingState> { &self.currently_uploading }
}

// ------- PhysicsBodyResource -------
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
	pub fn physics_body_id(&self) -> Option<PhysicsBodyId> { self.physics_body_id }
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
pub struct PhysicsBodyGridResource {
	physics_body_uuid: ResourceUUID,
	physics_body_grid_id: Option<PhysicsBodyGridId>,
	bounds: Option<Aabb>,
	sub_grid_resources: Vec<(Aabb, ResourceUUID)>,
}

impl PhysicsBodyGridResource {
	pub fn new(physics_body_uuid: ResourceUUID) -> Self {
		Self {
			physics_body_uuid: physics_body_uuid,
			physics_body_grid_id: None,
			sub_grid_resources: Vec::new(),
			bounds: None,
		}
	}
	pub fn physics_body_uuid(&self) -> &ResourceUUID { &self.physics_body_uuid }
	pub fn physics_body_grid_id(&self) -> Option<PhysicsBodyGridId> { self.physics_body_grid_id }
	pub fn bounds(&self) -> Option<Aabb> { self.bounds }
	pub fn sub_grid_resources(&self) ->  &Vec<(Aabb, ResourceUUID)> { &self.sub_grid_resources}
}

impl ResourceInfo for PhysicsBodyGridResource {
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

// ------- SubGridResource -------
pub struct SubGridResource {
	physics_body_grid_uuid: ResourceUUID,
	sub_grid_id: Option<SubGridId>,
	bounds: Option<Aabb>,
	gpu_state: SubGridGpuState,
}

impl SubGridResource {
	pub fn new(physics_body_grid_uuid: ResourceUUID) -> Self {
		Self {
			physics_body_grid_uuid: physics_body_grid_uuid,
			sub_grid_id: None,
			bounds: None,
			gpu_state: SubGridGpuState::new(),
		}
	}
	pub fn physics_body_grid_uuid(&self) -> &ResourceUUID { &self.physics_body_grid_uuid }
	pub fn sub_grid_id(&self) -> Option<SubGridId> { self.sub_grid_id }
	pub fn bounds(&self) -> Option<Aabb> { self.bounds }
	pub fn gpu_state(&self) ->  &SubGridGpuState{ &self.gpu_state }
	pub fn request_gpu_state(&mut self, state: &mut State, request: SubGridGpuUploadingState) {
		if let Some(_currently_uploading) = &self.gpu_state.currently_uploading {
			return;
		} else {
			if (self.gpu_state.on_gpu == false) || (self.gpu_state.lod_level - request.lod_level).abs() < 0.2 {
				self.gpu_state.currently_uploading = Some(request.clone());

				if self.sub_grid_id.is_none() {
					unimplemented!("Everything must be loaded!");
				}
				let physics_body_grid_resource = state.resource_manager.resource(self.physics_body_grid_uuid()).unwrap();
				let physics_body_grid_resource_info = match physics_body_grid_resource.info() {
					ResourceInfoType::PhysicsBodyGrid { grid_resource } => grid_resource,
					_ => panic!(),
				};
				if physics_body_grid_resource_info.physics_body_grid_id().is_none() {
					unimplemented!("Everything must be loaded!");
				}
				let physics_body_resource = state.resource_manager.resource(physics_body_grid_resource_info.physics_body_uuid()).unwrap();
				let physics_body_resource_info = match physics_body_resource.info() {
					ResourceInfoType::PhysicsBody { physics_body_resource } => physics_body_resource,
					_ => panic!(),
				};
				if physics_body_resource_info.physics_body_id().is_none() {
					unimplemented!("Everything must be loaded!");
				}

				let physics_body = state.physics_engine.physics_body(physics_body_resource_info.physics_body_id().unwrap()).unwrap();
				let physics_body_grid = physics_body.grid(physics_body_grid_resource_info.physics_body_grid_id().unwrap()).unwrap();
				let _sub_grid = physics_body_grid.sub_grid(self.sub_grid_id().unwrap()).unwrap();
				// sub_grid.v

				let task_queue = state.task_queue.clone();
				tokio::spawn(async move {

					task_queue.lock().unwrap().push_back(Task::new(|state: &mut State| {
						let _ = request;
						let _ = state;
					}));
				});
			}
		}
	}
}

impl ResourceInfo for SubGridResource {
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
