use std::cell::Cell;

use parry3d::bounding_volume::Aabb;

use crate::gpu_objects::gpu_grid_tree::make_gpu_grid_tree;
use crate::physics::{physics_body::{PhysicsBodyGridId, PhysicsBodyId, SubGridId}, physics_engine::{PhysicsEngine}};
use crate::resource_manager::{ResourceInfo, ResourceInfoType, ResourceManager, ResourceUUID};
use crate::state::{AsyncTaskPriorityQueue, PriorityTask, State, Task, TaskQueue};

// ------- SubGridGpuState -------
#[derive(Clone, Copy, Debug)]
pub struct SubGridGpuUploadingState {
	pub lod_level: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct SubGridGpuState {
	on_gpu: bool,
	lod_level: f32,
	tree_id: u32,
	voxels_id: u32,
	currently_uploading: Option<SubGridGpuUploadingState>, // we be replaced with this when upload is done
}

impl SubGridGpuState {
	pub fn new() -> Self {
		Self {
			on_gpu: false,
			lod_level: 0.0,
			tree_id: 0,
			voxels_id: 0,
			currently_uploading: None,
		}
	}
	pub fn on_gpu(&self) -> bool { self.on_gpu }
	pub fn lod_level(&self) -> f32 { self.lod_level }
	pub fn tree_id(&self) -> u32 { self.tree_id }
	pub fn voxels_id(&self) -> u32 { self.voxels_id }
	pub fn currently_uploading(&self) -> &Option<SubGridGpuUploadingState> { &self.currently_uploading }
}

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
pub struct PhysicsBodyGridResource {
	physics_body_uuid: ResourceUUID,
	physics_body_grid_uuid: ResourceUUID,
	physics_body_grid_id: Option<PhysicsBodyGridId>,
	bounds: Option<Aabb>,
	sub_grid_resources: Vec<(Aabb, ResourceUUID)>,
}

impl PhysicsBodyGridResource {
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
	pub fn set_physics_body_grid_id(&mut self, physics_body_grid_id: Option<PhysicsBodyGridId>) {
		self.physics_body_grid_id = physics_body_grid_id;
	}
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
	sub_grid_uuid: ResourceUUID,
	sub_grid_id: Option<SubGridId>,
	bounds: Option<Aabb>,
	gpu_state: Cell<SubGridGpuState>,
}

impl SubGridResource {
	pub fn new(sub_grid_uuid: ResourceUUID, physics_body_grid_uuid: ResourceUUID) -> Self {
		Self {
			physics_body_grid_uuid: physics_body_grid_uuid,
			sub_grid_uuid: sub_grid_uuid,
			sub_grid_id: None,
			bounds: None,
			gpu_state: Cell::new(SubGridGpuState::new()),
		}
	}
	pub fn physics_body_grid_uuid(&self) -> &ResourceUUID { &self.physics_body_grid_uuid }
	pub fn sub_grid_id(&self) -> Option<SubGridId> { self.sub_grid_id }
	pub fn set_sub_grid_id(&mut self, sub_grid_id: Option<SubGridId>) {
		self.sub_grid_id = sub_grid_id;
	}
	pub fn bounds(&self) -> Option<Aabb> { self.bounds }
	pub fn gpu_state(&self) -> SubGridGpuState { self.gpu_state.get() }
	pub fn request_gpu_state(
		&self,
		async_task_priority_queue: &AsyncTaskPriorityQueue,
		task_queue: &TaskQueue,
		physics_engine: &PhysicsEngine,
		resource_manager: &ResourceManager,
		request: SubGridGpuUploadingState,
		priority: f32
	) {
		if let Some(_currently_uploading) = &self.gpu_state.get().currently_uploading {
			return;
		} else {
			let mut gpu_state = self.gpu_state.get();
			gpu_state.currently_uploading = Some(request.clone());
			self.gpu_state.set(gpu_state);

			if self.sub_grid_id.is_none() {
				unimplemented!("Everything must be loaded!");
			}
			let physics_body_grid_resource = resource_manager.resource(self.physics_body_grid_uuid()).unwrap();
			let physics_body_grid_resource_info = match physics_body_grid_resource.info() {
				ResourceInfoType::PhysicsBodyGrid { grid_resource } => grid_resource,
				_ => panic!(),
			};
			if physics_body_grid_resource_info.physics_body_grid_id().is_none() {
				unimplemented!("Everything must be loaded!");
			}
			let physics_body_resource = resource_manager.resource(physics_body_grid_resource_info.physics_body_uuid()).unwrap();
			let physics_body_resource_info = match physics_body_resource.info() {
				ResourceInfoType::PhysicsBody { physics_body_resource } => physics_body_resource,
				_ => panic!(),
			};
			if physics_body_resource_info.physics_body_id().is_none() {
				unimplemented!("Everything must be loaded!");
			}

			let physics_body_id = physics_body_resource_info.physics_body_id().unwrap();
			let physics_body_grid_id = physics_body_grid_resource_info.physics_body_grid_id().unwrap();
			let sub_grid_id = self.sub_grid_id().unwrap();
			let physics_body = physics_engine.physics_body(physics_body_id).unwrap();
			let physics_body_grid = physics_body.grid(physics_body_grid_id).unwrap();
			let sub_grid = physics_body_grid.sub_grid(sub_grid_id).unwrap();
			let palette = sub_grid.get_voxels().get_palette().clone();
			let voxels = sub_grid.get_voxels().get_voxels().clone();

			let task_queue = task_queue.clone();
			let sub_grid_uuid = self.sub_grid_uuid.clone();

			async_task_priority_queue.push(PriorityTask::new(priority, async move {
				let (tree_buffer, voxel_buffer) = make_gpu_grid_tree(&voxels, &palette, request.lod_level);

				task_queue.lock().unwrap().push_back(Task::new(move |state: &mut State| {
					let sub_grid_resource = match state.resource_manager.resource(&sub_grid_uuid).unwrap().info() {
						ResourceInfoType::SubGrid { sub_grid_resource } => sub_grid_resource,
						_ => panic!()
					};
					let mut gpu_state = sub_grid_resource.gpu_state.get();
					gpu_state.currently_uploading = None;
					if gpu_state.on_gpu {
						match state.renderer.voxel_renderer.packed_64_tree_dynamic_buffer.replace_buffer(&state.renderer.device, &state.renderer.queue, gpu_state.tree_id, &tree_buffer) {
							Ok(gpu_grid_tree_id) => {
								match state.renderer.voxel_renderer.packed_voxel_data_dynamic_buffer.replace_buffer(&state.renderer.device, &state.renderer.queue, gpu_state.voxels_id, &voxel_buffer) {
									Ok(gpu_voxel_data_id) => {
										gpu_state.on_gpu = true;
										gpu_state.lod_level = request.lod_level;
										gpu_state.tree_id = gpu_grid_tree_id;
										gpu_state.voxels_id = gpu_voxel_data_id;
										tracy_client::plot!("64 tree bytes", state.renderer.voxel_renderer.packed_64_tree_dynamic_buffer.held_bytes() as f64);
										tracy_client::plot!("voxel data bytes", state.renderer.voxel_renderer.packed_voxel_data_dynamic_buffer.held_bytes() as f64);
									},
									Err(err) => {
										println!("{}", err);
										if let Err(err) = state.renderer.voxel_renderer.packed_64_tree_dynamic_buffer.remove_buffer(gpu_grid_tree_id) {
											println!("{}", err);
										}
										tracy_client::plot!("64 tree bytes", state.renderer.voxel_renderer.packed_64_tree_dynamic_buffer.held_bytes() as f64);
										tracy_client::plot!("voxel data bytes", state.renderer.voxel_renderer.packed_voxel_data_dynamic_buffer.held_bytes() as f64);
									},
								}
							},
							Err(err) => {
								println!("{}", err);
								if let Err(err) = state.renderer.voxel_renderer.packed_voxel_data_dynamic_buffer.remove_buffer(gpu_state.voxels_id) {
									println!("{}", err);
								}
								tracy_client::plot!("64 tree bytes", state.renderer.voxel_renderer.packed_64_tree_dynamic_buffer.held_bytes() as f64);
								tracy_client::plot!("voxel data bytes", state.renderer.voxel_renderer.packed_voxel_data_dynamic_buffer.held_bytes() as f64);
							}
						};
					} else {
						match state.renderer.voxel_renderer.packed_64_tree_dynamic_buffer.add_buffer(&state.renderer.device, &state.renderer.queue, &tree_buffer) {
							Ok(gpu_grid_tree_id) => {
								match state.renderer.voxel_renderer.packed_voxel_data_dynamic_buffer.add_buffer(&state.renderer.device, &state.renderer.queue, &voxel_buffer) {
									Ok(gpu_voxel_data_id) => {
										gpu_state.on_gpu = true;
										gpu_state.lod_level = request.lod_level;
										gpu_state.tree_id = gpu_grid_tree_id;
										gpu_state.voxels_id = gpu_voxel_data_id;
										tracy_client::plot!("64 tree bytes", state.renderer.voxel_renderer.packed_64_tree_dynamic_buffer.held_bytes() as f64);
										tracy_client::plot!("voxel data bytes", state.renderer.voxel_renderer.packed_voxel_data_dynamic_buffer.held_bytes() as f64);
									},
									Err(err) => {
										println!("{}", err);
										if let Err(err) = state.renderer.voxel_renderer.packed_64_tree_dynamic_buffer.remove_buffer(gpu_grid_tree_id) {
											println!("{}", err);
										}
										tracy_client::plot!("64 tree bytes", state.renderer.voxel_renderer.packed_64_tree_dynamic_buffer.held_bytes() as f64);
										tracy_client::plot!("voxel data bytes", state.renderer.voxel_renderer.packed_voxel_data_dynamic_buffer.held_bytes() as f64);
									},
								}
							},
							Err(err) => {
								println!("{}", err);
								tracy_client::plot!("64 tree bytes", state.renderer.voxel_renderer.packed_64_tree_dynamic_buffer.held_bytes() as f64);
								tracy_client::plot!("voxel data bytes", state.renderer.voxel_renderer.packed_voxel_data_dynamic_buffer.held_bytes() as f64);
							}
						};
					}
					sub_grid_resource.gpu_state.set(gpu_state);
				}));
			}));
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
