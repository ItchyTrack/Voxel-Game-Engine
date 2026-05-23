use bevy::ecs::entity::Entity;
use bevy::ecs::message::{Message, MessageReader};
use bevy::ecs::system::{Query, Res};
use bevy::ecs::world::World;

use crate::task_queue::{AsyncTaskPriorityQueueResource, PriorityTask, Task, TaskQueueResource};

use crate::grid::{Grid, SubGridId};
use crate::gpu_grid_tree::make_gpu_grid_tree;
use crate::world_gpu_data;

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

#[derive(Message, Debug, Clone)]
pub struct GpuStateRequestMessage {
	request: SubGridGpuUploadingState,
	priority: f32,
	grid_id: Entity,
	sub_grid_id: SubGridId,
}

pub fn request_gpu_state(
	gpu_state_request_messages: MessageReader<GpuStateRequestMessage>,
	mut grids: Query<&mut Grid>,
	task_queue: Res<TaskQueueResource>,
	async_task_priority_queue: Res<AsyncTaskPriorityQueueResource>,
) {
	for gpu_state_request_message in gpu_state_request_messages.read() {
		let grid = if let Ok(grid) = grids.get_mut(gpu_state_request_message.grid_id) { grid } else { continue; };
		let sub_grid = if let Some(sub_grid) = grid.sub_grid_mut(gpu_state_request_message.sub_grid_id) { sub_grid } else { continue; };
		let current_gpu_state = sub_grid.gpu_state_mut();
		if let Some(_currently_uploading) = &mut current_gpu_state.currently_uploading {
			return;
		} else {
			current_gpu_state.currently_uploading = Some(gpu_state_request_message.request.clone());
			let gpu_state_request_message = gpu_state_request_message.clone();

			let palette = sub_grid.get_voxels().get_palette().clone();
			let voxels = sub_grid.get_voxels().get_voxels().clone();

			let task_queue = task_queue.clone();

			async_task_priority_queue.push(PriorityTask::new(gpu_state_request_message.priority, async move {
				let (tree_buffer, voxel_buffer) = make_gpu_grid_tree(&voxels, &palette, gpu_state_request_message.request.lod_level);

				task_queue.push_back(Task::new(move |world: &World| {
					let grid = if let Ok(entity) = world.get_entity_mut(gpu_state_request_message.grid_id) { entity.get_components_mut::<&Grid>() } else { return; };
					let sub_grid = if let Some(sub_grid) = grid.sub_grid_mut(gpu_state_request_message.sub_grid_id) { sub_grid } else { return; };
					let gpu_state = sub_grid.gpu_state_mut();
					gpu_state.currently_uploading = None;
					let world_gpu_data = &mut world.get_resource_mut::<world_gpu_data::WorldGpuData>().unwrap();
					if gpu_state.on_gpu {
						let packed_64_tree_dynamic_buffer = &mut world_gpu_data.packed_64_tree_dynamic_buffer;
						match packed_64_tree_dynamic_buffer.replace_buffer(gpu_state.tree_id, &tree_buffer) {
							Ok(gpu_grid_tree_id) => {
								let packed_voxel_data_dynamic_buffer = &mut world_gpu_data.packed_voxel_data_dynamic_buffer;
								match packed_voxel_data_dynamic_buffer.replace_buffer(gpu_state.voxels_id, &voxel_buffer) {
									Ok(gpu_voxel_data_id) => {
										gpu_state.on_gpu = true;
										gpu_state.lod_level = gpu_state_request_message.request.lod_level;
										gpu_state.tree_id = gpu_grid_tree_id;
										gpu_state.voxels_id = gpu_voxel_data_id;
										tracy_client::plot!("64 tree bytes", packed_64_tree_dynamic_buffer.held_bytes() as f64);
										tracy_client::plot!("voxel data bytes", packed_voxel_data_dynamic_buffer.held_bytes() as f64);
									},
									Err(err) => {
										println!("{}", err);
										if let Err(err) = packed_64_tree_dynamic_buffer.remove_buffer(gpu_grid_tree_id) {
											println!("{}", err);
										}
										tracy_client::plot!("64 tree bytes", packed_64_tree_dynamic_buffer.held_bytes() as f64);
										tracy_client::plot!("voxel data bytes", packed_voxel_data_dynamic_buffer.held_bytes() as f64);
									},
								}
							},
							Err(err) => {
								println!("{}", err);
								if let Err(err) = world_gpu_data.packed_voxel_data_dynamic_buffer.remove_buffer(gpu_state.voxels_id) {
									println!("{}", err);
								}
								tracy_client::plot!("64 tree bytes", world_gpu_data.packed_64_tree_dynamic_buffer.held_bytes() as f64);
								tracy_client::plot!("voxel data bytes", world_gpu_data.packed_voxel_data_dynamic_buffer.held_bytes() as f64);
							}
						};
					} else {
						let packed_64_tree_dynamic_buffer = &mut world_gpu_data.packed_64_tree_dynamic_buffer;
						match packed_64_tree_dynamic_buffer.add_buffer(&tree_buffer) {
							Ok(gpu_grid_tree_id) => {
								let packed_voxel_data_dynamic_buffer = &mut world_gpu_data.packed_voxel_data_dynamic_buffer;
								match packed_voxel_data_dynamic_buffer.add_buffer(&voxel_buffer) {
									Ok(gpu_voxel_data_id) => {
										gpu_state.on_gpu = true;
										gpu_state.lod_level = gpu_state_request_message.request.lod_level;
										gpu_state.tree_id = gpu_grid_tree_id;
										gpu_state.voxels_id = gpu_voxel_data_id;
										tracy_client::plot!("64 tree bytes", packed_64_tree_dynamic_buffer.held_bytes() as f64);
										tracy_client::plot!("voxel data bytes", packed_voxel_data_dynamic_buffer.held_bytes() as f64);
									},
									Err(err) => {
										println!("{}", err);
										if let Err(err) = packed_64_tree_dynamic_buffer.remove_buffer(gpu_grid_tree_id) {
											println!("{}", err);
										}
										tracy_client::plot!("64 tree bytes", packed_64_tree_dynamic_buffer.held_bytes() as f64);
										tracy_client::plot!("voxel data bytes", packed_voxel_data_dynamic_buffer.held_bytes() as f64);
									},
								}
							},
							Err(err) => {
								println!("{}", err);
								tracy_client::plot!("64 tree bytes", packed_64_tree_dynamic_buffer.held_bytes() as f64);
								tracy_client::plot!("voxel data bytes", world_gpu_data.packed_voxel_data_dynamic_buffer.held_bytes() as f64);
							}
						};
					}
				}));
			}));
		}
	}
}
