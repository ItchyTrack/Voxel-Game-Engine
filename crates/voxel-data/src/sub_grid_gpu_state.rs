use crate::world::voxels::Voxels;

use super::super::{grid::{GridId, SubGridId}, world::{PriorityTask, Task, World}};
use super::gpu_grid_tree::make_gpu_grid_tree;

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

	pub fn request_gpu_state(
		&mut self,
		world: &World,
		request: SubGridGpuUploadingState,
		priority: f32,
		grid_id: GridId,
		sub_grid_id: SubGridId,
		voxels: &Voxels
	) {
		if let Some(_currently_uploading) = &self.currently_uploading {
			return;
		} else {
			self.currently_uploading = Some(request.clone());

			let palette = voxels.get_palette().clone();
			let voxels = voxels.get_voxels().clone();

			let task_queue = world.task_queue.clone();

			world.async_task_priority_queue.push(PriorityTask::new(priority, async move {
				let (tree_buffer, voxel_buffer) = make_gpu_grid_tree(&voxels, &palette, request.lod_level);

				task_queue.lock().push_back(Task::new(move |world: &World| {
					let grid = &mut if let Some(grid) = world.grid_mut(grid_id) { grid } else { return; };
					let sub_grid = if let Some(sub_grid) = grid.sub_grid_mut(sub_grid_id) { sub_grid } else { return; };
					let gpu_state = sub_grid.gpu_state_mut();
					gpu_state.currently_uploading = None;
					let world_gpu_data = &mut world.world_gpu_data.read();
					if gpu_state.on_gpu {
						let packed_64_tree_dynamic_buffer = &mut world_gpu_data.packed_64_tree_dynamic_buffer.write();
						match packed_64_tree_dynamic_buffer.replace_buffer(gpu_state.tree_id, &tree_buffer) {
							Ok(gpu_grid_tree_id) => {
								let packed_voxel_data_dynamic_buffer = &mut world_gpu_data.packed_voxel_data_dynamic_buffer.write();
								match packed_voxel_data_dynamic_buffer.replace_buffer(gpu_state.voxels_id, &voxel_buffer) {
									Ok(gpu_voxel_data_id) => {
										gpu_state.on_gpu = true;
										gpu_state.lod_level = request.lod_level;
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
								if let Err(err) = world_gpu_data.packed_voxel_data_dynamic_buffer.write().remove_buffer(gpu_state.voxels_id) {
									println!("{}", err);
								}
								tracy_client::plot!("64 tree bytes", world_gpu_data.packed_64_tree_dynamic_buffer.read().held_bytes() as f64);
								tracy_client::plot!("voxel data bytes", world_gpu_data.packed_voxel_data_dynamic_buffer.read().held_bytes() as f64);
							}
						};
					} else {
						let packed_64_tree_dynamic_buffer = &mut world_gpu_data.packed_64_tree_dynamic_buffer.write();
						match packed_64_tree_dynamic_buffer.add_buffer(&tree_buffer) {
							Ok(gpu_grid_tree_id) => {
								let packed_voxel_data_dynamic_buffer = &mut world_gpu_data.packed_voxel_data_dynamic_buffer.write();
								match packed_voxel_data_dynamic_buffer.add_buffer(&voxel_buffer) {
									Ok(gpu_voxel_data_id) => {
										gpu_state.on_gpu = true;
										gpu_state.lod_level = request.lod_level;
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
								tracy_client::plot!("voxel data bytes", world_gpu_data.packed_voxel_data_dynamic_buffer.read().held_bytes() as f64);
							}
						};
					}
				}));
			}));
		}
	}
}
