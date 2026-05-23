use bevy::ecs::entity::Entity;
use bevy::ecs::message::{Message, MessageReader};
use bevy::ecs::system::{Query, Res};
use bevy::ecs::world::World;

use crate::task_queue::{AsyncTaskPriorityQueueResource, PriorityTask, Task, TaskQueueResource};
use crate::grid::{Grid, SubGridId};
use crate::gpu_grid_tree::make_gpu_grid_tree;
use crate::world_gpu_data::WorldGpuData;

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
    currently_uploading: Option<SubGridGpuUploadingState>,
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
    pub fn currently_uploading(&self) -> Option<&SubGridGpuUploadingState> {
        self.currently_uploading.as_ref()
    }
}

#[derive(Message, Debug, Clone)]
pub struct GpuStateRequestMessage {
    request: SubGridGpuUploadingState,
    priority: f32,
    grid_id: Entity,
    sub_grid_id: SubGridId,
}

pub fn request_gpu_state(
    mut gpu_state_request_messages: MessageReader<GpuStateRequestMessage>,
    mut grids: Query<&mut Grid>,
    task_queue: Res<TaskQueueResource>,
    async_task_priority_queue: Res<AsyncTaskPriorityQueueResource>,
) {
    for msg in gpu_state_request_messages.read() {
        let Ok(mut grid) = grids.get_mut(msg.grid_id) else { continue };
        let Some(sub_grid) = grid.sub_grid_mut(msg.sub_grid_id) else { continue };
        let gpu_state = sub_grid.gpu_state_mut();

        if gpu_state.currently_uploading.is_some() {
            continue; // upload already in progress
        }

        gpu_state.currently_uploading = Some(msg.request);

        let msg = msg.clone();
        let palette = sub_grid.get_voxels().get_palette().clone();
        let voxels = sub_grid.get_voxels().get_voxels().clone();
        let task_queue = task_queue.clone();

        async_task_priority_queue.push(PriorityTask::new(msg.priority, async move {
            let (tree_buffer, voxel_buffer) =
                make_gpu_grid_tree(&voxels, &palette, msg.request.lod_level);

            task_queue.push_back(Task::new(move |world: &mut World| {
                apply_gpu_upload(world, &msg, &tree_buffer, &voxel_buffer);
            }));
        }));
    }
}

fn apply_gpu_upload(
    world: &mut World,
    msg: &GpuStateRequestMessage,
    tree_buffer: &[u8],
    voxel_buffer: &[u8],
) {
    let mut gpu_state = {
		let Some(grid) = world.get::<Grid>(msg.grid_id) else { return };
		let Some(sub_grid) = grid.sub_grid(msg.sub_grid_id) else { return };
		sub_grid.gpu_state().clone()
	};
    gpu_state.currently_uploading = None;

    let mut world_gpu_data = world.resource_mut::<WorldGpuData>();

    if gpu_state.on_gpu {
        upload_replace(&mut *world_gpu_data, &mut gpu_state, msg, tree_buffer, voxel_buffer);
    } else {
        upload_new(&mut *world_gpu_data, &mut gpu_state, msg, tree_buffer, voxel_buffer);
    }

    plot_gpu_usage(&mut *world_gpu_data);

	let Some(mut grid) = world.get_mut::<Grid>(msg.grid_id) else { return };
	let Some(sub_grid) = grid.sub_grid_mut(msg.sub_grid_id) else { return };
	*sub_grid.gpu_state_mut() = gpu_state;
}

fn upload_new(
    gpu_data: &mut WorldGpuData,
    gpu_state: &mut SubGridGpuState,
    msg: &GpuStateRequestMessage,
    tree_buffer: &[u8],
    voxel_buffer: &[u8],
) {
	let tree_id = match gpu_data.packed_64_tree_dynamic_buffer.add_buffer(tree_buffer) {
        Ok(id) => id,
        Err(err) => { println!("{err}"); return; }
    };

    match gpu_data.packed_voxel_data_dynamic_buffer.add_buffer(voxel_buffer) {
        Ok(voxels_id) => {
            gpu_state.on_gpu = true;
            gpu_state.lod_level = msg.request.lod_level;
            gpu_state.tree_id = tree_id;
            gpu_state.voxels_id = voxels_id;
        }
        Err(err) => {
            println!("{err}");
            if let Err(err) = gpu_data.packed_64_tree_dynamic_buffer.remove_buffer(tree_id) {
                println!("{err}");
            }
        }
    }
}

fn upload_replace(
    gpu_data: &mut WorldGpuData,
    gpu_state: &mut SubGridGpuState,
    msg: &GpuStateRequestMessage,
    tree_buffer: &[u8],
    voxel_buffer: &[u8],
) {
    let tree_id = match gpu_data.packed_64_tree_dynamic_buffer.replace_buffer(gpu_state.tree_id, tree_buffer) {
        Ok(id) => id,
        Err(err) => {
            println!("{err}");
            if let Err(err) = gpu_data.packed_voxel_data_dynamic_buffer.remove_buffer(gpu_state.voxels_id) {
                println!("{err}");
            }
            return;
        }
    };

    match gpu_data.packed_voxel_data_dynamic_buffer.replace_buffer(gpu_state.voxels_id, voxel_buffer) {
        Ok(voxels_id) => {
            gpu_state.on_gpu = true;
            gpu_state.lod_level = msg.request.lod_level;
            gpu_state.tree_id = tree_id;
            gpu_state.voxels_id = voxels_id;
        }
        Err(err) => {
            println!("{err}");
            if let Err(err) = gpu_data.packed_64_tree_dynamic_buffer.remove_buffer(tree_id) {
                println!("{err}");
            }
        }
    }
}

fn plot_gpu_usage(gpu_data: &WorldGpuData) {
    tracy_client::plot!(
        "64 tree bytes",
        gpu_data.packed_64_tree_dynamic_buffer.held_bytes() as f64
    );
    tracy_client::plot!(
        "voxel data bytes",
        gpu_data.packed_voxel_data_dynamic_buffer.held_bytes() as f64
    );
}
