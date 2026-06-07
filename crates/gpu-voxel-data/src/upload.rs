use std::collections::HashSet;

use bevy::ecs::world::World;
use bevy::prelude::*;

use voxel_data::grid::Grid;
use voxel_data::subgrid::SubGrid;
use voxel_data::task_queue::{AsyncTaskPriorityQueueResource, PriorityTask, TaskQueueResource};

use crate::gpu_grid_tree::make_gpu_grid_tree;
use crate::lod_request::DesiredLods;
use crate::sub_grid_gpu_state::SubGridGpuState;
use crate::world_gpu_data::WorldGpuData;

const LOD_REUPLOAD_EPSILON: f32 = 0.25;

#[derive(Resource, Default)]
pub(crate) struct InFlightUploads(HashSet<Entity>);

#[derive(Component)]
pub(crate) struct NeedsReupload;

pub(crate) fn flag_changed_sub_grids(
	mut commands: Commands,
	changed: Query<Entity, Changed<SubGrid>>,
) {
	for entity in changed.iter() {
		commands.entity(entity).insert(NeedsReupload);
	}
}

pub(crate) fn manage_gpu_uploads(
	desired: Res<DesiredLods>,
	mut commands: Commands,
	mut in_flight: ResMut<InFlightUploads>,
	sub_grids: Query<(Entity, &SubGrid, Option<&SubGridGpuState>, Has<NeedsReupload>)>,
	grids: Query<&Grid>,
	task_queue: Res<TaskQueueResource>,
	async_task_priority_queue: Res<AsyncTaskPriorityQueueResource>,
) {
	for (entity, sub_grid, state, needs_reupload) in sub_grids.iter() {
		let Some(request) = desired.get(entity) else {
			if state.is_some() && !in_flight.0.contains(&entity) {
				commands.entity(entity).remove::<SubGridGpuState>();
			}
			continue;
		};

		if in_flight.0.contains(&entity) { continue; }
		let needs_upload = match state {
			Some(s) if request.lod_level == 0.0 => s.lod_level() != 0.0 || needs_reupload,
			Some(s) => (s.lod_level() - request.lod_level).abs() > LOD_REUPLOAD_EPSILON || needs_reupload,
			None => true,
		};
		if !needs_upload { continue; }

		let Some(view) = grids.get(sub_grid.grid()).ok().and_then(|g| g.view(sub_grid)) else { continue };

		in_flight.0.insert(entity);
		commands.entity(entity).remove::<NeedsReupload>();

		let palette = view.voxels().palette().clone();
		let voxels = view.voxels().grid_tree().clone();
		let task_queue = task_queue.clone();
		let lod_level = request.lod_level;

		async_task_priority_queue.push(PriorityTask::new(request.priority, async move {
			let (tree_buffer, voxel_buffer) = make_gpu_grid_tree(&voxels, &palette, lod_level);
			task_queue.push(move |world: &mut World| {
				apply_gpu_upload(world, entity, lod_level, &tree_buffer, &voxel_buffer);
			});
		}));
	}
}

fn apply_gpu_upload(
	world: &mut World,
	entity: Entity,
	lod_level: f32,
	tree_buffer: &[u8],
	voxel_buffer: &[u8],
) {
	if let Some(mut in_flight) = world.get_resource_mut::<InFlightUploads>() {
		in_flight.0.remove(&entity);
	}

	// The sub-grid may have been despawned (or unloaded) while we were building.
	if world.get_entity(entity).is_err() { return; }
	let existing = world.get::<SubGridGpuState>(entity).copied();

	let Some(mut gpu_data) = world.get_resource_mut::<WorldGpuData>() else { return };
	let new_state = match existing {
		Some(old) => upload_replace(&mut gpu_data, old, lod_level, tree_buffer, voxel_buffer),
		None => upload_new(&mut gpu_data, lod_level, tree_buffer, voxel_buffer),
	};
	plot_gpu_usage(&gpu_data);

	if let Some(new_state) = new_state {
		if let Ok(mut entity_mut) = world.get_entity_mut(entity) {
			entity_mut.insert(new_state);
		}
	}
}

fn upload_new(
	gpu_data: &mut WorldGpuData,
	lod_level: f32,
	tree_buffer: &[u8],
	voxel_buffer: &[u8],
) -> Option<SubGridGpuState> {
	let tree_id = match gpu_data.packed_64_tree_dynamic_buffer.add_buffer(tree_buffer) {
		Ok(id) => id,
		Err(err) => { log::warn!("{err}"); return None; }
	};
	match gpu_data.packed_voxel_data_dynamic_buffer.add_buffer(voxel_buffer) {
		Ok(voxels_id) => Some(SubGridGpuState::new(lod_level, tree_id, voxels_id)),
		Err(err) => {
			log::warn!("{err}");
			if let Err(err) = gpu_data.packed_64_tree_dynamic_buffer.remove_buffer(tree_id) {
				log::warn!("{err}");
			}
			None
		}
	}
}

fn upload_replace(
	gpu_data: &mut WorldGpuData,
	old: SubGridGpuState,
	lod_level: f32,
	tree_buffer: &[u8],
	voxel_buffer: &[u8],
) -> Option<SubGridGpuState> {
	let tree_id = match gpu_data.packed_64_tree_dynamic_buffer.replace_buffer(old.tree_id(), tree_buffer) {
		Ok(id) => id,
		Err(err) => {
			log::warn!("{err}");
			if let Err(err) = gpu_data.packed_voxel_data_dynamic_buffer.remove_buffer(old.voxels_id()) {
				log::warn!("{err}");
			}
			return None;
		}
	};
	match gpu_data.packed_voxel_data_dynamic_buffer.replace_buffer(old.voxels_id(), voxel_buffer) {
		Ok(voxels_id) => Some(SubGridGpuState::new(lod_level, tree_id, voxels_id)),
		Err(err) => {
			log::warn!("{err}");
			if let Err(err) = gpu_data.packed_64_tree_dynamic_buffer.remove_buffer(tree_id) {
				log::warn!("{err}");
			}
			None
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
