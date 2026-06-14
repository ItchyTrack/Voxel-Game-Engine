use std::collections::HashSet;

use bevy::camera::Camera;
use bevy::ecs::message::Messages;
use bevy::ecs::world::World;
use bevy::prelude::*;
use tracy_client::span;

use voxel_data::grid::Grid;
use voxel_data::subgrid::SubGrid;
use voxel_data::task_queue::{AsyncTaskPriorityQueueResource, PriorityTask, TaskQueueResource};

use crate::gpu_grid_tree::make_gpu_grid_tree;
use crate::lod_voxels::LodVoxels;
use crate::sub_grid_gpu_state::{SubGridGpuState, SubGridPlacement};
use crate::world_gpu_data::WorldGpuData;
use crate::VoxelGpuUploadFinished;

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
	mut commands: Commands,
	mut in_flight: ResMut<InFlightUploads>,
	sub_grids: Query<(Entity, &SubGrid, Option<&SubGridGpuState>, Has<NeedsReupload>)>,
	grids: Query<&Grid>,
	grid_transforms: Query<&GlobalTransform>,
	cameras: Query<(&Camera, &GlobalTransform)>,
	task_queue: Res<TaskQueueResource>,
	async_task_priority_queue: Res<AsyncTaskPriorityQueueResource>,
) {
	let _zone = span!("GPU uploads: schedule sub-grids");
	let camera_world = cameras
		.iter()
		.find(|(camera, _)| camera.is_active)
		.map(|(_, transform)| transform.translation());

	for (entity, sub_grid, state, needs_reupload) in sub_grids.iter() {
		if in_flight.0.contains(&entity) { continue; }
		let needs_upload = match state {
			Some(_) => needs_reupload,
			None => true,
		};
		if !needs_upload { continue; }

		let Some(view) = grids.get(sub_grid.grid()).ok().and_then(|g| g.view(sub_grid)) else { continue };
		let Some((bounds_min, bounds_max)) = view.voxels().bounding_box() else { continue };
		let placement = SubGridPlacement {
			tree_root_pos: view.voxels().grid_tree().view().root_pos(),
			bounds_min,
			bounds_max,
		};

		// Negative world distance: closer uploads first, comparable to LOD-tile priorities.
		let priority = camera_world
			.zip(grid_transforms.get(sub_grid.grid()).ok())
			.map(|(camera, grid_global)| {
				let center_local = sub_grid.sub_grid_pos().as_vec3()
					+ (bounds_min.as_vec3() + bounds_max.as_vec3()) * 0.5;
				let center_world = grid_global.transform_point(center_local);
				-camera.distance(center_world)
			})
			.unwrap_or(0.0);

		in_flight.0.insert(entity);
		commands.entity(entity).remove::<NeedsReupload>();
		tracy_client::plot!("gpu upload in flight", in_flight.0.len() as f64);

		let palette = view.voxels().palette().clone();
		let voxels = view.voxels().grid_tree().clone();
		let task_queue = task_queue.clone();

		async_task_priority_queue.push(PriorityTask::new(priority, async move {
			let _zone = span!("GPU upload build sub-grid");
			let (tree_buffer, voxel_buffer) = make_gpu_grid_tree(&voxels, &palette);
			tracy_client::plot!("gpu upload tree bytes built", tree_buffer.len() as f64);
			tracy_client::plot!("gpu upload voxel bytes built", voxel_buffer.len() as f64);
			task_queue.push(move |world: &mut World| {
				let _zone = span!("GPU upload apply sub-grid");
				apply_gpu_upload(world, entity, placement, &tree_buffer, &voxel_buffer);
			});
		}));
	}
}

pub(crate) fn manage_lod_uploads(
	mut in_flight: ResMut<InFlightUploads>,
	lod_voxel_entities: Query<(Entity, &LodVoxels, Option<&SubGridGpuState>)>,
	task_queue: Res<TaskQueueResource>,
	async_task_priority_queue: Res<AsyncTaskPriorityQueueResource>,
) {
	let _zone = span!("GPU uploads: schedule LODs");
	for (entity, lod_voxels, gpu_state) in lod_voxel_entities.iter() {
		if gpu_state.is_some() || in_flight.0.contains(&entity) { continue; }
		let Some((bounds_min, bounds_max)) = lod_voxels.voxels.bounding_box() else { continue };
		let placement = SubGridPlacement {
			tree_root_pos: lod_voxels.voxels.grid_tree().view().root_pos(),
			bounds_min,
			bounds_max,
		};

		in_flight.0.insert(entity);
		tracy_client::plot!("gpu upload in flight", in_flight.0.len() as f64);

		let palette = lod_voxels.voxels.palette().clone();
		let voxels = lod_voxels.voxels.grid_tree().clone();
		let task_queue = task_queue.clone();
		let priority = lod_voxels.priority;

		async_task_priority_queue.push(PriorityTask::new(priority, async move {
			let _zone = span!("GPU upload build LOD");
			let (tree_buffer, voxel_buffer) = make_gpu_grid_tree(&voxels, &palette);
			tracy_client::plot!("gpu upload tree bytes built", tree_buffer.len() as f64);
			tracy_client::plot!("gpu upload voxel bytes built", voxel_buffer.len() as f64);
			task_queue.push(move |world: &mut World| {
				let _zone = span!("GPU upload apply LOD");
				apply_gpu_upload(world, entity, placement, &tree_buffer, &voxel_buffer);
			});
		}));
	}
}

fn apply_gpu_upload(
	world: &mut World,
	entity: Entity,
	placement: SubGridPlacement,
	tree_buffer: &[u8],
	voxel_buffer: &[u8],
) {
	let _zone = span!("GPU upload apply");
	if let Some(mut in_flight) = world.get_resource_mut::<InFlightUploads>() {
		in_flight.0.remove(&entity);
		tracy_client::plot!("gpu upload in flight", in_flight.0.len() as f64);
	}

	// The sub-grid may have been despawned (or unloaded) while we were building.
	if world.get_entity(entity).is_err() { return; }
	let existing = world.get::<SubGridGpuState>(entity).copied();

	let Some(mut gpu_data) = world.get_resource_mut::<WorldGpuData>() else { return };
	let new_state = match existing {
		Some(old) => upload_replace(&mut gpu_data, old, placement, tree_buffer, voxel_buffer),
		None => upload_new(&mut gpu_data, placement, tree_buffer, voxel_buffer),
	};
	plot_gpu_usage(&gpu_data);

	if let Some(new_state) = new_state {
		if let Ok(mut entity_mut) = world.get_entity_mut(entity) {
			entity_mut.insert(new_state);
		}
		if let Some(mut messages) = world.get_resource_mut::<Messages<VoxelGpuUploadFinished>>() {
			messages.write(VoxelGpuUploadFinished { entity });
		}
	}
}

fn upload_new(
	gpu_data: &mut WorldGpuData,
	placement: SubGridPlacement,
	tree_buffer: &[u8],
	voxel_buffer: &[u8],
) -> Option<SubGridGpuState> {
	let _zone = span!("GPU upload buffer add");
	let tree_id = match gpu_data.packed_64_tree_dynamic_buffer.add_buffer(tree_buffer) {
		Ok(id) => id,
		Err(err) => { log::warn!("{err}"); return None; }
	};
	match gpu_data.packed_voxel_data_dynamic_buffer.add_buffer(voxel_buffer) {
		Ok(voxels_id) => Some(SubGridGpuState::new(tree_id, voxels_id, placement)),
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
	placement: SubGridPlacement,
	tree_buffer: &[u8],
	voxel_buffer: &[u8],
) -> Option<SubGridGpuState> {
	let _zone = span!("GPU upload buffer replace");
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
		Ok(voxels_id) => Some(SubGridGpuState::new(tree_id, voxels_id, placement)),
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
