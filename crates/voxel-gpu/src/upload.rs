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
use crate::gpu_raster_mesh::make_gpu_raster_mesh;
use crate::lod_voxels::LodVoxels;
use crate::voxel_gpu_state::{
	free_raster_buffer, free_ray_buffers, RasterGpuState, RayGpuState, SubGridPlacement,
	VoxelGpuBounds, VoxelGpuState,
};
use crate::world_gpu_data::WorldGpuData;
use crate::{VoxelGpuFormat, VoxelGpuUploadFinished};

#[derive(Resource, Default)]
pub(crate) struct InFlightRayUploads(HashSet<Entity>);

#[derive(Resource, Default)]
pub(crate) struct InFlightRasterUploads(HashSet<Entity>);

#[derive(Component)]
pub(crate) struct NeedsRayReupload;

#[derive(Component)]
pub(crate) struct NeedsRasterReupload;

pub(crate) fn flag_changed_sub_grids(
	mut commands: Commands,
	changed: Query<Entity, Changed<SubGrid>>,
) {
	for entity in changed.iter() {
		commands.entity(entity).insert((NeedsRayReupload, NeedsRasterReupload));
	}
}

fn format_in_use(
	format: VoxelGpuFormat,
	cameras: &Query<(&Camera, &GlobalTransform, Option<&VoxelGpuFormat>)>,
) -> bool {
	cameras
		.iter()
		.any(|(camera, _, camera_format)| camera.is_active && camera_format.copied().unwrap_or_default() == format)
}

fn active_camera_world_position(
	format: VoxelGpuFormat,
	cameras: &Query<(&Camera, &GlobalTransform, Option<&VoxelGpuFormat>)>,
) -> Option<Vec3> {
	cameras
		.iter()
		.find(|(camera, _, camera_format)| {
			camera.is_active && camera_format.copied().unwrap_or_default() == format
		})
		.map(|(_, transform, _)| transform.translation())
}

fn world_format_in_use(world: &mut World, format: VoxelGpuFormat) -> bool {
	let mut cameras = world.query::<(&Camera, Option<&VoxelGpuFormat>)>();
	cameras
		.iter(world)
		.any(|(camera, camera_format)| camera.is_active && camera_format.copied().unwrap_or_default() == format)
}

pub(crate) fn clear_inactive_formats(
	mut commands: Commands,
	cameras: Query<(&Camera, &GlobalTransform, Option<&VoxelGpuFormat>)>,
	mut gpu_state: Query<(Entity, &mut VoxelGpuState)>,
	mut gpu_data: ResMut<WorldGpuData>,
) {
	let keep_volume = format_in_use(VoxelGpuFormat::Volume, &cameras);
	let keep_surface = format_in_use(VoxelGpuFormat::Surface, &cameras);
	if keep_volume && keep_surface { return; }

	for (entity, mut state) in &mut gpu_state {
		if !keep_volume && let Some(ray) = state.ray {
			free_ray_buffers(&mut gpu_data, ray);
			state.clear_ray();
		}
		if !keep_surface && let Some(raster) = state.raster {
			free_raster_buffer(&mut gpu_data, raster);
			state.clear_raster();
		}
		if state.is_empty() {
			commands.entity(entity).remove::<VoxelGpuState>();
		}
	}
}

fn upload_priority(
	camera_world: Option<Vec3>,
	grid_transform: Option<&GlobalTransform>,
	center_local: Vec3,
) -> f32 {
	camera_world
		.zip(grid_transform)
		.map(|(camera, grid_global)| {
			let center_world = grid_global.transform_point(center_local);
			-camera.distance(center_world)
		})
		.unwrap_or(0.0)
}

pub(crate) fn manage_ray_gpu_uploads(
	mut commands: Commands,
	mut in_flight: ResMut<InFlightRayUploads>,
	sub_grids: Query<(Entity, &SubGrid, Option<&VoxelGpuState>, Has<NeedsRayReupload>)>,
	grids: Query<&Grid>,
	grid_transforms: Query<&GlobalTransform>,
	cameras: Query<(&Camera, &GlobalTransform, Option<&VoxelGpuFormat>)>,
	task_queue: Res<TaskQueueResource>,
	async_task_priority_queue: Res<AsyncTaskPriorityQueueResource>,
) {
	let _zone = span!("GPU uploads: schedule ray sub-grids");
	let Some(camera_world) = active_camera_world_position(VoxelGpuFormat::Volume, &cameras) else { return; };

	for (entity, sub_grid, state, needs_reupload) in sub_grids.iter() {
		if in_flight.0.contains(&entity) { continue; }
		let needs_upload = match state.and_then(|state| state.ray) {
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
		let center_local = sub_grid.sub_grid_pos().as_vec3() + (bounds_min.as_vec3() + bounds_max.as_vec3()) * 0.5;
		let priority = upload_priority(Some(camera_world), grid_transforms.get(sub_grid.grid()).ok(), center_local);

		in_flight.0.insert(entity);
		commands.entity(entity).remove::<NeedsRayReupload>();

		let palette = view.voxels().palette().clone();
		let voxels = view.voxels().grid_tree().clone();
		let task_queue = task_queue.clone();

		async_task_priority_queue.push(PriorityTask::new(priority, async move {
			let _zone = span!("GPU upload build ray sub-grid");
			let (tree_buffer, voxel_buffer) = make_gpu_grid_tree(&voxels, &palette);
			task_queue.push(move |world: &mut World| {
				let _zone = span!("GPU upload apply ray sub-grid");
				apply_ray_upload(world, entity, placement, &tree_buffer, &voxel_buffer);
			});
		}));
	}
}

pub(crate) fn manage_ray_lod_uploads(
	mut in_flight: ResMut<InFlightRayUploads>,
	lod_voxel_entities: Query<(Entity, &LodVoxels, Option<&VoxelGpuState>)>,
	cameras: Query<(&Camera, &GlobalTransform, Option<&VoxelGpuFormat>)>,
	task_queue: Res<TaskQueueResource>,
	async_task_priority_queue: Res<AsyncTaskPriorityQueueResource>,
) {
	if active_camera_world_position(VoxelGpuFormat::Volume, &cameras).is_none() { return; }
	let _zone = span!("GPU uploads: schedule ray LODs");
	for (entity, lod_voxels, gpu_state) in lod_voxel_entities.iter() {
		if gpu_state.and_then(|state| state.ray).is_some() || in_flight.0.contains(&entity) { continue; }
		let Some((bounds_min, bounds_max)) = lod_voxels.voxels.bounding_box() else { continue };
		let placement = SubGridPlacement {
			tree_root_pos: lod_voxels.voxels.grid_tree().view().root_pos(),
			bounds_min,
			bounds_max,
		};

		in_flight.0.insert(entity);

		let palette = lod_voxels.voxels.palette().clone();
		let voxels = lod_voxels.voxels.grid_tree().clone();
		let task_queue = task_queue.clone();
		let priority = lod_voxels.priority;

		async_task_priority_queue.push(PriorityTask::new(priority, async move {
			let _zone = span!("GPU upload build ray LOD");
			let (tree_buffer, voxel_buffer) = make_gpu_grid_tree(&voxels, &palette);
			task_queue.push(move |world: &mut World| {
				let _zone = span!("GPU upload apply ray LOD");
				apply_ray_upload(world, entity, placement, &tree_buffer, &voxel_buffer);
			});
		}));
	}
}

pub(crate) fn manage_raster_gpu_uploads(
	mut commands: Commands,
	mut in_flight: ResMut<InFlightRasterUploads>,
	sub_grids: Query<(Entity, &SubGrid, Option<&VoxelGpuState>, Has<NeedsRasterReupload>)>,
	grids: Query<&Grid>,
	grid_transforms: Query<&GlobalTransform>,
	cameras: Query<(&Camera, &GlobalTransform, Option<&VoxelGpuFormat>)>,
	task_queue: Res<TaskQueueResource>,
	async_task_priority_queue: Res<AsyncTaskPriorityQueueResource>,
) {
	let _zone = span!("GPU uploads: schedule raster sub-grids");
	let Some(camera_world) = active_camera_world_position(VoxelGpuFormat::Surface, &cameras) else { return; };

	for (entity, sub_grid, state, needs_reupload) in sub_grids.iter() {
		if in_flight.0.contains(&entity) { continue; }
		let needs_upload = match state.and_then(|state| state.raster) {
			Some(_) => needs_reupload,
			None => true,
		};
		if !needs_upload { continue; }

		let Some(view) = grids.get(sub_grid.grid()).ok().and_then(|g| g.view(sub_grid)) else { continue };
		let Some((bounds_min, bounds_max)) = view.voxels().bounding_box() else { continue };
		let bounds = VoxelGpuBounds { min: bounds_min, max: bounds_max };
		let center_local = sub_grid.sub_grid_pos().as_vec3() + (bounds_min.as_vec3() + bounds_max.as_vec3()) * 0.5;
		let priority = upload_priority(Some(camera_world), grid_transforms.get(sub_grid.grid()).ok(), center_local);

		in_flight.0.insert(entity);
		commands.entity(entity).remove::<NeedsRasterReupload>();

		let palette = view.voxels().palette().clone();
		let voxels = view.voxels().grid_tree().clone();
		let task_queue = task_queue.clone();

		async_task_priority_queue.push(PriorityTask::new(priority, async move {
			let _zone = span!("GPU upload build raster sub-grid");
			let (face_buffer, palette_buffer, face_count) = make_gpu_raster_mesh(&voxels, &palette);
			task_queue.push(move |world: &mut World| {
				let _zone = span!("GPU upload apply raster sub-grid");
				apply_raster_upload(world, entity, bounds, &face_buffer, &palette_buffer, face_count);
			});
		}));
	}
}

pub(crate) fn manage_raster_lod_uploads(
	mut in_flight: ResMut<InFlightRasterUploads>,
	lod_voxel_entities: Query<(Entity, &LodVoxels, Option<&VoxelGpuState>)>,
	cameras: Query<(&Camera, &GlobalTransform, Option<&VoxelGpuFormat>)>,
	task_queue: Res<TaskQueueResource>,
	async_task_priority_queue: Res<AsyncTaskPriorityQueueResource>,
) {
	if active_camera_world_position(VoxelGpuFormat::Surface, &cameras).is_none() { return; }
	let _zone = span!("GPU uploads: schedule raster LODs");
	for (entity, lod_voxels, gpu_state) in lod_voxel_entities.iter() {
		if gpu_state.and_then(|state| state.raster).is_some() || in_flight.0.contains(&entity) { continue; }

		in_flight.0.insert(entity);

		let Some((bounds_min, bounds_max)) = lod_voxels.voxels.bounding_box() else { continue; };
		let bounds = VoxelGpuBounds { min: bounds_min, max: bounds_max };
		let palette = lod_voxels.voxels.palette().clone();
		let voxels = lod_voxels.voxels.grid_tree().clone();
		let task_queue = task_queue.clone();
		let priority = lod_voxels.priority;

		async_task_priority_queue.push(PriorityTask::new(priority, async move {
			let _zone = span!("GPU upload build raster LOD");
			let (face_buffer, palette_buffer, face_count) = make_gpu_raster_mesh(&voxels, &palette);
			task_queue.push(move |world: &mut World| {
				let _zone = span!("GPU upload apply raster LOD");
				apply_raster_upload(world, entity, bounds, &face_buffer, &palette_buffer, face_count);
			});
		}));
	}
}

fn apply_ray_upload(
	world: &mut World,
	entity: Entity,
	placement: SubGridPlacement,
	tree_buffer: &[u8],
	voxel_buffer: &[u8],
) {
	let _zone = span!("GPU upload apply ray");
	if let Some(mut in_flight) = world.get_resource_mut::<InFlightRayUploads>() {
		in_flight.0.remove(&entity);
	}

	if !world_format_in_use(world, VoxelGpuFormat::Volume) { return; }
	if world.get_entity(entity).is_err() { return; }
	let existing = world.get::<VoxelGpuState>(entity).cloned();

	let Some(mut gpu_data) = world.get_resource_mut::<WorldGpuData>() else { return; };
	let new_state = match existing.as_ref().and_then(|state| state.ray) {
		Some(old) => upload_replace_ray(&mut gpu_data, old, placement, tree_buffer, voxel_buffer),
		None => upload_new_ray(&mut gpu_data, placement, tree_buffer, voxel_buffer),
	};
	if let Some(raster) = existing.as_ref().and_then(|state| state.raster) {
		free_raster_buffer(&mut gpu_data, raster);
	}
	drop(gpu_data);

	if let Some(new_state) = new_state {
		let mut next = existing.unwrap_or_default();
		next.clear_raster();
		next.set_ray(new_state);
		if let Ok(mut entity_mut) = world.get_entity_mut(entity) {
			entity_mut.insert(next);
		}
		if let Some(mut messages) = world.get_resource_mut::<Messages<VoxelGpuUploadFinished>>() {
			messages.write(VoxelGpuUploadFinished { entity });
		}
	}
}

fn apply_raster_upload(
	world: &mut World,
	entity: Entity,
	bounds: VoxelGpuBounds,
	face_buffer: &[u8],
	palette_buffer: &[u8],
	face_count: u32,
) {
	let _zone = span!("GPU upload apply raster");
	if let Some(mut in_flight) = world.get_resource_mut::<InFlightRasterUploads>() {
		in_flight.0.remove(&entity);
	}

	if !world_format_in_use(world, VoxelGpuFormat::Surface) { return; }
	if world.get_entity(entity).is_err() { return; }
	let existing = world.get::<VoxelGpuState>(entity).cloned();
	if face_count == 0 || face_buffer.is_empty() {
		let Some(mut next) = existing else { return; };
		let Some(old) = next.raster else { return; };
		let Some(mut gpu_data) = world.get_resource_mut::<WorldGpuData>() else { return; };
		free_raster_buffer(&mut gpu_data, old);
		drop(gpu_data);
		next.clear_raster();
		if let Ok(mut entity_mut) = world.get_entity_mut(entity) {
			if next.is_empty() {
				entity_mut.remove::<VoxelGpuState>();
			} else {
				entity_mut.insert(next);
			}
		}
		return;
	}

	let Some(mut gpu_data) = world.get_resource_mut::<WorldGpuData>() else { return; };
	let new_state = match existing.as_ref().and_then(|state| state.raster) {
		Some(old) => upload_replace_raster(&mut gpu_data, old, face_buffer, palette_buffer, face_count),
		None => upload_new_raster(&mut gpu_data, face_buffer, palette_buffer, face_count),
	};
	if let Some(ray) = existing.as_ref().and_then(|state| state.ray) {
		free_ray_buffers(&mut gpu_data, ray);
	}
	drop(gpu_data);

	if let Some(new_state) = new_state {
		let mut next = existing.unwrap_or_default();
		next.clear_ray();
		next.set_raster(new_state, bounds);
		if let Ok(mut entity_mut) = world.get_entity_mut(entity) {
			entity_mut.insert(next);
		}
		if let Some(mut messages) = world.get_resource_mut::<Messages<VoxelGpuUploadFinished>>() {
			messages.write(VoxelGpuUploadFinished { entity });
		}
	}
}

fn upload_new_ray(
	gpu_data: &mut WorldGpuData,
	placement: SubGridPlacement,
	tree_buffer: &[u8],
	voxel_buffer: &[u8],
) -> Option<RayGpuState> {
	let _zone = span!("GPU upload buffer add ray");
	let tree_id = match gpu_data.packed_64_tree_dynamic_buffer.add_buffer(tree_buffer) {
		Ok(id) => id,
		Err(err) => { log::warn!("{err}"); return None; }
	};
	match gpu_data.packed_voxel_data_dynamic_buffer.add_buffer(voxel_buffer) {
		Ok(voxels_id) => Some(RayGpuState::new(tree_id, voxels_id, placement)),
		Err(err) => {
			log::warn!("{err}");
			if let Err(err) = gpu_data.packed_64_tree_dynamic_buffer.remove_buffer(tree_id) {
				log::warn!("{err}");
			}
			None
		}
	}
}

fn upload_replace_ray(
	gpu_data: &mut WorldGpuData,
	old: RayGpuState,
	placement: SubGridPlacement,
	tree_buffer: &[u8],
	voxel_buffer: &[u8],
) -> Option<RayGpuState> {
	let _zone = span!("GPU upload buffer replace ray");
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
		Ok(voxels_id) => Some(RayGpuState::new(tree_id, voxels_id, placement)),
		Err(err) => {
			log::warn!("{err}");
			if let Err(err) = gpu_data.packed_64_tree_dynamic_buffer.remove_buffer(tree_id) {
				log::warn!("{err}");
			}
			None
		}
	}
}

fn upload_new_raster(
	gpu_data: &mut WorldGpuData,
	face_buffer: &[u8],
	palette_buffer: &[u8],
	face_count: u32,
) -> Option<RasterGpuState> {
	let _zone = span!("GPU upload buffer add raster");
	let buffer_id = match gpu_data.packed_raster_face_dynamic_buffer.add_buffer(face_buffer) {
		Ok(buffer_id) => buffer_id,
		Err(err) => {
			log::warn!("{err}");
			return None;
		}
	};
	match gpu_data.packed_raster_palette_dynamic_buffer.add_buffer(palette_buffer) {
		Ok(palette_id) => Some(RasterGpuState::new(buffer_id, palette_id, face_count)),
		Err(err) => {
			log::warn!("{err}");
			if let Err(err) = gpu_data.packed_raster_face_dynamic_buffer.remove_buffer(buffer_id) {
				log::warn!("{err}");
			}
			None
		}
	}
}

fn upload_replace_raster(
	gpu_data: &mut WorldGpuData,
	old: RasterGpuState,
	face_buffer: &[u8],
	palette_buffer: &[u8],
	face_count: u32,
) -> Option<RasterGpuState> {
	let _zone = span!("GPU upload buffer replace raster");
	let buffer_id = match gpu_data.packed_raster_face_dynamic_buffer.replace_buffer(old.buffer_id(), face_buffer) {
		Ok(buffer_id) => buffer_id,
		Err(err) => {
			log::warn!("{err}");
			if let Err(err) = gpu_data.packed_raster_palette_dynamic_buffer.remove_buffer(old.palette_id()) {
				log::warn!("{err}");
			}
			return None;
		}
	};
	match gpu_data.packed_raster_palette_dynamic_buffer.replace_buffer(old.palette_id(), palette_buffer) {
		Ok(palette_id) => Some(RasterGpuState::new(buffer_id, palette_id, face_count)),
		Err(err) => {
			log::warn!("{err}");
			if let Err(err) = gpu_data.packed_raster_face_dynamic_buffer.remove_buffer(buffer_id) {
				log::warn!("{err}");
			}
			None
		}
	}
}
