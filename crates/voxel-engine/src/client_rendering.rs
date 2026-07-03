use bevy::app::{PluginGroup, PluginGroupBuilder};
use bevy::prelude::*;
use camera_voxel_loader::{CameraVoxelLoaderPlugin, CameraVoxelLoaderSet, CameraVoxelRenderState};
use voxel_gpu::VoxelGpuFormat;
use voxel_raster_renderer::{VoxelRasterRendererPlugin, voxel_camera::VoxelRasterCamera};
use voxel_ray_renderer::{VoxelRayRendererPlugin, voxel_camera::VoxelCamera};

use crate::VoxelCameraMode;

#[derive(Default)]
pub struct ClientRenderingPlugins;

impl PluginGroup for ClientRenderingPlugins {
	fn build(self) -> PluginGroupBuilder {
		PluginGroupBuilder::start::<Self>()
			.add(VoxelRayRendererPlugin)
			.add(VoxelRasterRendererPlugin)
			.add(CameraVoxelLoaderPlugin)
			.add(ClientRenderingLinkPlugin)
	}
}

struct ClientRenderingLinkPlugin;

impl Plugin for ClientRenderingLinkPlugin {
	fn build(&self, app: &mut App) {
		app.register_type::<VoxelCameraMode>()
			.add_systems(Update, ensure_voxel_camera_modes.before(voxel_gpu::GpuUploadSet::Upload))
			.add_systems(Update, sync_voxel_camera_components.after(CameraVoxelLoaderSet::RefreshVisibility));
	}
}

fn ensure_voxel_camera_modes(
	mut commands: Commands,
	cameras: Query<(Entity, Option<&VoxelCameraMode>, Option<&VoxelGpuFormat>), With<Camera3d>>,
) {
	for (entity, mode, format) in &cameras {
		let mode_value = mode.copied().unwrap_or_default();
		let desired_format = match mode_value {
			VoxelCameraMode::Ray => VoxelGpuFormat::Volume,
			VoxelCameraMode::Raster => VoxelGpuFormat::Surface,
		};
		let mut entity_commands = commands.entity(entity);
		if mode.is_none() {
			entity_commands.insert(mode_value);
		}
		if format.copied() != Some(desired_format) {
			entity_commands.insert(desired_format);
		}
	}
}

fn sync_voxel_camera_components(
	mut commands: Commands,
	mut cameras: Query<(
		Entity,
		&VoxelCameraMode,
		Option<&CameraVoxelRenderState>,
		Option<&mut VoxelCamera>,
		Option<&mut VoxelRasterCamera>,
		Option<&VoxelGpuFormat>,
	), With<Camera3d>>,
) {
	for (entity, mode, render_state, ray_camera, raster_camera, format) in &mut cameras {
		let (subgrids_to_render, lods_to_render) = render_state
			.map(|state| (state.subgrids_to_render.clone(), state.lods_to_render.clone()))
			.unwrap_or_default();

		let desired_format = match mode {
			VoxelCameraMode::Ray => VoxelGpuFormat::Volume,
			VoxelCameraMode::Raster => VoxelGpuFormat::Surface,
		};
		let mut entity_commands = commands.entity(entity);
		if format.copied() != Some(desired_format) {
			entity_commands.insert(desired_format);
		}
		match mode {
			VoxelCameraMode::Ray => {
				if let Some(mut ray_camera) = ray_camera {
					ray_camera.subgrids_to_render = subgrids_to_render.clone();
					ray_camera.lods_to_render = lods_to_render.clone();
				} else {
					entity_commands.insert(VoxelCamera {
						subgrids_to_render: subgrids_to_render.clone(),
						lods_to_render: lods_to_render.clone(),
					});
				}
				if raster_camera.is_some() {
					entity_commands.remove::<VoxelRasterCamera>();
				}
			}
			VoxelCameraMode::Raster => {
				if let Some(mut raster_camera) = raster_camera {
					raster_camera.subgrids_to_render = subgrids_to_render.clone();
					raster_camera.lods_to_render = lods_to_render.clone();
				} else {
					entity_commands.insert(VoxelRasterCamera {
						subgrids_to_render: subgrids_to_render.clone(),
						lods_to_render: lods_to_render.clone(),
					});
				}
				if ray_camera.is_some() {
					entity_commands.remove::<VoxelCamera>();
				}
			}
		}
	}
}
