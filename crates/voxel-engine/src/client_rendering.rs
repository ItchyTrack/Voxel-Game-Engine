use bevy::app::{PluginGroup, PluginGroupBuilder};
use bevy::prelude::*;
use camera_voxel_loader::{CameraVoxelLoader, CameraVoxelLoaderPlugin, CameraVoxelLoaderSet, CameraVoxelTileClass};
use voxel_raster_renderer::{VoxelRasterRendererPlugin, voxel_camera::VoxelRasterCamera};
use voxel_ray_renderer::{VoxelRayRendererPlugin, voxel_camera::VoxelCamera};
use tile_data::TileClassId;

use crate::VoxelCameraMode;

#[derive(Resource, Clone, Copy, Debug)]
pub struct VoxelRenderTileClasses {
	pub ray: TileClassId,
	pub raster: TileClassId,
}

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
			.add_systems(Update, ensure_voxel_camera_modes)
			.add_systems(Update, sync_voxel_camera_components.after(CameraVoxelLoaderSet::RefreshVisibility));
	}
}

fn ensure_voxel_camera_modes(mut commands: Commands, cameras: Query<(Entity, Option<&VoxelCameraMode>), With<Camera3d>>) {
	for (entity, mode) in &cameras {
		if mode.is_none() { commands.entity(entity).insert(VoxelCameraMode::default()); }
	}
}

fn sync_voxel_camera_components(
	mut commands: Commands,
	classes: Option<Res<VoxelRenderTileClasses>>,
	mut cameras: Query<(
		Entity,
		&VoxelCameraMode,
		Option<&CameraVoxelLoader>,
		Option<&CameraVoxelTileClass>,
		Option<&mut VoxelCamera>,
		Option<&mut VoxelRasterCamera>,
	), With<Camera3d>>,
) {
	let Some(classes) = classes else { return };
	for (entity, mode, loader, current_class, ray_camera, raster_camera) in &mut cameras {
		let tiles_to_render = loader.map(|loader| loader.tiles_to_render().collect::<Vec<_>>()).unwrap_or_default();
		let desired_class = match mode {
			VoxelCameraMode::Ray => classes.ray,
			VoxelCameraMode::Raster => classes.raster,
		};
		let mut entity_commands = commands.entity(entity);
		if current_class.map(|class| class.0) != Some(desired_class) {
			entity_commands.insert(CameraVoxelTileClass(desired_class));
		}
		match mode {
			VoxelCameraMode::Ray => {
				if let Some(mut camera) = ray_camera { camera.tiles_to_render = tiles_to_render.clone(); }
				else { entity_commands.insert(VoxelCamera { tiles_to_render: tiles_to_render.clone() }); }
				if raster_camera.is_some() { entity_commands.remove::<VoxelRasterCamera>(); }
			}
			VoxelCameraMode::Raster => {
				if let Some(mut camera) = raster_camera { camera.tiles_to_render = tiles_to_render.clone(); }
				else { entity_commands.insert(VoxelRasterCamera { tiles_to_render: tiles_to_render.clone() }); }
				if ray_camera.is_some() { entity_commands.remove::<VoxelCamera>(); }
			}
		}
	}
}
