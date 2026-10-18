use bevy::app::{PluginGroup, PluginGroupBuilder};
use bevy::prelude::*;
use voxel_marching_renderer::{VoxelMarchingRendererPlugin, voxel_camera::VoxelMarchingCamera};
use voxel_raster_renderer::{VoxelRasterRendererPlugin, voxel_camera::VoxelRasterCamera};
use voxel_ray_renderer::{VoxelRayRendererPlugin, voxel_camera::VoxelCamera};

use crate::camera_tiles::{CameraTileLoader, CameraTilesPlugin};

#[derive(Default)]
pub struct ClientRenderingPlugins;

impl PluginGroup for ClientRenderingPlugins {
	fn build(self) -> PluginGroupBuilder {
		PluginGroupBuilder::start::<Self>()
			.add(VoxelRayRendererPlugin)
			.add(VoxelRasterRendererPlugin)
			.add(VoxelMarchingRendererPlugin)
			.add(CameraTilesPlugin)
			.add(ClientRenderingLinkPlugin)
	}
}

struct ClientRenderingLinkPlugin;

impl Plugin for ClientRenderingLinkPlugin {
	fn build(&self, app: &mut App) {
		app.add_systems(PostUpdate, sync_voxel_camera_components);
	}
}

fn sync_voxel_camera_components(
	mut commands: Commands,
	loaders: Query<&CameraTileLoader>,
	mut cameras: Query<(
		Entity,
		Option<&mut VoxelCamera>,
		Option<&mut VoxelRasterCamera>,
		Option<&mut VoxelMarchingCamera>,
	), With<Camera3d>>,
) {
	let tiles_to_render: Vec<_> = loaders.iter().flat_map(|loader| loader.loaded_tiles().map(|(_, entity)| entity)).collect();
	for (entity, ray_camera, raster_camera, marching_camera) in &mut cameras {
		let mut entity_commands = commands.entity(entity);
		if let Some(mut camera) = ray_camera { camera.tiles_to_render = tiles_to_render.clone(); }
		else { entity_commands.insert(VoxelCamera { tiles_to_render: tiles_to_render.clone() }); }
		if let Some(mut camera) = raster_camera { camera.tiles_to_render = tiles_to_render.clone(); }
		else { entity_commands.insert(VoxelRasterCamera { tiles_to_render: tiles_to_render.clone() }); }
		if let Some(mut camera) = marching_camera { camera.tiles_to_render = tiles_to_render.clone(); }
		else { entity_commands.insert(VoxelMarchingCamera { tiles_to_render: tiles_to_render.clone() }); }
	}
}
