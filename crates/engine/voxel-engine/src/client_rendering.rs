use bevy::app::{PluginGroup, PluginGroupBuilder};
use bevy::prelude::*;
use camera_voxel_loader::{CameraVoxelLoader, CameraVoxelLoaderPlugin, CameraVoxelLoaderSet, CameraVoxelTileClass};
use voxel_gpu::{RenderingGenerationPlugin, RenderingTileClass};
use voxel_marching_renderer::{VoxelMarchingRendererPlugin, voxel_camera::VoxelMarchingCamera};
use voxel_raster_renderer::{VoxelRasterRendererPlugin, voxel_camera::VoxelRasterCamera};
use voxel_ray_renderer::{VoxelRayRendererPlugin, voxel_camera::VoxelCamera};

#[derive(Default)]
pub struct ClientRenderingPlugins;

impl PluginGroup for ClientRenderingPlugins {
	fn build(self) -> PluginGroupBuilder {
		PluginGroupBuilder::start::<Self>()
			.add(RenderingGenerationPlugin)
			.add(VoxelRayRendererPlugin)
			.add(VoxelRasterRendererPlugin)
			.add(VoxelMarchingRendererPlugin)
			.add(CameraVoxelLoaderPlugin)
			.add(ClientRenderingLinkPlugin)
	}
}

struct ClientRenderingLinkPlugin;

impl Plugin for ClientRenderingLinkPlugin {
	fn build(&self, app: &mut App) {
		app.add_systems(Update, sync_voxel_camera_components.after(CameraVoxelLoaderSet::RefreshVisibility));
	}
}

fn sync_voxel_camera_components(
	mut commands: Commands,
	rendering_class: Res<RenderingTileClass>,
	mut cameras: Query<(
		Entity,
		Option<&CameraVoxelLoader>,
		Option<&CameraVoxelTileClass>,
		Option<&mut VoxelCamera>,
		Option<&mut VoxelRasterCamera>,
		Option<&mut VoxelMarchingCamera>,
	), With<Camera3d>>,
) {
	for (entity, loader, current_class, ray_camera, raster_camera, marching_camera) in &mut cameras {
		let tiles_to_render = loader.map(|loader| loader.tiles_to_render().collect::<Vec<_>>()).unwrap_or_default();
		let mut entity_commands = commands.entity(entity);
		if current_class.map(|class| class.0) != Some(rendering_class.0) {
			entity_commands.insert(CameraVoxelTileClass(rendering_class.0));
		}
		if let Some(mut camera) = ray_camera { camera.tiles_to_render = tiles_to_render.clone(); }
		else { entity_commands.insert(VoxelCamera { tiles_to_render: tiles_to_render.clone() }); }
		if let Some(mut camera) = raster_camera { camera.tiles_to_render = tiles_to_render.clone(); }
		else { entity_commands.insert(VoxelRasterCamera { tiles_to_render: tiles_to_render.clone() }); }
		if let Some(mut camera) = marching_camera { camera.tiles_to_render = tiles_to_render.clone(); }
		else { entity_commands.insert(VoxelMarchingCamera { tiles_to_render }); }
	}
}
