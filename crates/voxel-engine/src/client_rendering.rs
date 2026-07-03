use bevy::app::{PluginGroup, PluginGroupBuilder};
use bevy::prelude::*;
use camera_voxel_loader::{CameraVoxelLoaderPlugin, CameraVoxelLoaderSet, CameraVoxelRenderState};
use voxel_renderer::{VoxelRendererPlugin, voxel_camera::VoxelCamera};

#[derive(Default)]
pub struct ClientRenderingPlugins;

impl PluginGroup for ClientRenderingPlugins {
	fn build(self) -> PluginGroupBuilder {
		PluginGroupBuilder::start::<Self>()
			.add(VoxelRendererPlugin)
			.add(CameraVoxelLoaderPlugin)
			.add(ClientRenderingLinkPlugin)
	}
}

struct ClientRenderingLinkPlugin;

impl Plugin for ClientRenderingLinkPlugin {
	fn build(&self, app: &mut App) {
		app.add_systems(Update, ensure_voxel_camera_components.after(CameraVoxelLoaderSet::RefreshVisibility));
	}
}

fn ensure_voxel_camera_components(
	mut commands: Commands,
	mut cameras: Query<(Entity, Option<&CameraVoxelRenderState>, Option<&mut VoxelCamera>), With<Camera3d>>,
) {
	for (entity, render_state, voxel_camera) in &mut cameras {
		match (render_state, voxel_camera) {
			(Some(render_state), Some(mut voxel_camera)) => {
				voxel_camera.subgrids_to_render = render_state.subgrids_to_render.clone();
				voxel_camera.lods_to_render = render_state.lods_to_render.clone();
			}
			(Some(render_state), None) => {
				commands.entity(entity).insert(VoxelCamera {
					subgrids_to_render: render_state.subgrids_to_render.clone(),
					lods_to_render: render_state.lods_to_render.clone(),
				});
			}
			(None, Some(mut voxel_camera)) => {
				voxel_camera.subgrids_to_render.clear();
				voxel_camera.lods_to_render.clear();
			}
			(None, None) => {
				commands.entity(entity).insert(VoxelCamera::default());
			}
		}
	}
}
