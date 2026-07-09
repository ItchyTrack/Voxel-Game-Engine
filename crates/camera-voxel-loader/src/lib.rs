mod camera_voxel_loader;
mod replacement_graph;
mod loading;
mod scheduling;
mod lod_bands;
mod lod_policy;
mod subgrid_interface;
mod types;
mod unresolved_tile_index;

#[cfg(test)]
mod church_flight_invariants;

use bevy::prelude::*;
use voxel_streaming::{StreamingPhase, StreamingSchedule, VoxelStreamingAppExt};

pub use crate::camera_voxel_loader::{CameraVoxelLoader, CameraVoxelLoaderSettings};

voxel_streaming::chunk_consumer!(pub CameraVoxelLoaderConsumer);

#[derive(Component, Debug, Clone, Default)]
pub struct CameraVoxelRenderState {
	pub subgrids_to_render: Vec<Entity>,
	pub lods_to_render: Vec<Entity>,
}

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct FreezeCameraVoxelLoader(pub bool);

#[derive(Resource, Default, Debug, Clone)]
pub struct CameraVoxelLoaderDefaultSettings(pub CameraVoxelLoaderSettings);

fn ensure_camera_voxel_loader_components(
	mut commands: Commands, default_settings: Res<CameraVoxelLoaderDefaultSettings>,
	cameras: Query<(Entity, Option<&CameraVoxelLoader>, Option<&CameraVoxelLoaderConsumer>, Option<&CameraVoxelRenderState>), With<Camera3d>>,
) {
	for (entity, loader, consumer, render_state) in &cameras {
		let mut entity_commands = commands.entity(entity);
		if loader.is_none() { entity_commands.insert(CameraVoxelLoader::with_settings(default_settings.0.clone())); }
		if consumer.is_none() { entity_commands.insert(CameraVoxelLoaderConsumer::default()); }
		if render_state.is_none() { entity_commands.insert(CameraVoxelRenderState::default()); }
	}
}

#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub enum CameraVoxelLoaderSet {
	RefreshVisibility,
}

#[derive(Default)]
pub struct CameraVoxelLoaderPlugin;

impl Plugin for CameraVoxelLoaderPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<FreezeCameraVoxelLoader>()
			.init_resource::<CameraVoxelLoaderDefaultSettings>()
			.register_chunk_consumer::<CameraVoxelLoaderConsumer>()
			.add_systems(Update, ensure_camera_voxel_loader_components)
			.add_systems(Update, scheduling::update_camera_voxel_loader_requests.run_if(|freeze: Res<FreezeCameraVoxelLoader>| !freeze.0).in_set(StreamingPhase::Request))
			.add_systems(StreamingSchedule, loading::receive_camera_voxel_loader_results.after(voxel_streaming::receive_lod_results).in_set(StreamingPhase::Receive))
			.add_systems(Update, scheduling::refresh_camera_voxel_loader_visibility.after(voxel_gpu::GpuUploadSet::Upload).in_set(CameraVoxelLoaderSet::RefreshVisibility));
	}
}
