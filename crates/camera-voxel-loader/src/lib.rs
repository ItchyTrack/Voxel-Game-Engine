mod camera_voxel_loader;
mod coverage;
mod loading;
mod lod_policy;
mod types;

use bevy::prelude::*;
use voxel_renderer::voxel_camera::VoxelCamera;
use voxel_streaming::{StreamingPhase, StreamingSchedule, VoxelStreamingAppExt};

pub use crate::camera_voxel_loader::{CameraVoxelLoader, CameraVoxelLoaderSettings};

voxel_streaming::chunk_consumer!(pub CameraVoxelLoaderConsumer);

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct FreezeCameraVoxelLoader(pub bool);

#[derive(Resource, Debug, Clone)]
pub struct CameraVoxelLoaderDefaultSettings(pub CameraVoxelLoaderSettings);

impl Default for CameraVoxelLoaderDefaultSettings {
	fn default() -> Self {
		Self(CameraVoxelLoaderSettings::default())
	}
}

fn ensure_camera_voxel_loader_components(
	mut commands: Commands,
	default_settings: Res<CameraVoxelLoaderDefaultSettings>,
	cameras: Query<
		(Entity, Option<&CameraVoxelLoader>, Option<&CameraVoxelLoaderConsumer>, Option<&VoxelCamera>),
		With<Camera3d>,
	>,
) {
	for (entity, loader, consumer, voxel_camera) in &cameras {
		let mut entity_commands = commands.entity(entity);
		if loader.is_none() {
			entity_commands.insert(CameraVoxelLoader::with_settings(default_settings.0.clone()));
		}
		if consumer.is_none() {
			entity_commands.insert(CameraVoxelLoaderConsumer::default());
		}
		if voxel_camera.is_none() {
			entity_commands.insert(VoxelCamera::default());
		}
	}
}

#[derive(Default)]
pub struct CameraVoxelLoaderPlugin;

impl Plugin for CameraVoxelLoaderPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<FreezeCameraVoxelLoader>()
			.init_resource::<CameraVoxelLoaderDefaultSettings>()
			.register_chunk_consumer::<CameraVoxelLoaderConsumer>()
			.add_systems(Update, ensure_camera_voxel_loader_components)
			.add_systems(
				Update,
				loading::update_camera_voxel_loader_requests
					.run_if(|freeze: Res<FreezeCameraVoxelLoader>| !freeze.0)
					.in_set(StreamingPhase::Request),
			)
			.add_systems(
				StreamingSchedule,
				loading::receive_camera_voxel_loader_results
					.after(voxel_streaming::receive_lod_results)
					.in_set(StreamingPhase::Receive),
			)
			.add_systems(
				Update,
				(
					loading::refresh_camera_voxel_loader_visibility,
					loading::retire_replaced_tiles,
					loading::retire_replaced_chunks,
				)
					.chain()
					.after(gpu_voxel_data::GpuUploadSet::Upload),
			)
			;
	}
}
