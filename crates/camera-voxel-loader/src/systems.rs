use bevy::{ecs::message::MessageReader, prelude::*};
use voxel_data::grid::GridId;
use voxel_streaming::{
	CHUNK_SIZE,
	ChunkAvailabilityChangeKind,
	ChunkAvailabilityChanged,
	ChunkConsumer,
	GridStreaming,
	TileLoadStatus,
};

use crate::{
	CameraVoxelLoaderConsumer,
	camera_voxel_loader::{CameraVoxelLoader, CameraVoxelTileClass},
	lod_bands::for_each_tile_in_bands,
	lod_policy::{nearest_chunk_center, tile_has_present_source, update_desired_sources_delta},
	tile_lifecycle::ResolvedTile,
	types::TileKey,
};

fn acquire_tile(
	lifecycle: &mut crate::tile_lifecycle::TileLifecycle,
	requester: Entity,
	key: TileKey,
	priority: f32,
	streaming: &mut GridStreaming,
) {
	if !streaming.fetch_tile(requester, key.streaming_key(), priority) {
		let _ = lifecycle.resolve(key, ResolvedTile::Empty);
	}
}

fn release_tiles(streaming: &mut GridStreaming, requester: Entity, keys: impl IntoIterator<Item = TileKey>) {
	for key in keys { streaming.release_tile(requester, key.streaming_key()); }
}

pub(crate) fn update_camera_voxel_loader_requests(
	mut cameras: Query<(Entity, &Camera, &GlobalTransform, &CameraVoxelTileClass, &mut CameraVoxelLoader), With<Camera3d>>,
	mut grids: Query<(GridId, &GlobalTransform, &mut GridStreaming)>,
) {
	for (camera_entity, camera, camera_global, tile_class, mut loader) in &mut cameras {
		if !camera.is_active { continue; }
		let camera_world = camera_global.translation();
		let settings = loader.settings.clone();

		let mut acquire = Vec::new();
		let mut release = Vec::new();
		for (grid_id, grid_global, streaming) in &mut grids {
			let camera_local = grid_global.affine().inverse().transform_point3(camera_world);
			let delta = update_desired_sources_delta(
				&mut loader,
				grid_id,
				tile_class.0,
				nearest_chunk_center(camera_local),
				&settings,
				streaming.as_ref(),
			);
			let streaming = streaming.into_inner();
			loader.tiles.apply_delta(&delta.added, &delta.removed, &mut acquire, &mut release);
			release_tiles(streaming, camera_entity, release.drain(..));

			for &key in &acquire {
				let center_local = ((key.min + key.size() / 2) * CHUNK_SIZE).as_vec3();
				let priority = -camera_world.distance(grid_global.transform_point(center_local));
				acquire_tile(&mut loader.tiles, camera_entity, key, priority, streaming);
			}
		}
	}
}

pub(crate) fn receive_camera_voxel_loader_results(
	mut loaders: Query<&mut CameraVoxelLoader>,
	mut consumers: Query<&mut CameraVoxelLoaderConsumer>,
	mut grids: Query<&mut GridStreaming>,
) {
	for mut consumer in &mut consumers {
		for result in consumer.drain_tiles() {
			let Ok(mut loader) = loaders.get_mut(result.requester) else { continue };
			let key = TileKey { grid: result.grid, class: result.key.class, lod: result.key.lod, min: result.key.min };
			if !loader.tiles.contains_source(key) { continue; }
			let Ok(streaming) = grids.get_mut(result.grid) else { continue };
			let streaming = streaming.into_inner();
			let resolution = match result.status {
				TileLoadStatus::Ready(entity) => ResolvedTile::Tile(entity),
				TileLoadStatus::Empty => ResolvedTile::Empty,
			};
			release_tiles(streaming, result.requester, loader.tiles.resolve(key, resolution));
		}
	}
}

pub(crate) fn refresh_camera_voxel_loader_visibility(
	mut availability_events: MessageReader<ChunkAvailabilityChanged>,
	mut cameras: Query<(Entity, &mut CameraVoxelLoader)>,
	mut grids: Query<&mut GridStreaming>,
) {
	let availability_events: Vec<_> = availability_events.read().copied().collect();

	for (camera_entity, mut loader) in &mut cameras {
		let mut changed = Vec::new();
		let mut acquire = Vec::new();
		let mut release = Vec::new();
		for event in &availability_events {
			match event.kind {
				ChunkAvailabilityChangeKind::BecamePresent => {
					let Some(bands) = loader.bands.get(&event.grid) else { continue };
					let Some(class) = loader.classes.get(&event.grid).copied() else { continue };
					let Ok(streaming) = grids.get_mut(event.grid) else { continue };
					let streaming = streaming.into_inner();
					changed.clear();
					for_each_tile_in_bands(bands, event.min, event.size, |lod, min| {
						let key = TileKey { grid: event.grid, class, lod, min };
						if !loader.tiles.contains_desired(key) && tile_has_present_source(streaming, key) { changed.push(key); }
					});
					loader.tiles.apply_delta(&changed, &[], &mut acquire, &mut release);
					release_tiles(streaming, camera_entity, release.drain(..));
					for &key in &acquire { acquire_tile(&mut loader.tiles, camera_entity, key, 0.0, streaming); }
				}
				ChunkAvailabilityChangeKind::BecameEmpty => {
					let Ok(streaming) = grids.get_mut(event.grid) else { continue };
					let streaming = streaming.into_inner();
					loader.tiles.desired_in_area(event.grid, event.min, event.size, &mut changed);
					changed.retain(|&key| !tile_has_present_source(streaming, key));
					loader.tiles.apply_delta(&[], &changed, &mut acquire, &mut release);
					release_tiles(streaming, camera_entity, release.drain(..));
				}
			}
		}
	}
}
