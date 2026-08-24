use bevy::{ecs::message::MessageReader, prelude::*};
use tile_data::{CHUNK_SIZE, TileBuildingParameters};
use voxel_data::grid::GridId;
use voxel_streaming::{
	ChunkAvailabilityChangeKind,
	ChunkAvailabilityChanged,
	GridStreaming,
	TileLoadStatus,
	TileLoadUpdate,
	TileRequester,
};

use crate::{
	FreezeCameraVoxelLoader,
	camera_voxel_loader::{CameraVoxelLoader, CameraVoxelTileClass},
	lod_bands::for_each_tile_in_bands,
	lod_policy::{nearest_chunk_center, tile_has_present_source, update_desired_sources_delta},
	tile_lifecycle::ResolvedTile,
	types::GridTileKey,
};

fn release_tiles(requester: &mut TileRequester, requester_entity: Entity, keys: impl IntoIterator<Item = GridTileKey>) {
	for key in keys { requester.release_tile(key.grid, requester_entity, key.tile_key); }
}

fn acquire_tile(
	requester: &mut TileRequester,
	lifecycle: &mut crate::tile_lifecycle::TileLifecycle,
	requester_entity: Entity,
	key: GridTileKey,
	priority: f32,
	context: Option<&TileBuildingParameters>,
) {
	if requester.fetch_tile(key.grid, requester_entity, key.tile_key, priority, context) { return; }
	let released = lifecycle.resolve(key, ResolvedTile::Empty);
	release_tiles(requester, requester_entity, released);
}

pub(crate) fn update_camera_voxel_loader_requests(
	freeze: Res<FreezeCameraVoxelLoader>,
	mut cameras: Query<(Entity, &Camera, &GlobalTransform, &CameraVoxelTileClass, &mut CameraVoxelLoader), With<Camera3d>>,
	mut streaming: ParamSet<(
		TileRequester,
		Query<(GridId, &GlobalTransform, &GridStreaming, Option<&TileBuildingParameters>)>,
	)>,
) {
	for (camera_entity, camera, camera_global, tile_class, mut loader) in &mut cameras {
		if !camera.is_active {
			let release: Vec<_> = loader.tiles.entries().map(|(key, _)| key).collect();
			loader.bands.clear();
			loader.classes.clear();
			loader.tiles = Default::default();
			let mut requester = streaming.p0();
			release_tiles(&mut requester, camera_entity, release);
			continue;
		}
		if freeze.0 { continue; }

		let camera_world = camera_global.translation();
		let settings = loader.settings.clone();
		let mut acquisitions = Vec::new();
		let mut releases = Vec::new();
		{
			let grids = streaming.p1();
			let mut acquire = Vec::new();
			let mut release = Vec::new();
			for (grid_id, grid_global, grid_streaming, context) in &grids {
				let camera_local = grid_global.affine().inverse().transform_point3(camera_world);
				let delta = update_desired_sources_delta(
					&mut loader,
					grid_id,
					tile_class.0,
					nearest_chunk_center(camera_local),
					&settings,
					grid_streaming,
				);
				loader.tiles.apply_delta(&delta.added, &delta.removed, &mut acquire, &mut release);
				releases.extend(release.drain(..));
				for key in acquire.drain(..) {
					let center_local = ((key.tile_key.region.min() + key.tile_key.region.size().as_ivec3() / 2) * CHUNK_SIZE as i32).as_vec3();
					let priority = -camera_world.distance(grid_global.transform_point(center_local));
					acquisitions.push((key, priority, context.cloned()));
				}
			}
		}

		let mut requester = streaming.p0();
		release_tiles(&mut requester, camera_entity, releases);
		for (key, priority, context) in acquisitions {
			if !loader.tiles.contains_source(key) { continue; }
			acquire_tile(&mut requester, &mut loader.tiles, camera_entity, key, priority, context.as_ref());
		}
	}
}

pub(crate) fn receive_camera_voxel_loader_results(
	mut results: MessageReader<TileLoadUpdate>,
	mut loaders: Query<&mut CameraVoxelLoader>,
	mut requester: TileRequester,
) {
	for result in results.read() {
		let Ok(mut loader) = loaders.get_mut(result.requester) else { continue };
		let key = GridTileKey { grid: result.grid, tile_key: result.key };
		if !loader.tiles.contains_source(key) { continue; }
		let resolution = match result.status {
			TileLoadStatus::Ready(entity) => ResolvedTile::Tile(entity),
			TileLoadStatus::Empty => ResolvedTile::Empty,
		};
		let released = loader.tiles.resolve(key, resolution);
		release_tiles(&mut requester, result.requester, released);
	}
}

pub(crate) fn refresh_camera_voxel_loader_visibility(
	mut availability_events: MessageReader<ChunkAvailabilityChanged>,
	mut cameras: Query<(Entity, &mut CameraVoxelLoader)>,
	mut streaming: ParamSet<(
		TileRequester,
		Query<(&GridStreaming, Option<&TileBuildingParameters>)>,
	)>,
) {
	let availability_events: Vec<_> = availability_events.read().copied().collect();

	for (camera_entity, mut loader) in &mut cameras {
		let mut acquisitions = Vec::new();
		let mut releases = Vec::new();
		{
			let grids = streaming.p1();
			let mut changed = Vec::new();
			let mut acquire = Vec::new();
			let mut release = Vec::new();
			for event in &availability_events {
				match event.kind {
					ChunkAvailabilityChangeKind::BecamePresent => {
						let Some(bands) = loader.bands.get(&event.grid) else { continue };
						let Some(class) = loader.classes.get(&event.grid).copied() else { continue };
						let Ok((grid_streaming, context)) = grids.get(event.grid) else { continue };
						changed.clear();
						for_each_tile_in_bands(bands, event.region, |lod, min| {
							let key = GridTileKey::new(event.grid, class, lod, min);
							if !loader.tiles.contains_desired(key) && tile_has_present_source(grid_streaming, key) { changed.push(key); }
						});
						loader.tiles.apply_delta(&changed, &[], &mut acquire, &mut release);
						releases.extend(release.drain(..));
						acquisitions.extend(acquire.drain(..).map(|key| (key, context.cloned())));
					}
					ChunkAvailabilityChangeKind::BecameEmpty => {
						let Ok((grid_streaming, _)) = grids.get(event.grid) else { continue };
						loader.tiles.desired_in_area(event.grid, event.region, &mut changed);
						changed.retain(|&key| !tile_has_present_source(grid_streaming, key));
						loader.tiles.apply_delta(&[], &changed, &mut acquire, &mut release);
						releases.extend(release.drain(..));
					}
				}
			}
		}

		let mut requester = streaming.p0();
		release_tiles(&mut requester, camera_entity, releases);
		for (key, context) in acquisitions {
			if !loader.tiles.contains_source(key) { continue; }
			acquire_tile(&mut requester, &mut loader.tiles, camera_entity, key, 0.0, context.as_ref());
		}
	}
}
