mod camera_voxel_loader;
mod types;

use std::collections::HashSet;

use bevy::prelude::*;
use gpu_voxel_data::{LodVoxels, SubGridGpuState};
use voxel_data::grid::{Grid, GridId};
use voxel_data::subgrid::SubGrid;
use voxel_renderer::voxel_camera::VoxelCamera;
use voxel_streaming::ChunkConsumer;
use voxel_streaming::{ChunkRequestChannel, GridStreaming, LodRequestChannel, StreamingPhase, StreamingSchedule, VoxelStreamingAppExt, CHUNK_SIZE};

use crate::camera_voxel_loader::CameraVoxelLoader;
use crate::types::{ChunkKey, TileKey, TileRecord, TileStatus};

voxel_streaming::chunk_consumer!(pub CameraVoxelLoaderConsumer);

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct FreezeCameraVoxelLoader(pub bool);

#[derive(Default)]
pub struct CameraVoxelLoaderPlugin;

impl Plugin for CameraVoxelLoaderPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<FreezeCameraVoxelLoader>()
			.register_chunk_consumer::<CameraVoxelLoaderConsumer>()
			.add_systems(Update, ensure_camera_voxel_loader_components)
			.add_systems(
				StreamingSchedule,
				update_camera_voxel_loader_requests.run_if(|freeze: Res<FreezeCameraVoxelLoader>| !freeze.0).in_set(StreamingPhase::Request),
			)
			.add_systems(StreamingSchedule, receive_camera_voxel_loader_results.after(voxel_streaming::receive_lod_results).in_set(StreamingPhase::Receive))
			.add_systems(
				Update,
				(refresh_camera_voxel_loader_visibility, retire_replaced_tiles, retire_replaced_chunks).chain().after(gpu_voxel_data::GpuUploadSet::Upload),
			);
			// .add_systems(Update, (draw_lod_bounds_gizmos, draw_retiring_lod_gizmos));
	}
}

fn ensure_camera_voxel_loader_components(mut commands: Commands, cameras: Query<Entity, (With<Camera3d>, Without<CameraVoxelLoader>)>) {
	for entity in &cameras {
		commands.entity(entity).insert((CameraVoxelLoader::default(), CameraVoxelLoaderConsumer::default(), VoxelCamera::default()));
	}
}

fn update_camera_voxel_loader_requests(
	chunk_channel: Res<ChunkRequestChannel>, lod_channel: Res<LodRequestChannel>,
	mut cameras: Query<(Entity, &Camera, &GlobalTransform, &mut CameraVoxelLoader, &mut CameraVoxelLoaderConsumer), With<Camera3d>>,
	mut grids: Query<(GridId, &GlobalTransform, &mut GridStreaming)>, grid_transforms: Query<&GlobalTransform, With<GridStreaming>>,
) {
	for (camera_entity, camera, camera_global, mut camera_voxel_loader, mut consumer) in &mut cameras {
		if !camera.is_active {
			continue;
		}
		let camera_world = camera_global.translation();
		let mut desired_chunks = HashSet::new();
		let mut desired_tiles = HashSet::new();

		for (grid, grid_global, streaming) in &mut grids {
			let local = grid_global.affine().inverse().transform_point3(camera_world);
			let camera_chunk = nearest_chunk_center(local);
			camera_voxel_loader.grid_centers.insert(grid, camera_chunk);
			add_near_chunks(&mut desired_chunks, grid, camera_chunk, &camera_voxel_loader);
			add_lod_tiles(&mut desired_tiles, grid, camera_chunk, &camera_voxel_loader, streaming.as_ref());
		}

		camera_voxel_loader.desired_tiles = desired_tiles.clone();

		let chunks_to_fetch = update_desired_chunks(&mut camera_voxel_loader, desired_chunks);
		for chunk_key in chunks_to_fetch {
			if let Ok((_, _, mut streaming)) = grids.get_mut(chunk_key.grid) {
				streaming.fetch_needed(chunk_key.grid, consumer.as_mut(), &chunk_channel, chunk_key.chunk);
			}
		}

		let old_tiles: Vec<TileKey> = camera_voxel_loader.tiles.keys().copied().collect();
		for key in old_tiles {
			if desired_tiles.contains(&key) {
				if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
					if record.status == TileStatus::Retiring {
						record.status = if record.entity.is_some() { TileStatus::Ready } else { TileStatus::Empty };
					}
				}
				continue;
			}

			handle_non_desired_tile(&mut camera_voxel_loader, key);
		}

		for key in desired_tiles {
			if camera_voxel_loader.tiles.contains_key(&key) {
				continue;
			}
			camera_voxel_loader.tiles.insert(key, TileRecord { status: TileStatus::Queued, entity: None });
			camera_voxel_loader.queue.push_back(key);
		}

		let mut sent = 0usize;
		while sent < camera_voxel_loader.settings.requests_per_frame && in_flight_count(&camera_voxel_loader) < camera_voxel_loader.settings.max_in_flight {
			let Some(key) = camera_voxel_loader.queue.pop_front() else {
				break;
			};
			if !matches!(camera_voxel_loader.tiles.get(&key).map(|r| r.status), Some(TileStatus::Queued)) {
				continue;
			}
			let priority = tile_priority(camera_world, key, &grid_transforms);
			let requested = grids
				.get(key.grid)
				.map(|(_, _, streaming)| streaming.fetch_lod(key.grid, camera_entity, &lod_channel, key.min, key.size(), key.lod as f32, priority))
				.unwrap_or(false);
			if requested {
				if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
					record.status = TileStatus::Loading;
				}
				sent += 1;
			} else if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
				record.status = TileStatus::Empty;
			}
		}
	}
}

fn nearest_chunk_center(local_voxels: Vec3) -> IVec3 {
	(local_voxels / CHUNK_SIZE as f32).round().as_ivec3()
}

fn add_near_chunks(out: &mut HashSet<ChunkKey>, grid: GridId, center: IVec3, camera_voxel_loader: &CameraVoxelLoader) {
	let r = camera_voxel_loader.settings.near_radius_chunks;
	for x in -r..=r {
		for y in -r..=r {
			for z in -r..=r {
				out.insert(ChunkKey { grid, chunk: center + IVec3::new(x, y, z) });
			}
		}
	}
}

fn add_lod_tiles(out: &mut HashSet<TileKey>, grid: GridId, center: IVec3, camera_voxel_loader: &CameraVoxelLoader, streaming: &GridStreaming) {
	if presence_bounds(streaming).is_none() { return; }

	let mut inner = camera_voxel_loader.settings.near_radius_chunks + 1;
	for lod in 1..=camera_voxel_loader.settings.max_lod {
		let tile_size = 1i32 << lod;
		let outer = inner + camera_voxel_loader.settings.rings_per_lod * tile_size;
		let lod_min = center - IVec3::splat(outer);
		let lod_max = center + IVec3::splat(outer);
		let min_tile = align_chunk_to_tile(lod_min, tile_size);
		let max_tile = align_chunk_to_tile(lod_max, tile_size);
		let band_inner = if lod == 1 { 0 } else { inner };
		for x in (min_tile.x..=max_tile.x).step_by(tile_size as usize) {
			for y in (min_tile.y..=max_tile.y).step_by(tile_size as usize) {
				for z in (min_tile.z..=max_tile.z).step_by(tile_size as usize) {
					let min = IVec3::new(x, y, z);
					if tile_intersects_ring(center, min, tile_size, band_inner, outer)
						&& !tile_inside_near_box(center, min, tile_size, camera_voxel_loader.settings.near_radius_chunks)
						&& tile_has_present_chunk(streaming, min, tile_size)
					{
						out.insert(TileKey { grid, lod, min });
					}
				}
			}
		}
		inner = outer + 1;
	}
}

fn presence_bounds(streaming: &GridStreaming) -> Option<(IVec3, IVec3)> {
	let mut min = IVec3::splat(i32::MAX);
	let mut max = IVec3::splat(i32::MIN);
	let mut any = false;
	for (origin, size) in streaming.presence().iter_present() {
		any = true;
		min = min.min(origin);
		max = max.max(origin + IVec3::splat(size as i32) - IVec3::ONE);
	}
	any.then_some((min, max))
}

fn tile_has_present_chunk(streaming: &GridStreaming, min: IVec3, tile_size: i32) -> bool {
	let max = min + IVec3::splat(tile_size) - IVec3::ONE;
	let mut any = false;
	streaming.presence().for_each_in_region(min, max, |_| {
		any = true;
	});
	any
}

fn align_chunk_to_tile(chunk: IVec3, tile_size: i32) -> IVec3 {
	chunk.div_euclid(IVec3::splat(tile_size)) * tile_size
}

fn axis_min_abs_delta(center: i32, lo: i32, hi: i32) -> i32 {
	if center < lo {
		lo - center
	} else if center > hi {
		center - hi
	} else {
		0
	}
}

fn tile_intersects_ring(center: IVec3, min: IVec3, tile_size: i32, inner: i32, outer: i32) -> bool {
	let max = min + IVec3::splat(tile_size - 1);
	let min_dist = axis_min_abs_delta(center.x, min.x, max.x)
		.max(axis_min_abs_delta(center.y, min.y, max.y))
		.max(axis_min_abs_delta(center.z, min.z, max.z));
	let max_dist = (min.x - center.x)
		.abs()
		.max((max.x - center.x).abs())
		.max((min.y - center.y).abs())
		.max((max.y - center.y).abs())
		.max((min.z - center.z).abs())
		.max((max.z - center.z).abs());
	max_dist >= inner && min_dist <= outer
}

fn tile_inside_near_box(center: IVec3, min: IVec3, tile_size: i32, near_radius: i32) -> bool {
	let max = min + IVec3::splat(tile_size - 1);
	(min.x - center.x).abs().max((max.x - center.x).abs()) <= near_radius
		&& (min.y - center.y).abs().max((max.y - center.y).abs()) <= near_radius
		&& (min.z - center.z).abs().max((max.z - center.z).abs()) <= near_radius
}

fn in_flight_count(camera_voxel_loader: &CameraVoxelLoader) -> usize {
	camera_voxel_loader.tiles.values().filter(|r| r.status == TileStatus::Loading).count()
}

fn update_desired_chunks(camera_voxel_loader: &mut CameraVoxelLoader, desired_chunks: HashSet<ChunkKey>) -> Vec<ChunkKey> {
	let old_chunks = camera_voxel_loader.desired_chunks.clone();
	for &chunk_key in old_chunks.difference(&desired_chunks) {
		camera_voxel_loader.retiring_chunks.insert(chunk_key);
	}

	let chunks_to_fetch: Vec<ChunkKey> = desired_chunks.difference(&old_chunks).copied().collect();
	for chunk_key in &chunks_to_fetch {
		camera_voxel_loader.retiring_chunks.remove(chunk_key);
	}
	camera_voxel_loader.desired_chunks = desired_chunks;
	chunks_to_fetch
}

fn handle_non_desired_tile(camera_voxel_loader: &mut CameraVoxelLoader, key: TileKey) {
	let Some(record) = camera_voxel_loader.tiles.get_mut(&key) else { return };
	match record.status {
		// Already-renderable tiles must stay visible until replacement coverage is ready.
		TileStatus::Ready | TileStatus::Retiring => {
			record.status = TileStatus::Retiring;
		}
		// In-flight work may still produce valid voxel data. Keep the record so the
		// late result can be applied, then retire it through the same no-gap path.
		TileStatus::Loading | TileStatus::LoadedWaitingGpu => {}
		// Queued work has not been sent yet, so it can be safely dropped.
		TileStatus::Queued => {
			camera_voxel_loader.tiles.remove(&key);
		}
		// Empty records render nothing. Drop them instead of treating them as coverage.
		TileStatus::Empty => {
			camera_voxel_loader.tiles.remove(&key);
		}
	}
}

fn tile_priority(camera_world: Vec3, key: TileKey, grid_transforms: &Query<&GlobalTransform, With<GridStreaming>>) -> f32 {
	let Ok(grid_global) = grid_transforms.get(key.grid) else { return 0.0 };
	let center_local = ((key.min + key.size() / 2) * CHUNK_SIZE).as_vec3();
	let center_world = grid_global.transform_point(center_local);
	-camera_world.distance(center_world)
}

fn receive_camera_voxel_loader_results(
	mut commands: Commands,
	mut camera_voxel_loaders: Query<&mut CameraVoxelLoader>,
	mut consumers: Query<&mut CameraVoxelLoaderConsumer>,
	grid_transforms: Query<&GlobalTransform, With<GridStreaming>>,
) {
	for mut consumer in &mut consumers {
		let results = consumer.drain_lod();
		if results.is_empty() {
			continue;
		}
		// The consumer lives on the same entity as the controller.
		// Query iteration order is not reliable, so use the requester embedded in each result.
		for result in results {
			let Ok(mut camera_voxel_loader) = camera_voxel_loaders.get_mut(result.requester) else { continue };
			let key = TileKey { grid: result.grid, lod: result.lod.max(0.0).floor() as u8, min: result.min };
			let Some(record) = camera_voxel_loader.tiles.get_mut(&key) else { continue };
			if record.entity.is_some() {
				continue;
			}
			match result.voxels {
				Some(voxels) if !voxels.is_empty() => {
					let Ok(grid_transform) = grid_transforms.get(result.grid) else { continue };
					let world_transform = grid_transform.compute_transform() * Transform::from_translation((result.min * CHUNK_SIZE).as_vec3());
					let entity = commands
						.spawn((
							LodVoxels { voxels, world_transform, lod: result.lod, priority: result.priority },
							ChildOf(result.grid),
						))
						.id();
					record.entity = Some(entity);
					record.status = TileStatus::LoadedWaitingGpu;
				}
				_ => {
					record.status = TileStatus::Empty;
				}
			}
		}
	}
}

fn refresh_camera_voxel_loader_visibility(
	mut cameras: Query<(&mut CameraVoxelLoader, &mut VoxelCamera)>, lod_gpu: Query<&SubGridGpuState, With<LodVoxels>>, grids: Query<&Grid>,
	subgrids: Query<&SubGrid>,
) {
	for (mut camera_voxel_loader, mut request_map) in &mut cameras {
		for record in camera_voxel_loader.tiles.values_mut() {
			if record.status == TileStatus::LoadedWaitingGpu && record.entity.is_some_and(|e| lod_gpu.get(e).is_ok()) {
				record.status = TileStatus::Ready;
			}
		}

		let mut subgrids_to_render = Vec::new();
		let mut lods_to_render = Vec::new();
		let mut seen_subgrids = HashSet::new();
		let mut seen_lods = HashSet::new();
		for chunk in camera_voxel_loader.desired_chunks.iter().chain(camera_voxel_loader.retiring_chunks.iter()) {
			let Ok(grid) = grids.get(chunk.grid) else { continue };
			let min = chunk.chunk * CHUNK_SIZE;
			for entity in grid.subgrid_entities_in_area(min, IVec3::splat(CHUNK_SIZE)) {
				if subgrids.get(entity).is_ok() && seen_subgrids.insert(entity) {
					subgrids_to_render.push(entity);
				}
			}
		}

		for (key, record) in &camera_voxel_loader.tiles {
			let desired_or_retiring = record.status == TileStatus::Retiring || !matches!(record.status, TileStatus::Queued | TileStatus::Loading);
			if !desired_or_retiring {
				continue;
			}
			if record.status == TileStatus::Ready {
				if let Some(entity) = record.entity {
					if seen_lods.insert(entity) {
						lods_to_render.push(entity);
					}
				}
			} else if record.status == TileStatus::Retiring {
				if let Some(entity) = record.entity {
					if seen_lods.insert(entity) {
						lods_to_render.push(entity);
					}
				}
			}
			let _ = key;
		}
		request_map.subgrids_to_render = subgrids_to_render;
		request_map.lods_to_render = lods_to_render;
	}
}

fn retire_replaced_tiles(mut commands: Commands, mut camera_voxel_loaders: Query<&mut CameraVoxelLoader>) {
	for mut camera_voxel_loader in &mut camera_voxel_loaders {
		let retiring: Vec<TileKey> =
			camera_voxel_loader.tiles.iter().filter_map(|(key, record)| (record.status == TileStatus::Retiring).then_some(*key)).collect();
		for key in retiring {
			if !area_is_covered_by_ready_desired(&camera_voxel_loader, key) {
				continue;
			}
			if let Some(record) = camera_voxel_loader.tiles.remove(&key) {
				if let Some(entity) = record.entity {
					commands.entity(entity).despawn();
				}
			}
		}
	}
}

fn retire_replaced_chunks(mut camera_voxel_loaders: Query<(&mut CameraVoxelLoader, &mut CameraVoxelLoaderConsumer)>, mut grids: Query<&mut GridStreaming>) {
	for (mut camera_voxel_loader, mut consumer) in &mut camera_voxel_loaders {
		let max_lod = camera_voxel_loader.settings.max_lod;
		let retiring = std::mem::take(&mut camera_voxel_loader.retiring_chunks);
		let mut still_retiring = HashSet::new();

		for chunk in retiring {
			if retiring_chunk_still_needs_high_res(&camera_voxel_loader, chunk, max_lod) {
				still_retiring.insert(chunk);
				continue;
			}
			if let Ok(mut streaming) = grids.get_mut(chunk.grid) {
				streaming.release_needed(chunk.grid, consumer.as_mut(), chunk.chunk);
			}
		}

		camera_voxel_loader.retiring_chunks = still_retiring;
	}
}

fn retiring_chunk_still_needs_high_res(camera_voxel_loader: &CameraVoxelLoader, chunk: ChunkKey, max_lod: u8) -> bool {
	chunk_is_wanted_by_desired_tile(camera_voxel_loader, chunk, max_lod) && !chunk_is_covered_by_renderable_tile(camera_voxel_loader, chunk, max_lod)
}

fn chunk_is_wanted_by_desired_tile(camera_voxel_loader: &CameraVoxelLoader, chunk: ChunkKey, max_lod: u8) -> bool {
	(1..=max_lod).any(|lod| {
		let tile_size = 1i32 << lod;
		let key = TileKey { grid: chunk.grid, lod, min: align_chunk_to_tile(chunk.chunk, tile_size) };
		camera_voxel_loader.desired_tiles.contains(&key)
	})
}

fn chunk_is_covered_by_renderable_tile(camera_voxel_loader: &CameraVoxelLoader, chunk: ChunkKey, max_lod: u8) -> bool {
	(1..=max_lod).any(|lod| {
		let tile_size = 1i32 << lod;
		let key = TileKey { grid: chunk.grid, lod, min: align_chunk_to_tile(chunk.chunk, tile_size) };
		camera_voxel_loader.tiles.get(&key).is_some_and(|record| matches!(record.status, TileStatus::Ready | TileStatus::Retiring) && record.entity.is_some())
	})
}

fn draw_lod_bounds_gizmos(
	mut gizmos: Gizmos,
	camera_voxel_loaders: Query<&CameraVoxelLoader, With<Camera3d>>,
	grids: Query<(GridId, &GlobalTransform), With<GridStreaming>>,
) {
	for camera_voxel_loader in &camera_voxel_loaders {
		for (grid, grid_transform) in &grids {
			if let Some((min_chunk, max_chunk)) = lod0_chunk_bounds(camera_voxel_loader, grid) {
				draw_lod_bound_box(&mut gizmos, grid_transform, min_chunk, max_chunk, lod_bound_color(0, camera_voxel_loader.settings.max_lod));
			}

			for key in camera_voxel_loader.desired_tiles.iter().filter(|key| key.grid == grid) {
				draw_lod_bound_box(&mut gizmos, grid_transform, key.min, key.min + key.size(), lod_bound_color(key.lod, camera_voxel_loader.settings.max_lod));
			}
		}
	}
}

fn lod0_chunk_bounds(camera_voxel_loader: &CameraVoxelLoader, grid: GridId) -> Option<(IVec3, IVec3)> {
	let mut min = IVec3::splat(i32::MAX);
	let mut max = IVec3::splat(i32::MIN);
	let mut any = false;
	for chunk in camera_voxel_loader.desired_chunks.iter().filter(|chunk| chunk.grid == grid) {
		any = true;
		min = min.min(chunk.chunk);
		max = max.max(chunk.chunk + IVec3::ONE);
	}
	any.then_some((min, max))
}

fn draw_lod_bound_box(gizmos: &mut Gizmos, grid_transform: &GlobalTransform, min_chunk: IVec3, max_chunk: IVec3, color: Color) {
	draw_transformed_box(
		gizmos,
		grid_transform,
		(min_chunk * CHUNK_SIZE).as_vec3(),
		(max_chunk * CHUNK_SIZE).as_vec3(),
		color,
	);
}

fn lod_bound_color(lod: u8, max_lod: u8) -> Color {
	let t = if max_lod == 0 { 0.0 } else { lod as f32 / max_lod as f32 };
	Color::srgba(0.05 + 0.25 * t, 0.20 + 0.45 * t, 1.0, 0.45 + 0.35 * (1.0 - t))
}

#[cfg(test)]
fn lod_outer_radius_chunks(camera_voxel_loader: &CameraVoxelLoader) -> i32 {
	let mut inner = camera_voxel_loader.settings.near_radius_chunks + 1;
	let mut outer = inner;
	for lod in 1..=camera_voxel_loader.settings.max_lod {
		outer = inner + camera_voxel_loader.settings.rings_per_lod * (1i32 << lod);
		inner = outer + 1;
	}
	outer
}

fn draw_retiring_lod_gizmos(
	mut gizmos: Gizmos,
	cameras: Query<&CameraVoxelLoader>,
	grid_transforms: Query<&GlobalTransform, With<GridStreaming>>,
) {
	let color = Color::srgba(1.0, 0.15, 0.0, 0.9);
	for camera_voxel_loader in &cameras {
		for (key, record) in &camera_voxel_loader.tiles {
			if record.status != TileStatus::Retiring {
				continue;
			}
			let Ok(grid_transform) = grid_transforms.get(key.grid) else { continue };
			let min = (key.min * CHUNK_SIZE).as_vec3();
			let max = ((key.min + key.size()) * CHUNK_SIZE).as_vec3();
			draw_transformed_box(&mut gizmos, grid_transform, min, max, color);
		}
	}
}

fn draw_transformed_box(gizmos: &mut Gizmos, transform: &GlobalTransform, min: Vec3, max: Vec3, color: Color) {
	let corner = |x: f32, y: f32, z: f32| transform.transform_point(Vec3::new(x, y, z));
	let c000 = corner(min.x, min.y, min.z);
	let c100 = corner(max.x, min.y, min.z);
	let c010 = corner(min.x, max.y, min.z);
	let c001 = corner(min.x, min.y, max.z);
	let c110 = corner(max.x, max.y, min.z);
	let c101 = corner(max.x, min.y, max.z);
	let c011 = corner(min.x, max.y, max.z);
	let c111 = corner(max.x, max.y, max.z);

	for (a, b) in [
		(c000, c100),
		(c010, c110),
		(c001, c101),
		(c011, c111),
		(c000, c010),
		(c100, c110),
		(c001, c011),
		(c101, c111),
		(c000, c001),
		(c100, c101),
		(c010, c011),
		(c110, c111),
	] {
		gizmos.line(a, b, color);
	}
}

fn area_is_covered_by_ready_desired(state: &CameraVoxelLoader, old: TileKey) -> bool {
	let max_lod = state.tiles.keys().map(|key| key.lod).max().unwrap_or(old.lod);
	let old_min = old.min;
	let old_max = old.min + old.size();
	for x in old_min.x..old_max.x {
		for y in old_min.y..old_max.y {
			for z in old_min.z..old_max.z {
				let chunk = IVec3::new(x, y, z);
				if state.desired_chunks.contains(&ChunkKey { grid: old.grid, chunk }) {
					continue;
				}
				let chunk_key = ChunkKey { grid: old.grid, chunk };
				if !chunk_is_wanted_by_non_retiring_tile_except(state, chunk_key, old) {
					continue;
				}
				if !chunk_is_covered_by_ready_tile_except(state, chunk_key, old, max_lod) {
					return false;
				}
			}
		}
	}
	true
}

fn chunk_is_covered_by_ready_tile_except(state: &CameraVoxelLoader, chunk: ChunkKey, except: TileKey, max_lod: u8) -> bool {
	(1..=max_lod).any(|lod| {
		let tile_size = 1i32 << lod;
		let key = TileKey { grid: chunk.grid, lod, min: align_chunk_to_tile(chunk.chunk, tile_size) };
		key != except && state.tiles.get(&key).is_some_and(|record| record.status == TileStatus::Ready)
	})
}

fn chunk_is_wanted_by_non_retiring_tile_except(state: &CameraVoxelLoader, chunk: ChunkKey, except: TileKey) -> bool {
	state.tiles.iter().any(|(key, record)| {
		*key != except
			&& key.grid == chunk.grid
			&& record.status != TileStatus::Retiring
			&& chunk.chunk.cmpge(key.min).all()
			&& chunk.chunk.cmplt(key.min + key.size()).all()
	})
}
