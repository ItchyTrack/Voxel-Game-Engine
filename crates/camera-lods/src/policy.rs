use std::collections::{HashMap, HashSet};

use bevy::prelude::*;
use gpu_voxel_data::residency::ResidencyBuffers;
use gpu_voxel_data::LodVoxels;
use lod_manager::{LodKey, LodRequestMap};
use voxel_data::grid::{Grid, GridId};
use voxel_streaming::{chunk_origin, ChunkRequestChannel, GridStreaming, CHUNK_SIZE};

use crate::debug::FreezeCameraLods;
use crate::grid_control::CameraLodGridControl;
use crate::render_set::CameraVoxelRenderSet;

/// Coarsest LOD level requested.
const MAX_LOD: u32 = 3;
/// Chunks within this camera distance render at full detail; LOD takes over beyond it.
pub const FULL_DETAIL_CHUNKS: f32 = 2.0;
const FULL_DETAIL_RADIUS: f32 = FULL_DETAIL_CHUNKS * CHUNK_SIZE as f32;
const LOD_TILE_BUDGET_PER_GRID_PER_FRAME: usize = 16;

fn ring_outer_chunks(lod: u32) -> f32 {
	FULL_DETAIL_CHUNKS * (1u32 << lod) as f32
}

pub fn max_lod_radius_chunks() -> f32 {
	ring_outer_chunks(MAX_LOD)
}

fn lod_for_distance(distance: f32) -> Option<u32> {
	if distance < FULL_DETAIL_CHUNKS { return None; }
	(1..=MAX_LOD).find(|&lod| distance < ring_outer_chunks(lod))
}

/// Declarative policy input for camera LODs.
#[derive(Component, Default, Debug, Clone)]
pub struct CameraLodPolicy {
	desired: Vec<CameraLodTarget>,
}

#[derive(Component, Default, Debug, Clone)]
pub struct CameraLazyLodScan {
	generation: u64,
	grids: HashMap<GridId, LazyGridScan>,
	seen_generation: HashMap<LodKey, u64>,
}

#[derive(Default, Debug, Clone)]
struct LazyGridScan {
	camera_chunk: Vec3,
	tiles: Vec<LazyLodTile>,
	cursor: usize,
	complete_this_generation: bool,
}

#[derive(Clone, Copy, Debug)]
struct LazyLodTile {
	level: u32,
	min: IVec3,
	size: IVec3,
	priority: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CameraLodTarget {
	pub key: LodKey,
	pub priority: f32,
}

impl CameraLodPolicy {
	pub fn clear(&mut self) {
		self.desired.clear();
	}

	pub fn set_lod(&mut self, grid: GridId, min: IVec3, size: IVec3, level: u32, priority: f32) {
		let key = LodKey::from_level(grid, min, size, level);
		if let Some(existing) = self.desired.iter_mut().find(|target| target.key == key) {
			existing.priority = priority;
			return;
		}
		self.desired.push(CameraLodTarget { key, priority });
	}

	pub fn remove_lod(&mut self, key: LodKey) {
		self.desired.retain(|target| target.key != key);
	}

	pub fn retain_lods(&mut self, mut keep: impl FnMut(&LodKey) -> bool) {
		self.desired.retain(|target| keep(&target.key));
	}

	pub fn targets(&self) -> &[CameraLodTarget] {
		&self.desired
	}
}

#[derive(Component, Default)]
pub(crate) struct CameraWantedChunks(HashMap<GridId, HashSet<IVec3>>);

fn lod_covered_chunks(
	lod_tiles: &Query<(Entity, &LodVoxels)>,
	grid: GridId,
	residency: &ResidencyBuffers,
) -> HashSet<IVec3> {
	let mut covered = HashSet::new();
	for (entity, tile) in lod_tiles.iter() {
		if tile.grid != grid || !residency.offsets().contains_key(&entity) { continue; }
		for z in 0..tile.size.z {
			for y in 0..tile.size.y {
				for x in 0..tile.size.x {
					covered.insert(tile.min + IVec3::new(x, y, z));
				}
			}
		}
	}
	covered
}

fn aligned_floor(v: IVec3, step: i32) -> IVec3 {
	v.div_euclid(IVec3::splat(step)) * step
}

fn tile_has_present(streaming: &GridStreaming, min: IVec3, size: IVec3) -> bool {
	for z in 0..size.z {
		for y in 0..size.y {
			for x in 0..size.x {
				if streaming.presence().is_present(min + IVec3::new(x, y, z)) {
					return true;
				}
			}
		}
	}
	false
}

fn min_tile_chunks_for_lod(level: u32) -> i32 {
	let factor = 1i32 << level;
	((factor + CHUNK_SIZE - 1) / CHUNK_SIZE).max(1)
}

fn build_lazy_tiles(camera_chunk: Vec3) -> Vec<LazyLodTile> {
	let max_radius = ring_outer_chunks(MAX_LOD);
	let region_min = (camera_chunk - max_radius).floor().as_ivec3();
	let region_max = (camera_chunk + max_radius).ceil().as_ivec3();
	let mut tiles = Vec::new();
	let mut emitted = HashSet::new();

	// Classify by chunk AABB, then emit the smallest valid request tile for that LOD.
	// This fixes ring-boundary gaps: large aligned tiles can straddle a lower-LOD
	// boundary, but each chunk is classified independently before being folded into
	// the minimum tile size that `GridStreaming::fetch_lod` accepts.
	for z in region_min.z..=region_max.z {
		for y in region_min.y..=region_max.y {
			for x in region_min.x..=region_max.x {
				let chunk = IVec3::new(x, y, z);
				let distance = camera_chunk.distance(camera_chunk.clamp(chunk.as_vec3(), (chunk + IVec3::ONE).as_vec3()));
				let Some(level) = lod_for_distance(distance) else { continue; };

				let tile_step = min_tile_chunks_for_lod(level);
				let tile_min = aligned_floor(chunk, tile_step);
				let tile_size = IVec3::splat(tile_step);
				if !emitted.insert((level, tile_min, tile_size)) { continue; }

				let nearest = camera_chunk.clamp(tile_min.as_vec3(), (tile_min + tile_size).as_vec3());
				let priority = -camera_chunk.distance(nearest) * CHUNK_SIZE as f32;
				tiles.push(LazyLodTile { level, min: tile_min, size: tile_size, priority });
			}
		}
	}

	tiles.sort_by(|a, b| b.priority.total_cmp(&a.priority));
	tiles
}

fn lod_key_still_near_camera(key: &LodKey, camera_chunks: &HashMap<GridId, Vec3>) -> bool {
	let Some(&camera_chunk) = camera_chunks.get(&key.grid) else { return false; };
	let nearest = camera_chunk.clamp(key.min.as_vec3(), (key.min + key.size).as_vec3());
	let distance = camera_chunk.distance(nearest);
	lod_for_distance(distance) == Some(key.level)
}

fn reset_lazy_scan(scan: &mut CameraLazyLodScan, camera_chunks: &HashMap<GridId, Vec3>) {
	scan.generation = scan.generation.wrapping_add(1).max(1);
	for (&grid, &camera_chunk) in camera_chunks {
		scan.grids.insert(grid, LazyGridScan {
			camera_chunk,
			tiles: build_lazy_tiles(camera_chunk),
			cursor: 0,
			complete_this_generation: false,
		});
	}
	scan.grids.retain(|grid, _| camera_chunks.contains_key(grid));
}

/// High-level camera policy: lazily scans present chunks and writes desired LOD tiles.
///
/// The LOD radius can cover hundreds of thousands of chunks. Instead of rebuilding the
/// whole request set every frame, this advances a small cursor budget each frame and
/// keeps previous decisions until a full lazy pass proves they are stale.
pub fn update_camera_lod_policy(
	freeze: Res<FreezeCameraLods>,
	mut cameras: Query<(&Camera, &GlobalTransform, &mut CameraLodPolicy, &mut CameraLazyLodScan)>,
	grids: Query<(GridId, &GlobalTransform, &GridStreaming)>,
) {
	if freeze.0 { return; }
	let Some((_, camera_transform, mut policy, mut scan)) = cameras.iter_mut().find(|(camera, _, _, _)| camera.is_active) else { return };
	let camera_world = camera_transform.translation();

	let mut camera_chunks = HashMap::new();
	for (grid, grid_global, _) in grids.iter() {
		let camera_local = grid_global.affine().inverse().transform_point3(camera_world);
		camera_chunks.insert(grid, camera_local / CHUNK_SIZE as f32);
	}

	let needs_reset = scan.grids.len() != camera_chunks.len()
		|| camera_chunks.iter().any(|(grid, camera_chunk)| {
			scan.grids
				.get(grid)
				.is_none_or(|state| state.camera_chunk.distance(*camera_chunk) >= 0.5)
		});
	if needs_reset || scan.generation == 0 {
		// Do not keep the old camera position's LOD requests alive until a full lazy
		// pass completes. Those stale requests compete with the new view in the loader
		// queue, which makes the world feel like it updates seconds late after moving.
		policy.retain_lods(|key| lod_key_still_near_camera(key, &camera_chunks));
		scan.seen_generation.retain(|key, _| lod_key_still_near_camera(key, &camera_chunks));
		reset_lazy_scan(&mut scan, &camera_chunks);
	}

	let generation = scan.generation;
	let mut completed_any = false;

	let mut seen_this_frame = Vec::new();
	for (grid, _, streaming) in grids.iter() {
		let mut remaining = LOD_TILE_BUDGET_PER_GRID_PER_FRAME;
		let Some(state) = scan.grids.get_mut(&grid) else { continue };
		if state.complete_this_generation { continue; }

		while remaining > 0 && state.cursor < state.tiles.len() {
			remaining -= 1;
			let tile = state.tiles[state.cursor];
			state.cursor += 1;
			if !tile_has_present(streaming, tile.min, tile.size) { continue; }

			let key = LodKey::from_level(grid, tile.min, tile.size, tile.level);
			policy.set_lod(grid, tile.min, tile.size, tile.level, tile.priority);
			seen_this_frame.push(key);
		}

		if state.cursor >= state.tiles.len() {
			state.complete_this_generation = true;
			completed_any = true;
		}
	}

	for key in seen_this_frame {
		scan.seen_generation.insert(key, generation);
	}

	if completed_any && scan.grids.values().all(|state| state.complete_this_generation) {
		policy.retain_lods(|key| scan.seen_generation.get(key).copied() == Some(generation));
		scan.seen_generation.retain(|_, seen| *seen == generation);
		reset_lazy_scan(&mut scan, &camera_chunks);
	}
}

/// High-level full-resolution policy: request chunks close to the active camera and
/// keep the camera render whitelist's sub-grid half in sync.
pub(crate) fn update_camera_full_res_chunks(
	mut commands: Commands,
	freeze: Res<FreezeCameraLods>,
	channel: Res<ChunkRequestChannel>,
	mut cameras: Query<(Entity, &Camera, &GlobalTransform, Option<&mut CameraVoxelRenderSet>)>,
	mut grids: Query<(GridId, &GlobalTransform, &mut GridStreaming, &Grid, Option<&mut CameraWantedChunks>)>,
	lod_tiles: Query<(Entity, &LodVoxels)>,
	residency: Res<ResidencyBuffers>,
) {
	if freeze.0 { return; }
	let Some((camera_entity, _, camera_transform, render_set)) = cameras.iter_mut().find(|(_, camera, _, _)| camera.is_active) else { return };
	let camera_world = camera_transform.translation();
	let mut visible_subgrids = Vec::new();

	for (grid, grid_global, mut streaming, grid_data, wanted) in grids.iter_mut() {
		let local = grid_global.affine().inverse().transform_point3(camera_world);
		let cmin = ((local - FULL_DETAIL_RADIUS) / CHUNK_SIZE as f32).floor().as_ivec3();
		let cmax = ((local + FULL_DETAIL_RADIUS) / CHUNK_SIZE as f32).floor().as_ivec3();

		let mut want = HashSet::new();
		streaming.presence().for_each_in_region(cmin, cmax, |chunk| {
			let lo = (chunk * CHUNK_SIZE).as_vec3();
			let hi = ((chunk + IVec3::ONE) * CHUNK_SIZE).as_vec3();
			if local.distance(local.clamp(lo, hi)) <= FULL_DETAIL_RADIUS {
				want.insert(chunk);
			}
		});

		let prev = wanted.as_deref().and_then(|w| w.0.get(&grid));
		let empty = HashSet::new();
		let prev = prev.unwrap_or(&empty);
		for &chunk in want.difference(prev) {
			streaming.fetch(grid, &channel, chunk);
		}
		let leaving: Vec<_> = prev.difference(&want).copied().collect();
		let mut held = want;
		if !leaving.is_empty() {
			let lod_covered = lod_covered_chunks(&lod_tiles, grid, &residency);
			let camera_chunk = local / CHUNK_SIZE as f32;
			let max_radius = max_lod_radius_chunks();
			for chunk in leaving {
				let lo = chunk.as_vec3();
				let hi = (chunk + IVec3::ONE).as_vec3();
				let distance = camera_chunk.distance(camera_chunk.clamp(lo, hi));
				if lod_covered.contains(&chunk) || distance >= max_radius {
					streaming.release(chunk);
				} else {
					held.insert(chunk);
				}
			}
		}

		for &chunk in &held {
			let min = chunk_origin(chunk);
			visible_subgrids.extend(grid_data.subgrid_entities_in_area(min, IVec3::splat(CHUNK_SIZE)));
		}

		match wanted {
			Some(mut wanted) => {
				wanted.0.insert(grid, held);
			}
			None => {
				commands.entity(grid).insert(CameraWantedChunks(HashMap::from([(grid, held)])));
			}
		}
	}

	match render_set {
		Some(mut render_set) => render_set.replace_subgrids(visible_subgrids),
		None => {
			let mut set = CameraVoxelRenderSet::default();
			set.replace_subgrids(visible_subgrids);
			commands.entity(camera_entity).insert(set);
		}
	}
}

pub fn apply_camera_lod_policy(
	freeze: Res<FreezeCameraLods>,
	mut cameras: Query<(&CameraLodPolicy, &mut CameraLodGridControl, &mut LodRequestMap)>,
) {
	if freeze.0 { return; }
	for (policy, mut control, mut requests) in cameras.iter_mut() {
		let wanted: HashSet<_> = policy.targets().iter().map(|target| target.key).collect();
		let stale: Vec<_> = control
			.visible_lods()
			.iter()
			.chain(control.waiting_lods().iter())
			.copied()
			.filter(|key| !wanted.contains(key))
			.collect();
		for key in stale {
			control.release_lod(&mut requests, key);
		}
		for target in policy.targets() {
			control.set_lod(&mut requests, target.key, target.priority);
		}
	}
}
