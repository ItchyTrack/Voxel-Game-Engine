use std::collections::{HashMap, HashSet};

use bevy::prelude::*;
use voxel_data::grid::GridId;
use voxel_streaming::GridStreaming;

mod camera_voxel_loader {
	#[derive(Debug, Clone, PartialEq, Eq)]
	pub struct CameraVoxelLoaderSettings {
		pub max_lod: u8,
		pub near_radius_chunks: i32,
		pub rings_per_lod: i32,
		pub requests_per_frame: usize,
		pub max_in_flight: usize,
	}

	impl Default for CameraVoxelLoaderSettings {
		fn default() -> Self {
			Self {
				max_lod: 3,
				near_radius_chunks: 3,
				rings_per_lod: 2,
				requests_per_frame: 16,
				max_in_flight: 128,
			}
		}
	}

	#[derive(Debug, Clone)]
	pub struct CameraVoxelLoader {
		pub settings: CameraVoxelLoaderSettings,
	}
}

mod types {
	use bevy::prelude::*;
	use voxel_data::grid::GridId;

	#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
	pub(crate) struct ChunkKey {
		pub(crate) grid: GridId,
		pub(crate) chunk: IVec3,
	}

	#[derive(Clone, Copy, Debug, PartialEq, Eq)]
	pub(crate) enum PolicyDebugBoxKind {
		NearChunks,
		LodOuter(u8),
		LodInner(u8),
		LodNearExclusion(u8),
	}

	#[derive(Clone, Copy, Debug, PartialEq, Eq)]
	pub(crate) struct PolicyDebugBox {
		pub(crate) grid: GridId,
		pub(crate) min: IVec3,
		pub(crate) max: IVec3,
		pub(crate) entering: bool,
		pub(crate) kind: PolicyDebugBoxKind,
	}

	#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
	pub(crate) struct TileKey {
		pub(crate) grid: GridId,
		pub(crate) lod: u8,
		pub(crate) min: IVec3,
	}

	impl TileKey {
		pub(crate) fn size(self) -> IVec3 {
			IVec3::splat(1i32 << self.lod)
		}
	}
}

#[path = "../src/lod_policy.rs"]
mod lod_policy;

use camera_voxel_loader::{CameraVoxelLoader, CameraVoxelLoaderSettings};
use lod_policy::{add_lod_tiles, add_near_chunks, update_lod_tiles_delta, update_near_chunks_delta};
use types::{ChunkKey, PolicyDebugBox, TileKey};

#[test]
fn minimal_delta_misses_lod3_tile_when_inner_ring_boundary_moves_one_chunk() {
	let grid: GridId = Entity::PLACEHOLDER;
	let settings = CameraVoxelLoaderSettings {
		max_lod: 4,
		near_radius_chunks: 4,
		rings_per_lod: 3,
		requests_per_frame: 512,
		max_in_flight: 4096,
	};
	let controller = CameraVoxelLoader { settings: settings.clone() };
	let mut streaming = GridStreaming::default();
	streaming.presence_mut().mark_present_area(IVec3::splat(-160), IVec3::splat(321));

	let old_center = IVec3::ZERO;
	let new_center = IVec3::new(1, 0, 0);
	let missed = TileKey { grid, lod: 3, min: IVec3::new(-24, 0, 0) };

	let mut incremental_tiles = HashSet::<TileKey>::new();
	let mut debug_boxes = Vec::<PolicyDebugBox>::new();
	add_lod_tiles(&mut incremental_tiles, grid, old_center, &controller, &streaming);
	update_lod_tiles_delta(&mut incremental_tiles, &mut debug_boxes, grid, old_center, new_center, &settings, &streaming);

	let mut rebuilt_tiles = HashSet::<TileKey>::new();
	add_lod_tiles(&mut rebuilt_tiles, grid, new_center, &controller, &streaming);

	assert!(rebuilt_tiles.contains(&missed), "control full rebuild should request the repro tile");
	assert!(
		incremental_tiles.contains(&missed),
		"incremental update missed {missed:?}: moving the center from {old_center:?} to {new_center:?} shifts the LOD 3 inner ring threshold. \
		The tile's max distance is 24 at the old center, so it is inside the LOD 2 area; at the new center its max distance is 25, so it enters LOD 3. \
		The current slab candidate generation only considers changed inner-box coordinate slabs and never tests this still-inside-both-boxes tile."
	);
}

#[test]
fn lod2_tile_is_missing_when_presence_appears_without_camera_movement() {
	let grid: GridId = Entity::PLACEHOLDER;
	let settings = CameraVoxelLoaderSettings::default();
	let controller = CameraVoxelLoader { settings: settings.clone() };
	let center = IVec3::ZERO;
	let new_present_chunk = IVec3::new(12, 0, 0);
	let expected_tile = TileKey { grid, lod: 2, min: IVec3::new(12, 0, 0) };

	let mut streaming = GridStreaming::default();
	let mut incremental_tiles = HashSet::<TileKey>::new();
	let mut debug_boxes = Vec::<PolicyDebugBox>::new();
	add_lod_tiles(&mut incremental_tiles, grid, center, &controller, &streaming);
	assert!(!incremental_tiles.contains(&expected_tile), "control setup should start with no LOD2 tile because no chunks are present");

	streaming.presence_mut().mark_present(new_present_chunk);
	update_lod_tiles_delta(&mut incremental_tiles, &mut debug_boxes, grid, center, center, &settings, &streaming);

	let mut rebuilt_tiles = HashSet::<TileKey>::new();
	add_lod_tiles(&mut rebuilt_tiles, grid, center, &controller, &streaming);
	assert!(rebuilt_tiles.contains(&expected_tile), "control full rebuild should request the newly-present LOD2 tile");
	assert!(
		incremental_tiles.contains(&expected_tile),
		"incremental policy missed {expected_tile:?}: presence changed inside an already-stationary LOD2 tile, but no camera-center delta produced tile candidates"
	);
}

#[test]
fn lod2_tile_is_not_requeued_when_existing_lod0_chunk_is_edited_then_camera_flies_away() {
	let grid: GridId = Entity::PLACEHOLDER;
	let controller = CameraVoxelLoader { settings: CameraVoxelLoaderSettings::default() };
	let mut streaming = GridStreaming::default();
	let edited_chunk = IVec3::new(12, 0, 0);
	let stale_tile = TileKey { grid, lod: 2, min: IVec3::new(12, 0, 0) };

	// The chunk already exists. Editing it does not change presence, so movement-only
	// policy deltas will not know that any old LOD result covering it is stale.
	streaming.presence_mut().mark_present(edited_chunk);

	// Simulate a past visit where this LOD2 tile resolved empty/stale before the
	// player edited the now-loaded LOD0 chunk.
	let mut tile_records = HashMap::from([(stale_tile, "Empty")]);
	let mut request_queue = Vec::<TileKey>::new();

	// Camera starts close enough that the edited chunk is represented by LOD0.
	let near_center = IVec3::new(12, 0, 0);
	let mut desired_tiles_near = HashSet::<TileKey>::new();
	add_lod_tiles(&mut desired_tiles_near, grid, near_center, &controller, &streaming);
	assert!(!desired_tiles_near.contains(&stale_tile), "control setup should render the edited chunk as LOD0 while nearby");

	// Player edits the existing loaded chunk here. Then the camera flies away and the
	// same chunk should be represented by LOD2.
	let far_center = IVec3::ZERO;
	let mut desired_tiles_far = HashSet::<TileKey>::new();
	add_lod_tiles(&mut desired_tiles_far, grid, far_center, &controller, &streaming);
	assert!(desired_tiles_far.contains(&stale_tile), "control full rebuild should want the edited chunk's LOD2 tile after flying away");

	// Mirrors loading.rs: desired tiles are only queued when there is no existing
	// TileRecord. A stale Empty/Ready tile survives the edit, so the replacement
	// LOD2 request is skipped when the camera later needs it.
	for key in desired_tiles_far {
		if tile_records.contains_key(&key) {
			continue;
		}
		tile_records.insert(key, "Queued");
		request_queue.push(key);
	}

	assert!(
		request_queue.contains(&stale_tile),
		"edited existing LOD0 chunk did not requeue stale {stale_tile:?} when it became LOD2 after flying away"
	);
}

#[test]
fn editing_chunk_inside_active_lod2_tile_should_dirty_tile_and_wait_to_unload_old_lod() {
	let grid: GridId = Entity::PLACEHOLDER;
	let controller = CameraVoxelLoader { settings: CameraVoxelLoaderSettings::default() };
	let center = IVec3::ZERO;
	let edited_chunk = IVec3::new(12, 0, 0);
	let dirty_tile = TileKey { grid, lod: 2, min: IVec3::new(12, 0, 0) };

	let mut streaming = GridStreaming::default();
	streaming.presence_mut().mark_present(edited_chunk);

	let mut desired_tiles = HashSet::<TileKey>::new();
	add_lod_tiles(&mut desired_tiles, grid, center, &controller, &streaming);
	assert!(desired_tiles.contains(&dirty_tile), "control setup should actively want the LOD2 tile that covers the edited chunk");

	// Simulate current loader state: the LOD2 tile is already resolved and would be
	// rendered/reused. Editing an underlying chunk should make this record dirty,
	// request a replacement LOD, and keep the old LOD alive until the replacement
	// resolves.
	let tile_records = HashMap::from([(dirty_tile, "Ready")]);
	let mut request_queue = Vec::<TileKey>::new();
	let mut unload_deps = HashMap::<TileKey, TileKey>::new();

	// Current behavior mirrored from loading.rs: desired tiles with any existing
	// TileRecord are skipped, so no replacement request/dependency is created.
	for key in desired_tiles {
		if tile_records.contains_key(&key) {
			continue;
		}
		request_queue.push(key);
	}

	assert!(
		request_queue.contains(&dirty_tile),
		"editing chunk {edited_chunk:?} should re-request dirty active LOD tile {dirty_tile:?}, but the existing TileRecord caused it to be reused"
	);
	unload_deps.insert(dirty_tile, dirty_tile);
	assert!(
		unload_deps.contains_key(&dirty_tile),
		"old dirty LOD tile {dirty_tile:?} should have an unload dependency on its replacement request resolving"
	);
}

#[test]
fn incremental_lod_policy_matches_full_rebuild_while_camera_flies() {
	let grid: GridId = Entity::PLACEHOLDER;
	let settings = CameraVoxelLoaderSettings {
		max_lod: 4,
		near_radius_chunks: 4,
		rings_per_lod: 3,
		requests_per_frame: 512,
		max_in_flight: 4096,
	};
	let controller = CameraVoxelLoader { settings: settings.clone() };
	let mut streaming = GridStreaming::default();
	streaming.presence_mut().mark_present_area(IVec3::splat(-160), IVec3::splat(321));

	let centers = flying_camera_path();
	let mut incremental_chunks = HashSet::<ChunkKey>::new();
	let mut incremental_tiles = HashSet::<TileKey>::new();
	let mut debug_boxes = Vec::<PolicyDebugBox>::new();

	let first = centers[0];
	add_near_chunks(&mut incremental_chunks, grid, first, &controller);
	add_lod_tiles(&mut incremental_tiles, grid, first, &controller, &streaming);
	assert_matches_full_rebuild(grid, &controller, &streaming, first, &incremental_chunks, &incremental_tiles);

	let mut previous = first;
	for center in centers.into_iter().skip(1) {
		debug_boxes.clear();
		update_near_chunks_delta(&mut incremental_chunks, &mut debug_boxes, grid, previous, center, &settings);
		update_lod_tiles_delta(&mut incremental_tiles, &mut debug_boxes, grid, previous, center, &settings, &streaming);

		assert_matches_full_rebuild(grid, &controller, &streaming, center, &incremental_chunks, &incremental_tiles);
		previous = center;
	}
}

fn assert_matches_full_rebuild(
	grid: GridId,
	controller: &CameraVoxelLoader,
	streaming: &GridStreaming,
	center: IVec3,
	incremental_chunks: &HashSet<ChunkKey>,
	incremental_tiles: &HashSet<TileKey>,
) {
	let mut rebuilt_chunks = HashSet::new();
	let mut rebuilt_tiles = HashSet::new();
	add_near_chunks(&mut rebuilt_chunks, grid, center, controller);
	add_lod_tiles(&mut rebuilt_tiles, grid, center, controller, streaming);

	let missing_chunks: Vec<_> = rebuilt_chunks.difference(incremental_chunks).take(16).copied().collect();
	let extra_chunks: Vec<_> = incremental_chunks.difference(&rebuilt_chunks).take(16).copied().collect();
	let missing_tiles: Vec<_> = rebuilt_tiles.difference(incremental_tiles).take(16).copied().collect();
	let extra_tiles: Vec<_> = incremental_tiles.difference(&rebuilt_tiles).take(16).copied().collect();

	assert!(
		missing_chunks.is_empty() && extra_chunks.is_empty() && missing_tiles.is_empty() && extra_tiles.is_empty(),
		"incremental LOD policy diverged from full rebuild at center {center:?}\n\
		 chunks: expected {}, got {}, missing sample {missing_chunks:?}, extra sample {extra_chunks:?}\n\
		 tiles: expected {}, got {}, missing sample {missing_tiles:?}, extra sample {extra_tiles:?}",
		rebuilt_chunks.len(),
		incremental_chunks.len(),
		rebuilt_tiles.len(),
		incremental_tiles.len(),
	);
}

fn flying_camera_path() -> Vec<IVec3> {
	let mut centers = Vec::new();
	let mut p = IVec3::ZERO;
	centers.push(p);

	let steps = [
		IVec3::new(1, 0, 0),
		IVec3::new(1, 0, 1),
		IVec3::new(0, 1, 1),
		IVec3::new(-1, 1, 0),
		IVec3::new(-1, 0, -1),
		IVec3::new(0, -1, -1),
		IVec3::new(1, -1, 0),
		IVec3::new(2, 0, 1),
		IVec3::new(-2, 1, 0),
		IVec3::new(0, -1, 2),
		IVec3::new(1, 1, -2),
	];

	for i in 0..96 {
		p += steps[i % steps.len()];
		centers.push(p);
	}
	centers
}
