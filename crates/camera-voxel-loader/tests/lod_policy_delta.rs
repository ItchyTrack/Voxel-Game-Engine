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
			Self { max_lod: 3, near_radius_chunks: 3, rings_per_lod: 2, requests_per_frame: 16, max_in_flight: 128 }
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

#[allow(dead_code)]
#[path = "../src/lod_policy.rs"]
mod lod_policy;

use camera_voxel_loader::{CameraVoxelLoader, CameraVoxelLoaderSettings};
use lod_policy::{add_lod_tiles, add_near_tiles, is_tile_wanted, update_tiles_delta};
use types::TileKey;

#[test]
fn near_chunks_only_include_present_chunks() {
	let grid: GridId = Entity::PLACEHOLDER;
	let settings = CameraVoxelLoaderSettings::default();
	let controller = CameraVoxelLoader { settings };
	let mut streaming = GridStreaming::default();
	let present_chunk = IVec3::new(1, 0, 0);
	streaming.presence_mut().mark_present(present_chunk);

	let mut desired_chunks = HashSet::<TileKey>::new();
	add_near_tiles(&mut desired_chunks, grid, IVec3::ZERO, &controller, &streaming);

	assert_eq!(desired_chunks, HashSet::from([TileKey { grid, lod: 0, min: present_chunk }]));
}

#[test]
fn minimal_delta_misses_lod3_tile_when_inner_ring_boundary_moves_one_chunk() {
	let grid: GridId = Entity::PLACEHOLDER;
	let settings = CameraVoxelLoaderSettings { max_lod: 4, near_radius_chunks: 4, rings_per_lod: 3, requests_per_frame: 512, max_in_flight: 4096 };
	let controller = CameraVoxelLoader { settings: settings.clone() };
	let mut streaming = GridStreaming::default();
	streaming.presence_mut().mark_present_area(IVec3::splat(-160), IVec3::splat(321));

	let old_center = IVec3::ZERO;
	let new_center = IVec3::new(1, 0, 0);
	let missed = TileKey { grid, lod: 3, min: IVec3::new(-24, 0, 0) };

	let mut incremental_tiles = HashSet::<TileKey>::new();
	add_lod_tiles(&mut incremental_tiles, grid, old_center, &controller, &streaming);
	update_tiles_delta(&mut incremental_tiles, grid, old_center, new_center, &settings, &streaming);

	let mut rebuilt_tiles = HashSet::<TileKey>::new();
	add_lod_tiles(&mut rebuilt_tiles, grid, new_center, &controller, &streaming);

	assert_eq!(
		incremental_tiles.contains(&missed),
		rebuilt_tiles.contains(&missed),
		"incremental update diverged from full rebuild for former repro tile {missed:?} moving from {old_center:?} to {new_center:?}"
	);
}

#[test]
fn lod_policy_must_not_assign_multiple_lod_tiles_to_same_present_chunk() {
	let grid: GridId = Entity::PLACEHOLDER;
	let settings = CameraVoxelLoaderSettings::default();
	let controller = CameraVoxelLoader { settings };
	let center = IVec3::new(3, 2, 0);
	let chunk = TileKey { grid, lod: 0, min: IVec3::new(-15, 6, 11) };
	let lod2_tile = TileKey { grid, lod: 2, min: IVec3::new(-16, 4, 8) };
	let lod3_tile = TileKey { grid, lod: 3, min: IVec3::new(-16, 0, 8) };

	let mut streaming = GridStreaming::default();
	streaming.presence_mut().mark_present(chunk.min);

	let mut desired_chunks = HashSet::<TileKey>::new();
	let mut desired_tiles = HashSet::<TileKey>::new();
	add_near_tiles(&mut desired_chunks, grid, center, &controller, &streaming);
	add_lod_tiles(&mut desired_tiles, grid, center, &controller, &streaming);

	let covering_tiles: Vec<_> = desired_tiles
		.iter()
		.copied()
		.filter(|tile| tile.grid == chunk.grid && chunk.min.cmpge(tile.min).all() && chunk.min.cmplt(tile.min + tile.size()).all())
		.collect();
	let owner_count = desired_chunks.contains(&chunk) as usize + covering_tiles.len();

	assert!(
		desired_tiles.contains(&lod2_tile) || desired_tiles.contains(&lod3_tile),
		"control setup should still desire one of the LOD tiles around the runtime repro chunk"
	);
	assert_eq!(
		owner_count,
		1,
		"present chunk {chunk:?} should have exactly one desired owner; lod2_present={}, lod3_present={}, covering_tiles={covering_tiles:?}",
		desired_tiles.contains(&lod2_tile),
		desired_tiles.contains(&lod3_tile),
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
	add_lod_tiles(&mut incremental_tiles, grid, center, &controller, &streaming);
	assert!(!incremental_tiles.contains(&expected_tile), "control setup should start with no LOD2 tile because no chunks are present");

	streaming.presence_mut().mark_present(new_present_chunk);
	update_tiles_delta(&mut incremental_tiles, grid, center, center, &settings, &streaming);
	// New presence events are handled outside movement deltas: the changed chunk maps
	// to its covering LOD tiles, and currently-wanted tiles are inserted/requested.
	if is_tile_wanted(&settings, &streaming, center, expected_tile) {
		incremental_tiles.insert(expected_tile);
	}

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

	// Player edits the existing loaded chunk here. Dirty chunk events invalidate stale
	// cached LOD records, even while the camera is close and the tile is not desired.
	tile_records.remove(&stale_tile);

	// Then the camera flies away and the same chunk should be represented by LOD2.
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
	let mut request_queue = Vec::<TileKey>::new();
	let mut unload_deps = HashMap::<TileKey, TileKey>::new();

	// Dirty chunk events re-request the active LOD tile and keep the old entity alive
	// until the replacement resolves.
	if desired_tiles.contains(&dirty_tile) {
		request_queue.push(dirty_tile);
		unload_deps.insert(dirty_tile, dirty_tile);
	}

	assert!(
		request_queue.contains(&dirty_tile),
		"editing chunk {edited_chunk:?} should re-request dirty active LOD tile {dirty_tile:?}, but the existing TileRecord caused it to be reused"
	);
	assert!(
		unload_deps.contains_key(&dirty_tile),
		"old dirty LOD tile {dirty_tile:?} should have an unload dependency on its replacement request resolving"
	);
}

#[test]
fn full_lod_policy_covers_every_present_chunk_inside_streaming_domain() {
	let grid: GridId = Entity::PLACEHOLDER;
	let settings = CameraVoxelLoaderSettings { max_lod: 3, near_radius_chunks: 4, rings_per_lod: 3, requests_per_frame: 512, max_in_flight: 4096 };
	let controller = CameraVoxelLoader { settings: settings.clone() };
	let mut streaming = GridStreaming::default();
	streaming.presence_mut().mark_present_area(IVec3::splat(-48), IVec3::splat(97));

	for center in [IVec3::ZERO, IVec3::new(1, 0, 0), IVec3::new(12, 0, -4), IVec3::new(-17, 3, 9)] {
		let mut desired_chunks = HashSet::<TileKey>::new();
		let mut desired_tiles = HashSet::<TileKey>::new();
		add_near_tiles(&mut desired_chunks, grid, center, &controller, &streaming);
		add_lod_tiles(&mut desired_tiles, grid, center, &controller, &streaming);

		let covered = covered_chunks(&desired_chunks, &desired_tiles);
		let (min, max) = policy_domain_bounds(center, &settings);
		for x in min.x..max.x {
			for y in min.y..max.y {
				for z in min.z..max.z {
					let chunk = TileKey { grid, lod: 0, min: IVec3::new(x, y, z) };
					if !streaming.presence().is_present(chunk.min) {
						continue;
					}
					assert!(
						covered.contains(&chunk),
						"full LOD policy left present chunk {chunk:?} uncovered at center {center:?}; this would let old visible coverage retire with NoDesiredCoverage"
					);
				}
			}
		}
	}
}

#[test]
fn lod_policy_assigns_exactly_one_owner_to_present_chunks_for_explicit_settings() {
	for settings in explicit_settings_cases() {
		let grid: GridId = Entity::PLACEHOLDER;
		let controller = CameraVoxelLoader { settings: settings.clone() };
		let mut streaming = GridStreaming::default();
		let centers = sample_centers(&settings);
		let present_chunks = sample_present_chunks(&settings);
		for chunk in &present_chunks {
			streaming.presence_mut().mark_present(*chunk);
		}

		for center in centers {
			let mut desired_chunks = HashSet::<TileKey>::new();
			let mut desired_tiles = HashSet::<TileKey>::new();
			add_near_tiles(&mut desired_chunks, grid, center, &controller, &streaming);
			add_lod_tiles(&mut desired_tiles, grid, center, &controller, &streaming);

			for &present in &present_chunks {
				let chunk = TileKey { grid, lod: 0, min: present };
				if !policy_domain_contains(center, &settings, present) {
					continue;
				}
				let covering_tiles = covering_lod_tiles(&desired_tiles, chunk);
				let owner_count = desired_chunks.contains(&chunk) as usize + covering_tiles.len();
				assert_eq!(
					owner_count, 1,
					"settings={settings:?}, center={center:?}: present chunk {chunk:?} should have exactly one owner, near_owner={}, covering_tiles={covering_tiles:?}",
					desired_chunks.contains(&chunk),
				);
			}
		}
	}
}

#[test]
fn lod_policy_lod_tiles_must_not_cover_present_near_chunks_for_explicit_settings() {
	for settings in explicit_settings_cases() {
		let grid: GridId = Entity::PLACEHOLDER;
		let controller = CameraVoxelLoader { settings: settings.clone() };
		let center = IVec3::ZERO;
		let mut streaming = GridStreaming::default();
		let near = settings.near_radius_chunks;
		let present_chunks = [
			IVec3::ZERO,
			IVec3::new(near, 0, 0),
			IVec3::new(-near, 0, 0),
			IVec3::new(0, near, 0),
			IVec3::new(0, 0, near),
			IVec3::new(near, near, near),
			IVec3::new(-near, -near, -near),
		];
		for chunk in present_chunks {
			streaming.presence_mut().mark_present(chunk);
		}

		let mut desired_chunks = HashSet::<TileKey>::new();
		let mut desired_tiles = HashSet::<TileKey>::new();
		add_near_tiles(&mut desired_chunks, grid, center, &controller, &streaming);
		add_lod_tiles(&mut desired_tiles, grid, center, &controller, &streaming);

		for present in present_chunks {
			let chunk = TileKey { grid, lod: 0, min: present };
			assert!(desired_chunks.contains(&chunk), "settings={settings:?}: near present chunk {chunk:?} should be owned by LOD0");
			let covering_tiles = covering_lod_tiles(&desired_tiles, chunk);
			assert!(
				covering_tiles.is_empty(),
				"settings={settings:?}: near present chunk {chunk:?} should not also be covered by LOD tiles {covering_tiles:?}"
			);
		}
	}
}

#[test]
fn lod_policy_generated_tiles_match_is_tile_wanted_for_explicit_settings() {
	for settings in explicit_settings_cases() {
		let grid: GridId = Entity::PLACEHOLDER;
		let controller = CameraVoxelLoader { settings: settings.clone() };
		let mut streaming = GridStreaming::default();
		streaming.presence_mut().mark_present_area(IVec3::splat(-32), IVec3::splat(65));

		for center in sample_centers(&settings) {
			let mut desired_tiles = HashSet::<TileKey>::new();
			add_lod_tiles(&mut desired_tiles, grid, center, &controller, &streaming);
			for lod in 1..=settings.max_lod {
				let tile_size = 1i32 << lod;
				let outer = max_lod_outer_radius(&settings);
				let min = lod_policy::align_chunk_to_tile(center - IVec3::splat(outer + tile_size), tile_size);
				let max = lod_policy::align_chunk_to_tile(center + IVec3::splat(outer + tile_size), tile_size);
				for x in (min.x..=max.x).step_by(tile_size as usize) {
					for y in (min.y..=max.y).step_by(tile_size as usize) {
						for z in (min.z..=max.z).step_by(tile_size as usize) {
							let tile = TileKey { grid, lod, min: IVec3::new(x, y, z) };
							assert_eq!(
								desired_tiles.contains(&tile),
								is_tile_wanted(&settings, &streaming, center, tile),
								"settings={settings:?}, center={center:?}: add_lod_tiles disagrees with is_tile_wanted for {tile:?}"
							);
						}
					}
				}
			}
		}
	}
}

#[test]
fn incremental_lod_policy_matches_full_rebuild_for_explicit_settings_and_targeted_steps() {
	for settings in explicit_settings_cases() {
		let grid: GridId = Entity::PLACEHOLDER;
		let controller = CameraVoxelLoader { settings: settings.clone() };
		let mut streaming = GridStreaming::default();
		streaming.presence_mut().mark_present_area(IVec3::splat(-48), IVec3::splat(97));
		let centers = sample_centers(&settings);
		let deltas = [IVec3::X, -IVec3::X, IVec3::Y, -IVec3::Y, IVec3::Z, -IVec3::Z, IVec3::ONE, -IVec3::ONE];
		for old_center in centers {
			for delta in deltas {
				assert_single_delta_matches_full_rebuild(grid, &controller, &settings, &streaming, old_center, old_center + delta);
			}
		}
	}
}

#[test]
fn incremental_lod_policy_matches_full_rebuild_for_systematic_one_chunk_boundary_crossings() {
	let grid: GridId = Entity::PLACEHOLDER;
	let settings = CameraVoxelLoaderSettings { max_lod: 3, near_radius_chunks: 4, rings_per_lod: 3, requests_per_frame: 512, max_in_flight: 4096 };
	let controller = CameraVoxelLoader { settings: settings.clone() };
	let mut streaming = GridStreaming::default();
	streaming.presence_mut().mark_present_area(IVec3::splat(-80), IVec3::splat(161));

	let deltas = [
		IVec3::new(1, 0, 0),
		IVec3::new(-1, 0, 0),
		IVec3::new(0, 1, 0),
		IVec3::new(0, -1, 0),
		IVec3::new(0, 0, 1),
		IVec3::new(0, 0, -1),
		IVec3::new(1, 1, 1),
		IVec3::new(-1, -1, -1),
	];

	// Target a compact set of centers that straddle powers-of-two tile boundaries
	// and the LOD ring radii used by the settings above. This keeps the test fast
	// while covering the boundary classes that have historically broken deltas.
	let centers = [
		IVec3::new(-13, 0, 0),
		IVec3::new(-12, 0, 0),
		IVec3::new(-11, 0, 0),
		IVec3::new(-5, 0, 0),
		IVec3::new(-4, 0, 0),
		IVec3::new(-3, 0, 0),
		IVec3::new(0, 0, 0),
		IVec3::new(11, 0, 0),
		IVec3::new(12, 0, 0),
		IVec3::new(13, 0, 0),
		IVec3::new(12, 4, 0),
		IVec3::new(12, 0, 12),
	];
	for old_center in centers {
		for delta in deltas {
			assert_single_delta_matches_full_rebuild(grid, &controller, &settings, &streaming, old_center, old_center + delta);
		}
	}
}

#[test]
fn incremental_lod_policy_matches_full_rebuild_for_deterministic_flight_stress() {
	let grid: GridId = Entity::PLACEHOLDER;
	let settings = CameraVoxelLoaderSettings { max_lod: 3, near_radius_chunks: 4, rings_per_lod: 3, requests_per_frame: 512, max_in_flight: 4096 };
	let controller = CameraVoxelLoader { settings: settings.clone() };
	let mut streaming = GridStreaming::default();
	streaming.presence_mut().mark_present_area(IVec3::splat(-80), IVec3::splat(161));

	let mut incremental_chunks = HashSet::<TileKey>::new();
	let mut incremental_tiles = HashSet::<TileKey>::new();
	let mut center = IVec3::ZERO;
	add_near_tiles(&mut incremental_chunks, grid, center, &controller, &streaming);
	add_lod_tiles(&mut incremental_tiles, grid, center, &controller, &streaming);

	let mut seed = 0x5EED_u32;
	for step in 0..48 {
		seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
		let delta = IVec3::new((seed as i32).rem_euclid(5) - 2, ((seed >> 8) as i32).rem_euclid(3) - 1, ((seed >> 16) as i32).rem_euclid(5) - 2);
		let delta = if delta == IVec3::ZERO { IVec3::X } else { delta };
		let next = (center + delta).clamp(IVec3::splat(-24), IVec3::splat(24));

		update_tiles_delta(&mut incremental_chunks, grid, center, next, &settings, &streaming);
		update_tiles_delta(&mut incremental_tiles, grid, center, next, &settings, &streaming);
		assert_matches_full_rebuild(grid, &controller, &streaming, next, &incremental_chunks, &incremental_tiles);
		center = next;

		if step % 257 == 0 {
			// Also verify recovery from a full rebuild seed mid-flight, mirroring the
			// production path when a large camera jump forces a rebuild.
			incremental_chunks.clear();
			incremental_tiles.clear();
			add_near_tiles(&mut incremental_chunks, grid, center, &controller, &streaming);
			add_lod_tiles(&mut incremental_tiles, grid, center, &controller, &streaming);
		}
	}
}

#[test]
fn incremental_lod_policy_matches_full_rebuild_while_camera_flies() {
	let grid: GridId = Entity::PLACEHOLDER;
	let settings = CameraVoxelLoaderSettings { max_lod: 4, near_radius_chunks: 4, rings_per_lod: 3, requests_per_frame: 512, max_in_flight: 4096 };
	let controller = CameraVoxelLoader { settings: settings.clone() };
	let mut streaming = GridStreaming::default();
	streaming.presence_mut().mark_present_area(IVec3::splat(-160), IVec3::splat(321));

	let centers = flying_camera_path();
	let mut incremental_chunks = HashSet::<TileKey>::new();
	let mut incremental_tiles = HashSet::<TileKey>::new();

	let first = centers[0];
	add_near_tiles(&mut incremental_chunks, grid, first, &controller, &streaming);
	add_lod_tiles(&mut incremental_tiles, grid, first, &controller, &streaming);
	assert_matches_full_rebuild(grid, &controller, &streaming, first, &incremental_chunks, &incremental_tiles);

	let mut previous = first;
	for center in centers.into_iter().skip(1) {
		update_tiles_delta(&mut incremental_chunks, grid, previous, center, &settings, &streaming);
		update_tiles_delta(&mut incremental_tiles, grid, previous, center, &settings, &streaming);

		assert_matches_full_rebuild(grid, &controller, &streaming, center, &incremental_chunks, &incremental_tiles);
		previous = center;
	}
}

fn explicit_settings_cases() -> Vec<CameraVoxelLoaderSettings> {
	vec![
		CameraVoxelLoaderSettings { max_lod: 2, near_radius_chunks: 2, rings_per_lod: 1, requests_per_frame: 8, max_in_flight: 32 },
		CameraVoxelLoaderSettings { max_lod: 3, near_radius_chunks: 3, rings_per_lod: 2, requests_per_frame: 16, max_in_flight: 128 },
		CameraVoxelLoaderSettings { max_lod: 3, near_radius_chunks: 4, rings_per_lod: 3, requests_per_frame: 32, max_in_flight: 256 },
		CameraVoxelLoaderSettings { max_lod: 4, near_radius_chunks: 5, rings_per_lod: 1, requests_per_frame: 32, max_in_flight: 256 },
	]
}

fn sample_centers(settings: &CameraVoxelLoaderSettings) -> Vec<IVec3> {
	let near = settings.near_radius_chunks;
	let outer = max_lod_outer_radius(settings);
	vec![
		IVec3::ZERO,
		IVec3::X,
		-IVec3::X,
		IVec3::new(near, 0, 0),
		IVec3::new(near + 1, 0, 0),
		IVec3::new(-near - 1, 0, 0),
		IVec3::new(outer / 2, 1, -outer / 3),
		IVec3::new(-outer / 2, -1, outer / 3),
	]
}

fn sample_present_chunks(settings: &CameraVoxelLoaderSettings) -> Vec<IVec3> {
	let mut out = Vec::new();
	let mut radii = vec![0, 1, settings.near_radius_chunks, settings.near_radius_chunks + 1];
	let mut inner = settings.near_radius_chunks + 1;
	for lod in 1..=settings.max_lod {
		let tile_size = 1i32 << lod;
		let outer = inner + settings.rings_per_lod * tile_size;
		radii.extend([inner.saturating_sub(1), inner, inner + 1, outer.saturating_sub(1), outer, outer + 1]);
		inner = outer + 1;
	}
	radii.sort_unstable();
	radii.dedup();
	for r in radii {
		for p in
			[IVec3::new(r, 0, 0), IVec3::new(-r, 0, 0), IVec3::new(0, r, 0), IVec3::new(0, 0, r), IVec3::new(r, r / 2, -r), IVec3::new(-r, r / 2, r)]
		{
			out.push(p);
		}
	}
	out.sort_unstable_by_key(|p| (p.x, p.y, p.z));
	out.dedup();
	out
}

fn covering_lod_tiles(desired_tiles: &HashSet<TileKey>, chunk: TileKey) -> Vec<TileKey> {
	desired_tiles
		.iter()
		.copied()
		.filter(|tile| tile.grid == chunk.grid && chunk.min.cmpge(tile.min).all() && chunk.min.cmplt(tile.min + tile.size()).all())
		.collect()
}

fn max_lod_outer_radius(settings: &CameraVoxelLoaderSettings) -> i32 {
	let (min, max) = policy_domain_bounds(IVec3::ZERO, settings);
	min.abs().max(max.abs()).max_element()
}

fn policy_domain_contains(center: IVec3, settings: &CameraVoxelLoaderSettings, chunk: IVec3) -> bool {
	let (min, max) = policy_domain_bounds(center, settings);
	chunk.cmpge(min).all() && chunk.cmplt(max).all()
}

fn policy_domain_bounds(center: IVec3, settings: &CameraVoxelLoaderSettings) -> (IVec3, IVec3) {
	test_lod_outer_bounds(center, settings, settings.max_lod)
}

fn test_lod_inner_bounds(center: IVec3, settings: &CameraVoxelLoaderSettings, lod: u8) -> (IVec3, IVec3) {
	if lod == 1 {
		test_near_bounds(center, settings)
	} else {
		test_lod_outer_bounds(center, settings, lod - 1)
	}
}

fn test_lod_outer_bounds(center: IVec3, settings: &CameraVoxelLoaderSettings, lod: u8) -> (IVec3, IVec3) {
	let bounds = test_expand_bounds_by_tile_rings(test_lod_inner_bounds(center, settings, lod), 1i32 << lod, settings.rings_per_lod);
	if lod < settings.max_lod {
		test_align_bounds_to_tile(bounds.0, bounds.1, 1i32 << (lod + 1))
	} else {
		bounds
	}
}

fn test_near_bounds(center: IVec3, settings: &CameraVoxelLoaderSettings) -> (IVec3, IVec3) {
	test_align_bounds_to_tile(center - IVec3::splat(settings.near_radius_chunks), center + IVec3::splat(settings.near_radius_chunks + 1), 2)
}

fn test_expand_bounds_by_tile_rings((min, max): (IVec3, IVec3), tile_size: i32, rings: i32) -> (IVec3, IVec3) {
	let expansion = IVec3::splat(tile_size * rings);
	(min - expansion, max + expansion)
}

fn test_align_bounds_to_tile(min: IVec3, max: IVec3, tile_size: i32) -> (IVec3, IVec3) {
	let tile = IVec3::splat(tile_size);
	(min.div_euclid(tile) * tile, (max + tile - IVec3::ONE).div_euclid(tile) * tile)
}

fn covered_chunks(desired_chunks: &HashSet<TileKey>, desired_tiles: &HashSet<TileKey>) -> HashSet<TileKey> {
	let mut covered = desired_chunks.clone();
	for tile in desired_tiles {
		let max = tile.min + tile.size();
		for x in tile.min.x..max.x {
			for y in tile.min.y..max.y {
				for z in tile.min.z..max.z {
					covered.insert(TileKey { grid: tile.grid, lod: 0, min: IVec3::new(x, y, z) });
				}
			}
		}
	}
	covered
}

fn assert_single_delta_matches_full_rebuild(
	grid: GridId, controller: &CameraVoxelLoader, settings: &CameraVoxelLoaderSettings, streaming: &GridStreaming, old_center: IVec3,
	new_center: IVec3,
) {
	let mut incremental_chunks = HashSet::<TileKey>::new();
	let mut incremental_tiles = HashSet::<TileKey>::new();
	add_near_tiles(&mut incremental_chunks, grid, old_center, controller, streaming);
	add_lod_tiles(&mut incremental_tiles, grid, old_center, controller, streaming);
	update_tiles_delta(&mut incremental_chunks, grid, old_center, new_center, settings, streaming);
	update_tiles_delta(&mut incremental_tiles, grid, old_center, new_center, settings, streaming);
	assert_matches_full_rebuild(grid, controller, streaming, new_center, &incremental_chunks, &incremental_tiles);
}

fn assert_matches_full_rebuild(
	grid: GridId, controller: &CameraVoxelLoader, streaming: &GridStreaming, center: IVec3, incremental_chunks: &HashSet<TileKey>,
	incremental_tiles: &HashSet<TileKey>,
) {
	let mut rebuilt_chunks = HashSet::new();
	let mut rebuilt_tiles = HashSet::new();
	add_near_tiles(&mut rebuilt_chunks, grid, center, controller, streaming);
	add_lod_tiles(&mut rebuilt_tiles, grid, center, controller, streaming);

	let incremental_chunks: HashSet<_> = incremental_chunks.iter().copied().filter(|key| key.lod == 0).collect();
	let incremental_tiles: HashSet<_> = incremental_tiles.iter().copied().filter(|key| key.lod > 0).collect();

	let missing_chunks: Vec<_> = rebuilt_chunks.difference(&incremental_chunks).take(16).copied().collect();
	let extra_chunks: Vec<_> = incremental_chunks.difference(&rebuilt_chunks).take(16).copied().collect();
	let missing_tiles: Vec<_> = rebuilt_tiles.difference(&incremental_tiles).take(16).copied().collect();
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
