//! Integration invariants for the camera loader's tile-entity handoff.
//! These tests intentionally drive the ECS systems rather than a duplicate coverage state machine.
//! They keep coverage policy independent of the concrete tile-data type.

use std::{
	collections::{HashMap, HashSet},
	path::PathBuf,
};

use bevy::{ecs::schedule::ScheduleLabel, prelude::*};
use voxel_data::{
	grid::Grid,
	voxels::{VoxelTypeId, VoxelTypeInfo},
};
use voxel_streaming::{
	ChunkAvailabilityChangeKind,
	ChunkAvailabilityChanged,
	ChunkConsumer,
	GridStreaming,
	TileClassId,
	TileLoadStatus,
	TileLoadUpdate,
	VoxelStreamingPlugin,
	chunk_of,
};

use crate::{
	CameraVoxelLoader,
	CameraVoxelLoaderConsumer,
	camera_voxel_loader::{CameraVoxelLoaderSettings, CameraVoxelTileClass},
	systems::{
		receive_camera_voxel_loader_results,
		refresh_camera_voxel_loader_visibility,
		update_camera_voxel_loader_requests,
	},
	tile_lifecycle::{ResolvedTile, TileEntry, TileResolution},
	types::TileKey,
};

#[derive(ScheduleLabel, Clone, Debug, PartialEq, Eq, Hash)]
struct RequestSchedule;

#[derive(ScheduleLabel, Clone, Debug, PartialEq, Eq, Hash)]
struct ReceiveSchedule;

#[derive(ScheduleLabel, Clone, Debug, PartialEq, Eq, Hash)]
struct RefreshSchedule;

const LOAD_LATENCY: u8 = 2;
const TEST_CLASS: TileClassId = TileClassId(0);

fn test_grid() -> Grid { Grid::new_with_type(VoxelTypeInfo { id: VoxelTypeId(1), size_bytes: 1 }) }

/// Builds an app whose three loader systems are individually runnable, so a test can interleave
/// request, receive, and availability-refresh phases exactly where an invariant needs checking.
fn test_app() -> App {
	let mut app = App::new();
	app.add_plugins(VoxelStreamingPlugin)
		.init_schedule(RequestSchedule)
		.init_schedule(ReceiveSchedule)
		.init_schedule(RefreshSchedule)
		.add_systems(RequestSchedule, update_camera_voxel_loader_requests)
		.add_systems(ReceiveSchedule, receive_camera_voxel_loader_results)
		.add_systems(RefreshSchedule, refresh_camera_voxel_loader_visibility);
	app
}

fn spawn_grid(app: &mut App, streaming: GridStreaming) -> Entity {
	app.world_mut().spawn((test_grid(), streaming, GlobalTransform::default())).id()
}

fn spawn_camera(app: &mut App, settings: CameraVoxelLoaderSettings, center: IVec3) -> Entity {
	app.world_mut()
		.spawn((
			Camera3d::default(),
			GlobalTransform::from_translation((center * voxel_streaming::CHUNK_SIZE).as_vec3()),
			CameraVoxelTileClass(TEST_CLASS),
			CameraVoxelLoader::with_settings(settings),
			CameraVoxelLoaderConsumer::default(),
		))
		.id()
}

fn move_camera(app: &mut App, camera: Entity, center: IVec3) {
	*app.world_mut().entity_mut(camera).get_mut::<GlobalTransform>().unwrap() =
		GlobalTransform::from_translation((center * voxel_streaming::CHUNK_SIZE).as_vec3());
}

fn mark_present(app: &mut App, grid: Entity, chunk: IVec3) {
	app.world_mut().entity_mut(grid).get_mut::<GridStreaming>().unwrap().mark_present(chunk);
	app.world_mut().resource_mut::<Messages<ChunkAvailabilityChanged>>().write(ChunkAvailabilityChanged {
		grid,
		min: chunk,
		size: IVec3::ONE,
		kind: ChunkAvailabilityChangeKind::BecamePresent,
	});
}

fn deliver(app: &mut App, grid: Entity, camera: Entity, key: TileKey, status: TileLoadStatus) {
	app.world_mut()
		.entity_mut(camera)
		.get_mut::<CameraVoxelLoaderConsumer>()
		.unwrap()
		.push_tile(TileLoadUpdate { grid, requester: camera, key: key.streaming_key(), status });
}

fn camera_loader(app: &App, camera: Entity) -> &CameraVoxelLoader { app.world().entity(camera).get::<CameraVoxelLoader>().unwrap() }

fn apply_delta(loader: &mut CameraVoxelLoader, added: &[TileKey], removed: &[TileKey]) -> (Vec<TileKey>, Vec<TileKey>) {
	let mut acquire = Vec::new();
	let mut release = Vec::new();
	loader.tiles.apply_delta(added, removed, &mut acquire, &mut release);
	(acquire, release)
}

/// Every tile the loader still wants but has not resolved, in request order-independent form.
fn unresolved_desired(loader: &CameraVoxelLoader) -> Vec<TileKey> {
	loader
		.tiles
		.entries()
		.filter_map(|(key, entry)| (loader.tiles.contains_desired(key) && entry.resolution == TileResolution::Requested).then_some(key))
		.collect()
}

#[test]
fn randomized_camera_coverage_transitions_do_not_leak_or_open_dependency_gaps() {
	for seed in [0x4d59_5df4_d0f3_3173, 0x94d0_49bb_1331_11eb, 0xda94_2042_e4dd_58b5] {
		run_randomized_coverage_stress(seed);
	}
}

fn run_randomized_coverage_stress(seed: u64) {
	let mut app = test_app();
	let grid = spawn_grid(&mut app, GridStreaming::default());
	let settings = CameraVoxelLoaderSettings { max_lod: 2, near_radius_chunks: 1, rings_per_lod: 1 };
	let camera = spawn_camera(&mut app, settings, IVec3::ZERO);

	let mut rng = StressRng(seed);
	let mut motion_rng = StressRng(seed ^ 0xa076_1d64_78bd_642f);
	let mut center = IVec3::ZERO;
	let mut known_chunks = HashSet::new();
	let mut pending: HashMap<TileKey, u8> = HashMap::new();

	let mut run_frame = |frame: usize, center: IVec3, add_availability: bool, rng: &mut StressRng| {
		move_camera(&mut app, camera, center);

		if add_availability {
			for _ in 0..2 {
				let chunk = IVec3::new(rng.coord(10), rng.coord(5), rng.coord(10));
				if known_chunks.insert(chunk) {
					mark_present(&mut app, grid, chunk);
				}
			}
		}

		app.world_mut().run_schedule(RequestSchedule);
		app.world_mut().run_schedule(RefreshSchedule);

		for key in unresolved_desired(camera_loader(&app, camera)) {
			pending.entry(key).or_insert_with(|| rng.delay());
		}

		for key in tick_pending(&mut pending) {
			// One in five tiles resolves as empty, exercising the non-renderable coverage path.
			let status = if rng.next() % 5 == 0 {
				TileLoadStatus::Empty
			} else {
				TileLoadStatus::Ready(app.world_mut().spawn_empty().id())
			};
			deliver(&mut app, grid, camera, key, status);
		}
		app.world_mut().run_schedule(ReceiveSchedule);

		let loader = camera_loader(&app, camera);
		assert_coverage_internal_consistency(loader, seed, frame);
		// A desired tile that is still Requested must have a result on the way. Anything else is a
		// request the loader issued and then forgot about.
		for key in unresolved_desired(loader) {
			assert!(pending.contains_key(&key), "seed={seed:#x}, frame={frame}: desired tile {key:?} is requested with no result pending");
		}
		// Conversely, nothing may stay in flight for a tile the loader has already dropped.
		pending.retain(|key, _| loader.tiles.contains_source(*key));
	};

	for frame in 0..180 {
		center = (center + IVec3::new(motion_rng.step(), motion_rng.step(), motion_rng.step())).clamp(IVec3::splat(-8), IVec3::splat(8));
		run_frame(frame, center, true, &mut rng);
	}
	for frame in 180..220 {
		run_frame(frame, center, false, &mut rng);
	}
	drop(run_frame);

	let loader = camera_loader(&app, camera);
	assert!(
		loader.tiles.entries().all(|(key, entry)| loader.tiles.contains_desired(key) && entry.resolution != TileResolution::Requested),
		"seed={seed:#x}: coverage did not settle after requests stopped changing: {:?}",
		loader.tiles,
	);
}

fn assert_coverage_internal_consistency(loader: &CameraVoxelLoader, seed: u64, frame: usize) {
	let debug = loader.tiles.coverage_debug_tiles();
	let pending: HashSet<_> = debug.iter().filter_map(|(key, pending, _)| (*pending).then_some(*key)).collect();
	let retained: HashSet<_> = debug.iter().filter_map(|(key, _, retained)| (*retained).then_some(*key)).collect();
	for (key, entry) in loader.tiles.entries() {
		if !loader.tiles.contains_desired(key) {
			assert!(retained.contains(&key), "seed={seed:#x}, frame={frame}: stale retiring source {key:?}");
		}
		if loader.tiles.contains_desired(key) && entry.resolution == TileResolution::Requested {
			assert!(pending.contains(&key), "seed={seed:#x}, frame={frame}: requested source is absent from Coverage {key:?}");
		}
	}
	for key in retained {
		assert!(
			loader.tiles.contains_source(key),
			"seed={seed:#x}, frame={frame}: Coverage retained a source missing from loader state {key:?}",
		);
	}
}

fn tick_pending(pending: &mut HashMap<TileKey, u8>) -> Vec<TileKey> {
	let mut completed = Vec::new();
	for (&key, delay) in pending.iter_mut() {
		if *delay == 0 { completed.push(key); } else { *delay -= 1; }
	}
	for key in &completed { pending.remove(key); }
	completed
}

struct StressRng(u64);

impl StressRng {
	fn next(&mut self) -> u64 {
		self.0 ^= self.0 << 13;
		self.0 ^= self.0 >> 7;
		self.0 ^= self.0 << 17;
		self.0
	}
	fn coord(&mut self, radius: i32) -> i32 { (self.next() % (radius as u64 * 2 + 1)) as i32 - radius }
	fn step(&mut self) -> i32 { (self.next() % 5) as i32 - 2 }
	fn delay(&mut self) -> u8 { (self.next() % 5) as u8 }
}

#[test]
fn newly_present_fine_tile_keeps_coarse_coverage_until_it_loads() {
	let mut app = test_app();
	let mut streaming = GridStreaming::default();
	streaming.mark_present(IVec3::ZERO);
	let grid = spawn_grid(&mut app, streaming);
	let settings = CameraVoxelLoaderSettings { max_lod: 1, near_radius_chunks: 0, rings_per_lod: 1 };
	let camera = spawn_camera(&mut app, settings, IVec3::new(2, 0, 0));
	app.world_mut().run_schedule(RequestSchedule);

	let coarse = TileKey { grid, class: TEST_CLASS, lod: 1, min: IVec3::ZERO };
	let coarse_entity = app.world_mut().spawn_empty().id();
	deliver(&mut app, grid, camera, coarse, TileLoadStatus::Ready(coarse_entity));
	app.world_mut().run_schedule(ReceiveSchedule);

	move_camera(&mut app, camera, IVec3::ZERO);
	app.world_mut().run_schedule(RequestSchedule);
	let loader = camera_loader(&app, camera);
	assert!(!loader.tiles.contains_desired(coarse));
	assert_eq!(loader.tiles.entry(coarse), Some(&TileEntry { resolution: TileResolution::Tile(coarse_entity) }));

	let newly_present = IVec3::new(1, 0, 0);
	mark_present(&mut app, grid, newly_present);
	app.world_mut().run_schedule(RefreshSchedule);
	let new_fine = TileKey { grid, class: TEST_CLASS, lod: 0, min: newly_present };
	assert_eq!(camera_loader(&app, camera).tiles.entry(new_fine), Some(&TileEntry { resolution: TileResolution::Requested }));

	let origin_fine = TileKey { grid, class: TEST_CLASS, lod: 0, min: IVec3::ZERO };
	let origin_entity = app.world_mut().spawn_empty().id();
	deliver(&mut app, grid, camera, origin_fine, TileLoadStatus::Ready(origin_entity));
	app.world_mut().run_schedule(ReceiveSchedule);
	assert_eq!(
		camera_loader(&app, camera).tiles.entry(coarse),
		Some(&TileEntry { resolution: TileResolution::Tile(coarse_entity) }),
		"coarse coverage was removed before the newly wanted fine tile loaded",
	);

	let new_entity = app.world_mut().spawn_empty().id();
	deliver(&mut app, grid, camera, new_fine, TileLoadStatus::Ready(new_entity));
	app.world_mut().run_schedule(ReceiveSchedule);
	assert!(
		!camera_loader(&app, camera).tiles.contains_source(coarse),
		"coarse coverage remained after every wanted fine tile loaded",
	);
}

#[test]
fn church_flyover_never_opens_a_hole_or_strands_a_request() {
	let church_chunks = load_church_chunks();
	assert!(!church_chunks.is_empty(), "church fixture has no chunks");
	let min = church_chunks.iter().copied().reduce(IVec3::min).unwrap() - IVec3::splat(4);
	let max = church_chunks.iter().copied().reduce(IVec3::max).unwrap() + IVec3::splat(4);

	let mut app = test_app();
	let mut streaming = GridStreaming::default();
	streaming.mark_present_area(min, max - min + IVec3::ONE);
	let grid = spawn_grid(&mut app, streaming);
	let settings = CameraVoxelLoaderSettings { max_lod: 3, near_radius_chunks: 1, rings_per_lod: 1 };
	let camera = spawn_camera(&mut app, settings, IVec3::ZERO);

	let mut loading: HashMap<TileKey, u8> = HashMap::new();
	let mut result_entities: HashMap<TileKey, Entity> = HashMap::new();
	let mut shown_chunks = HashSet::new();
	let mut covers_church_cache = HashMap::new();

	for (frame, center) in smooth_orbit_path(min, max).into_iter().enumerate() {
		move_camera(&mut app, camera, center);
		app.world_mut().run_schedule(RequestSchedule);

		for key in unresolved_desired(camera_loader(&app, camera)) {
			loading.entry(key).or_insert(LOAD_LATENCY);
		}

		for tile in tick_pending(&mut loading) {
			let status = if tile_covers_church(tile, &church_chunks, &mut covers_church_cache) {
				let entity = *result_entities.entry(tile).or_insert_with(|| app.world_mut().spawn_empty().id());
				TileLoadStatus::Ready(entity)
			} else {
				TileLoadStatus::Empty
			};
			deliver(&mut app, grid, camera, tile, status);
		}
		app.world_mut().run_schedule(ReceiveSchedule);

		let loader = camera_loader(&app, camera);
		for &chunk in &church_chunks {
			if !chunk_is_desired(loader, grid, chunk) {
				shown_chunks.remove(&chunk);
			} else if chunk_has_displayable_coverage(loader, grid, chunk) {
				shown_chunks.insert(chunk);
			}
		}
		for &chunk in &shown_chunks {
			assert!(
				chunk_has_displayable_coverage(loader, grid, chunk),
				"frame {frame}: church chunk {chunk:?} lost visible coverage during a contiguous desired period"
			);
		}
		for tile in unresolved_desired(loader) {
			assert!(loading.contains_key(&tile), "frame {frame}: desired tile {tile:?} is requested with no simulated result pending");
		}
		loading.retain(|tile, _| loader.tiles.contains_source(*tile));
	}

	assert!(!shown_chunks.is_empty(), "the flyover never displayed the church");
}

#[test]
fn availability_addition_requests_the_newly_present_tile() {
	let mut app = test_app();
	let grid = spawn_grid(&mut app, GridStreaming::default());
	let settings = CameraVoxelLoaderSettings { max_lod: 1, near_radius_chunks: 0, rings_per_lod: 1 };
	let camera = spawn_camera(&mut app, settings, IVec3::ZERO);
	app.world_mut().run_schedule(RequestSchedule);

	let key = TileKey { grid, class: TEST_CLASS, lod: 0, min: IVec3::ZERO };
	assert!(!camera_loader(&app, camera).tiles.contains_source(key), "an absent chunk must not be requested");

	mark_present(&mut app, grid, IVec3::ZERO);
	app.world_mut().run_schedule(RefreshSchedule);

	let loader = camera_loader(&app, camera);
	assert!(loader.tiles.contains_desired(key));
	assert_eq!(loader.tiles.entry(key), Some(&TileEntry { resolution: TileResolution::Requested }));
}

#[test]
fn unwanted_in_flight_tile_is_dropped_before_its_late_empty_result() {
	let mut app = test_app();
	let mut streaming = GridStreaming::default();
	streaming.mark_present(IVec3::ZERO);
	let grid = spawn_grid(&mut app, streaming);
	let settings = CameraVoxelLoaderSettings { max_lod: 1, near_radius_chunks: 0, rings_per_lod: 1 };
	let camera = spawn_camera(&mut app, settings, IVec3::ZERO);
	app.world_mut().run_schedule(RequestSchedule);

	let key = TileKey { grid, class: TEST_CLASS, lod: 0, min: IVec3::ZERO };
	assert_eq!(camera_loader(&app, camera).tiles.entry(key), Some(&TileEntry { resolution: TileResolution::Requested }));

	move_camera(&mut app, camera, IVec3::splat(100));
	app.world_mut().run_schedule(RequestSchedule);
	assert!(!camera_loader(&app, camera).tiles.contains_source(key));

	deliver(&mut app, grid, camera, key, TileLoadStatus::Empty);
	app.world_mut().run_schedule(ReceiveSchedule);

	let loader = camera_loader(&app, camera);
	assert!(!loader.tiles.contains_source(key), "a late result resurrected a tile the camera had already dropped");
	assert!(!loader.tiles.contains_desired(key));
}

#[test]
fn shared_tile_ownership_is_tracked_per_camera() {
	let mut app = test_app();
	let mut streaming = GridStreaming::default();
	streaming.mark_present(IVec3::ZERO);
	let grid = spawn_grid(&mut app, streaming);
	let settings = CameraVoxelLoaderSettings { max_lod: 1, near_radius_chunks: 0, rings_per_lod: 1 };
	let first = spawn_camera(&mut app, settings.clone(), IVec3::ZERO);
	let second = spawn_camera(&mut app, settings, IVec3::ZERO);
	app.world_mut().run_schedule(RequestSchedule);

	let key = TileKey { grid, class: TEST_CLASS, lod: 0, min: IVec3::ZERO };
	assert!(camera_loader(&app, first).tiles.contains_source(key));
	assert!(camera_loader(&app, second).tiles.contains_source(key));

	move_camera(&mut app, first, IVec3::splat(100));
	app.world_mut().run_schedule(RequestSchedule);
	assert!(!camera_loader(&app, first).tiles.contains_source(key), "the departing camera kept its tile source");
	assert!(camera_loader(&app, second).tiles.contains_source(key), "the remaining camera lost a tile it still wants");

	// A result addressed to the remaining camera must not reach the one that left.
	let entity = app.world_mut().spawn_empty().id();
	deliver(&mut app, grid, second, key, TileLoadStatus::Ready(entity));
	app.world_mut().run_schedule(ReceiveSchedule);
	assert_eq!(camera_loader(&app, second).tiles.entry(key), Some(&TileEntry { resolution: TileResolution::Tile(entity) }));
	assert!(!camera_loader(&app, first).tiles.contains_source(key));

	move_camera(&mut app, second, IVec3::splat(100));
	app.world_mut().run_schedule(RequestSchedule);
	assert!(!camera_loader(&app, second).tiles.contains_source(key));
	assert!(!camera_loader(&app, second).tiles_to_render().any(|candidate| candidate == entity));
}

#[test]
fn empty_tile_result_is_kept_while_desired_and_retired_after_departure() {
	let mut app = test_app();
	let grid = spawn_grid(&mut app, GridStreaming::default());
	let key = TileKey { grid, class: TEST_CLASS, lod: 0, min: IVec3::ZERO };
	let mut loader = CameraVoxelLoader::default();
	apply_delta(&mut loader, &[key], &[]);
	let camera = app.world_mut().spawn((loader, CameraVoxelLoaderConsumer::default())).id();

	deliver(&mut app, grid, camera, key, TileLoadStatus::Empty);
	app.world_mut().run_schedule(ReceiveSchedule);
	assert_eq!(
		app.world().entity(camera).get::<CameraVoxelLoader>().unwrap().tiles.entry(key),
		Some(&TileEntry { resolution: TileResolution::Empty }),
	);

	let release = {
		let mut camera_entity = app.world_mut().entity_mut(camera);
		let mut loader = camera_entity.get_mut::<CameraVoxelLoader>().unwrap();
		apply_delta(&mut loader, &[], &[key]).1
	};
	assert_eq!(release, vec![key]);
	assert!(!app.world().entity(camera).get::<CameraVoxelLoader>().unwrap().tiles.contains_source(key));
}

#[test]
fn availability_removal_retires_the_visible_tile_and_its_render_entity() {
	let mut app = test_app();
	let grid = spawn_grid(&mut app, GridStreaming::default());
	let key = TileKey { grid, class: TEST_CLASS, lod: 1, min: IVec3::ZERO };
	let render_entity = app.world_mut().spawn_empty().id();
	let mut loader = CameraVoxelLoader::default();
	apply_delta(&mut loader, &[key], &[]);
	let _ = loader.tiles.resolve(key, ResolvedTile::Tile(render_entity));
	let camera = app.world_mut().spawn(loader).id();

	app.world_mut().resource_mut::<Messages<ChunkAvailabilityChanged>>().write(ChunkAvailabilityChanged {
		grid,
		min: key.min,
		size: key.size(),
		kind: ChunkAvailabilityChangeKind::BecameEmpty,
	});
	app.world_mut().run_schedule(RefreshSchedule);

	let loader = app.world().entity(camera).get::<CameraVoxelLoader>().unwrap();
	assert!(!app.world().entity(grid).get::<GridStreaming>().unwrap().presence().is_present(key.min));
	assert!(!loader.tiles.contains_desired(key));
	assert!(!loader.tiles.contains_source(key));
	assert!(!loader.tiles_to_render().any(|candidate| candidate == render_entity));
}

#[test]
fn camera_policy_requests_classed_tiles_at_every_lod() {
	let mut app = test_app();
	let grid = spawn_grid(&mut app, GridStreaming::default());
	let camera = spawn_camera(&mut app, CameraVoxelLoaderSettings::default(), IVec3::ZERO);
	app.world_mut().entity_mut(grid).get_mut::<GridStreaming>().unwrap().mark_present(IVec3::ZERO);

	app.world_mut().run_schedule(RequestSchedule);

	let loader = camera_loader(&app, camera);
	assert!(loader.tiles.entries().any(|(key, _)| key.lod == 0));
	assert!(loader.tiles.entries().all(|(key, _)| key.class == TEST_CLASS));
}

#[test]
fn camera_accepts_one_tile_entity_for_every_lod() {
	let mut app = test_app();
	let grid = spawn_grid(&mut app, GridStreaming::default());
	let camera = spawn_camera(&mut app, CameraVoxelLoaderSettings::default(), IVec3::ZERO);
	let low = TileKey { grid, class: TEST_CLASS, lod: 0, min: IVec3::ZERO };
	let high = TileKey { grid, class: TEST_CLASS, lod: 3, min: IVec3::new(8, 0, 0) };
	let low_entity = app.world_mut().spawn_empty().id();
	let high_entity = app.world_mut().spawn_empty().id();

	{
		let mut camera_entity = app.world_mut().entity_mut(camera);
		let mut loader = camera_entity.get_mut::<CameraVoxelLoader>().unwrap();
		assert_eq!(apply_delta(&mut loader, &[low, high], &[]).0, vec![low, high]);
	}
	deliver(&mut app, grid, camera, low, TileLoadStatus::Ready(low_entity));
	deliver(&mut app, grid, camera, high, TileLoadStatus::Ready(high_entity));
	app.world_mut().run_schedule(ReceiveSchedule);

	let loader = camera_loader(&app, camera);
	assert_eq!(loader.tiles.entry(low), Some(&TileEntry { resolution: TileResolution::Tile(low_entity) }));
	assert_eq!(loader.tiles.entry(high), Some(&TileEntry { resolution: TileResolution::Tile(high_entity) }));
	assert!(loader.tiles_to_render().any(|entity| entity == low_entity));
	assert!(loader.tiles_to_render().any(|entity| entity == high_entity));
}

#[test]
fn finer_tile_coverage_retains_a_coarse_tile_until_every_replacement_arrives() {
	let mut app = test_app();
	let grid = spawn_grid(&mut app, GridStreaming::default());
	let camera = spawn_camera(&mut app, CameraVoxelLoaderSettings::default(), IVec3::ZERO);
	let coarse = TileKey { grid, class: TEST_CLASS, lod: 1, min: IVec3::ZERO };
	let coarse_entity = app.world_mut().spawn_empty().id();
	let fine: Vec<_> = (0..2)
		.flat_map(|x| (0..2).flat_map(move |y| (0..2).map(move |z| TileKey { grid, class: TEST_CLASS, lod: 0, min: IVec3::new(x, y, z) })))
		.collect();

	{
		let mut camera_entity = app.world_mut().entity_mut(camera);
		let mut loader = camera_entity.get_mut::<CameraVoxelLoader>().unwrap();
		apply_delta(&mut loader, &[coarse], &[]);
		assert!(loader.tiles.resolve(coarse, ResolvedTile::Tile(coarse_entity)).is_empty());
		apply_delta(&mut loader, &fine, &[]);
		assert!(apply_delta(&mut loader, &[], &[coarse]).1.is_empty());
	}

	for (index, key) in fine.iter().copied().enumerate() {
		let entity = app.world_mut().spawn_empty().id();
		let mut camera_entity = app.world_mut().entity_mut(camera);
		let mut loader = camera_entity.get_mut::<CameraVoxelLoader>().unwrap();
		let released = loader.tiles.resolve(key, ResolvedTile::Tile(entity));
		if index + 1 == fine.len() {
			assert_eq!(released, vec![coarse]);
		} else {
			assert!(released.is_empty());
		}
	}
}

#[test]
fn replacement_update_swaps_the_rendered_tile_entity() {
	let mut app = test_app();
	let grid = spawn_grid(&mut app, GridStreaming::default());
	let camera = spawn_camera(&mut app, CameraVoxelLoaderSettings::default(), IVec3::ZERO);
	let key = TileKey { grid, class: TEST_CLASS, lod: 0, min: IVec3::ZERO };
	let first = app.world_mut().spawn_empty().id();
	let second = app.world_mut().spawn_empty().id();

	let mut camera_entity = app.world_mut().entity_mut(camera);
	let mut loader = camera_entity.get_mut::<CameraVoxelLoader>().unwrap();
	apply_delta(&mut loader, &[key], &[]);
	assert!(loader.tiles.resolve(key, ResolvedTile::Tile(first)).is_empty());
	assert!(loader.tiles_to_render().any(|entity| entity == first));
	assert!(loader.tiles.resolve(key, ResolvedTile::Tile(second)).is_empty());
	assert!(!loader.tiles_to_render().any(|entity| entity == first));
	assert!(loader.tiles_to_render().any(|entity| entity == second));
}

fn region_contains(key: TileKey, chunk: IVec3) -> bool {
	let max = key.min + key.size();
	chunk.cmpge(key.min).all() && chunk.cmplt(max).all()
}

fn chunk_has_displayable_coverage(loader: &CameraVoxelLoader, grid: Entity, chunk: IVec3) -> bool {
	loader.tiles.entries().any(|(key, entry)| key.grid == grid && region_contains(key, chunk) && entry.resolution.is_visible())
}

fn chunk_is_desired(loader: &CameraVoxelLoader, grid: Entity, chunk: IVec3) -> bool {
	loader.tiles.desired().any(|key| key.grid == grid && region_contains(key, chunk))
}

fn tile_covers_church(tile: TileKey, chunks: &HashSet<IVec3>, cache: &mut HashMap<TileKey, bool>) -> bool {
	*cache.entry(tile).or_insert_with(|| chunks.iter().any(|&chunk| region_contains(tile, chunk)))
}

fn smooth_orbit_path(min: IVec3, max: IVec3) -> Vec<IVec3> {
	let center = (min + max) / 2;
	let radius = ((max.x - min.x).abs().max((max.z - min.z).abs()) / 2 + 8).max(12) as f32;
	let orbit: Vec<_> = (0..96)
		.map(|i| {
			let t = i as f32 * std::f32::consts::TAU / 96.0;
			let y = center.y + ((t * 0.7).sin() * 5.0).round() as i32;
			IVec3::new(center.x + (t.cos() * radius).round() as i32, y, center.z + (t.sin() * radius).round() as i32)
		})
		.collect();
	orbit.iter().copied().chain(orbit.iter().rev().copied()).collect()
}

fn load_church_chunks() -> HashSet<IVec3> {
	let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../res/Church_Of_St_Sophia.vox");
	let bytes = std::fs::read(&path).unwrap_or_else(|err| panic!("failed to read {path:?}: {err}"));
	let data = dot_vox::load_bytes(&bytes).expect("failed to parse church vox");
	let mut chunks = HashSet::new();

	#[derive(Clone, Copy)]
	struct Frame {
		translation: Vec3,
		rotation: Quat,
		flip: IVec3,
	}

	let mut stack = vec![(0u32, Frame { translation: Vec3::ZERO, rotation: Quat::IDENTITY, flip: IVec3::new(1, 1, -1) })];
	while let Some((scene_id, pose)) = stack.pop() {
		let Some(node) = data.scenes.get(scene_id as usize) else { continue };
		match node {
			dot_vox::SceneNode::Transform { frames, child, .. } => {
				let Some(frame) = frames.first() else { continue };
				let pos = frame.position().unwrap_or(dot_vox::Position { x: 0, y: 0, z: 0 });
				let (rot, flip_vec) = frame
					.orientation()
					.map(|q| {
						let (qarr, varr) = q.to_quat_scale();
						let q = Quat::from_array(qarr);
						(Quat::from_xyzw(q.x, q.z, -q.y, q.w), Vec3::from_array(varr).as_ivec3())
					})
					.unwrap_or((Quat::IDENTITY, IVec3::ONE));
				stack.push((
					*child,
					Frame {
						translation: pose.translation + pose.rotation * Vec3::new(pos.x as f32, pos.z as f32, -pos.y as f32),
						rotation: pose.rotation * rot,
						flip: pose.flip * IVec3::new(flip_vec.x, flip_vec.z, flip_vec.y),
					},
				));
			}
			dot_vox::SceneNode::Group { children, .. } => {
				for child in children {
					stack.push((*child, pose));
				}
			}
			dot_vox::SceneNode::Shape { models, .. } => {
				for shape_model in models {
					let Some(model) = data.models.get(shape_model.model_id as usize) else { continue };
					let size = Vec3::new(model.size.x as f32, model.size.z as f32, model.size.y as f32);
					let pose_transform = Transform { translation: pose.translation, rotation: pose.rotation, scale: Vec3::ONE };
					let half_offset = Transform::from_translation(-(size / 2.0).floor() * pose.flip.as_vec3());
					for voxel in &model.voxels {
						let local = IVec3::new(voxel.x as i32, voxel.z as i32, voxel.y as i32) * pose.flip + pose.flip.min(IVec3::ZERO);
						chunks.insert(chunk_of((pose_transform * half_offset).transform_point(local.as_vec3()).as_ivec3()));
					}
				}
			}
		}
	}
	chunks
}
