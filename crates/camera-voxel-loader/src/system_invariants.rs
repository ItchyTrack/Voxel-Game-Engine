//! Integration invariants for the flattened camera-loader systems.
//! These tests intentionally drive the ECS systems rather than a duplicate coverage state machine.

use std::{
	collections::{HashMap, HashSet},
	path::PathBuf,
};

use bevy::{ecs::schedule::ScheduleLabel, prelude::*};
use voxel_data::{grid::Grid, voxels::Voxels};
use voxel_gpu::VoxelGpuUploadFinished;
use voxel_streaming::{
	ChunkAvailabilityChangeKind,
	ChunkAvailabilityChanged,
	ChunkConsumer,
	ChunkLoadResolved,
	ChunkLoadResult,
	ChunkState,
	GridStreaming,
	LodKey,
	LodLoadResult,
	StreamingSchedule,
	VoxelStreamingPlugin,
	chunk_of,
};

use crate::{
	CameraVoxelLoaderConsumer,
	camera_voxel_loader::{CameraVoxelLoader, CameraVoxelLoaderSettings},
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

fn apply_tile_delta(loader: &mut CameraVoxelLoader, added: &[TileKey], removed: &[TileKey]) -> (Vec<TileKey>, Vec<TileKey>) {
	let mut acquire = Vec::new();
	let mut release = Vec::new();
	loader.tiles.apply_delta(added, removed, &mut acquire, &mut release);
	(acquire, release)
}

#[test]
fn randomized_camera_coverage_transitions_do_not_leak_or_open_dependency_gaps() {
	for seed in [0x4d59_5df4_d0f3_3173, 0x94d0_49bb_1331_11eb, 0xda94_2042_e4dd_58b5] {
		run_randomized_coverage_stress(seed);
	}
}

fn run_randomized_coverage_stress(seed: u64) {
	let mut app = App::new();
	app.add_plugins(VoxelStreamingPlugin)
		.add_message::<VoxelGpuUploadFinished>()
		.init_schedule(RequestSchedule)
		.init_schedule(ReceiveSchedule)
		.init_schedule(RefreshSchedule)
		.add_systems(RequestSchedule, update_camera_voxel_loader_requests)
		.add_systems(ReceiveSchedule, receive_camera_voxel_loader_results)
		.add_systems(RefreshSchedule, refresh_camera_voxel_loader_visibility);

	let grid = app.world_mut().spawn((Grid::default(), GridStreaming::default(), GlobalTransform::default())).id();
	let settings = CameraVoxelLoaderSettings { max_lod: 2, near_radius_chunks: 1, rings_per_lod: 1 };
	let camera = app.world_mut().spawn((Camera3d::default(), CameraVoxelLoader::with_settings(settings), CameraVoxelLoaderConsumer::default())).id();
	let mut rng = StressRng(seed);
	let mut motion_rng = StressRng(seed ^ 0xa076_1d64_78bd_642f);
	let mut center = IVec3::ZERO;
	let mut known_chunks = HashSet::new();
	let mut pending_chunks: HashMap<TileKey, u8> = HashMap::new();
	let mut pending_lods: HashMap<TileKey, u8> = HashMap::new();

	let mut run_frame = |frame: usize, center: IVec3, add_availability: bool| {
		*app.world_mut().entity_mut(camera).get_mut::<GlobalTransform>().unwrap() =
			GlobalTransform::from_translation((center * voxel_streaming::CHUNK_SIZE).as_vec3());

		if add_availability {
			for _ in 0..2 {
				let chunk = IVec3::new(rng.coord(10), rng.coord(5), rng.coord(10));
				if known_chunks.insert(chunk) {
					app.world_mut().entity_mut(grid).get_mut::<GridStreaming>().unwrap().mark_present(chunk);
					app.world_mut().resource_mut::<Messages<ChunkAvailabilityChanged>>().write(ChunkAvailabilityChanged {
						grid,
						min: chunk,
						size: IVec3::ONE,
						kind: ChunkAvailabilityChangeKind::BecamePresent,
					});
				}
			}
		}

		app.world_mut().run_schedule(RequestSchedule);
		let loader = app.world().entity(camera).get::<CameraVoxelLoader>().unwrap();
		let requested: Vec<_> = loader
			.tiles
			.entries()
			.filter_map(|(key, entry)| (loader.tiles.contains_desired(key) && entry.resolution == TileResolution::Requested).then_some(key))
			.collect();
		for key in requested {
			let pending = if key.is_chunk() { &mut pending_chunks } else { &mut pending_lods };
			pending.entry(key).or_insert_with(|| rng.delay());
		}

		let completed_chunks = tick_pending(&mut pending_chunks);
		for key in completed_chunks {
			app.world_mut().resource_mut::<Messages<ChunkLoadResult>>().write(ChunkLoadResult {
				grid: key.grid,
				chunk: key.min,
				generation: 0,
				voxels: Some(Voxels::new()),
			});
		}
		let completed_lods = tick_pending(&mut pending_lods);
		for key in completed_lods {
			let entity = (rng.next() % 5 != 0).then(|| app.world_mut().spawn_empty().id());
			app.world_mut().entity_mut(camera).get_mut::<CameraVoxelLoaderConsumer>().unwrap().push_lod(LodLoadResult {
				grid: key.grid,
				requester: camera,
				key: LodKey { min: key.min, size: key.size(), lod: key.lod },
				priority: 0.0,
				generation: 0,
				voxels: None,
				entity,
			});
		}

		app.world_mut().run_schedule(StreamingSchedule);
		app.world_mut().run_schedule(ReceiveSchedule);
		app.world_mut().run_schedule(RefreshSchedule);

		let loader = app.world().entity(camera).get::<CameraVoxelLoader>().unwrap();
		assert_coverage_internal_consistency(loader, seed, frame);
		let owned_chunks: HashSet<_> = loader.tiles.entries().filter_map(|(key, _)| key.is_chunk().then_some(key.min)).collect();
		let streaming = app.world().entity(grid).get::<GridStreaming>().unwrap();
		for &chunk in &known_chunks {
			let expected = u16::from(owned_chunks.contains(&chunk));
			assert_eq!(
				streaming.presence().request_count(chunk), expected,
				"seed={seed:#x}, frame={frame}: chunk request ownership diverged at {chunk:?}",
			);
		}
	};

	for frame in 0..180 {
		center = (center + IVec3::new(motion_rng.step(), motion_rng.step(), motion_rng.step())).clamp(IVec3::splat(-8), IVec3::splat(8));
		run_frame(frame, center, true);
	}
	for frame in 180..220 {
		run_frame(frame, center, false);
	}
	drop(run_frame);

	let loader = app.world().entity(camera).get::<CameraVoxelLoader>().unwrap();
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
fn newly_present_lod0_chunk_keeps_lod1_coverage_until_it_loads() {
	let mut app = App::new();
	app.add_plugins(VoxelStreamingPlugin)
		.add_message::<VoxelGpuUploadFinished>()
		.init_schedule(RequestSchedule)
		.init_schedule(ReceiveSchedule)
		.init_schedule(RefreshSchedule)
		.add_systems(RequestSchedule, update_camera_voxel_loader_requests)
		.add_systems(ReceiveSchedule, receive_camera_voxel_loader_results)
		.add_systems(RefreshSchedule, refresh_camera_voxel_loader_visibility);

	let mut streaming = GridStreaming::default();
	streaming.mark_present(IVec3::ZERO);
	let grid = app.world_mut().spawn((Grid::default(), streaming, GlobalTransform::default())).id();
	let settings = CameraVoxelLoaderSettings { max_lod: 1, near_radius_chunks: 0, rings_per_lod: 1 };
	let camera = app
		.world_mut()
		.spawn((
			Camera3d::default(),
			GlobalTransform::from_translation((IVec3::new(2, 0, 0) * voxel_streaming::CHUNK_SIZE).as_vec3()),
			CameraVoxelLoader::with_settings(settings),
			CameraVoxelLoaderConsumer::default(),
		))
		.id();
	app.world_mut().run_schedule(RequestSchedule);

	let coarse = TileKey { grid, lod: 1, min: IVec3::ZERO };
	let coarse_entity = app.world_mut().spawn_empty().id();
	app.world_mut().entity_mut(camera).get_mut::<CameraVoxelLoaderConsumer>().unwrap().push_lod(LodLoadResult {
		grid,
		requester: camera,
		key: LodKey { min: coarse.min, size: coarse.size(), lod: coarse.lod },
		priority: 0.0,
		generation: 0,
		voxels: None,
		entity: Some(coarse_entity),
	});
	app.world_mut().run_schedule(ReceiveSchedule);

	*app.world_mut().entity_mut(camera).get_mut::<GlobalTransform>().unwrap() = GlobalTransform::IDENTITY;
	app.world_mut().run_schedule(RequestSchedule);
	let loader = app.world().entity(camera).get::<CameraVoxelLoader>().unwrap();
	assert!(!loader.tiles.contains_desired(coarse));
	assert_eq!(loader.tiles.entry(coarse), Some(&TileEntry { resolution: TileResolution::Lod(coarse_entity) }));

	let newly_present = IVec3::new(1, 0, 0);
	app.world_mut().entity_mut(grid).get_mut::<GridStreaming>().unwrap().mark_present(newly_present);
	app.world_mut().resource_mut::<Messages<ChunkAvailabilityChanged>>().write(ChunkAvailabilityChanged {
		grid,
		min: newly_present,
		size: IVec3::ONE,
		kind: ChunkAvailabilityChangeKind::BecamePresent,
	});
	app.world_mut().run_schedule(RefreshSchedule);
	let new_fine = TileKey::chunk(grid, newly_present);
	assert_eq!(
		app.world().entity(camera).get::<CameraVoxelLoader>().unwrap().tiles.entry(new_fine),
		Some(&TileEntry { resolution: TileResolution::Requested }),
	);

	app.world_mut().resource_mut::<Messages<ChunkLoadResult>>().write(ChunkLoadResult {
		grid,
		chunk: IVec3::ZERO,
		generation: 0,
		voxels: Some(Voxels::new()),
	});
	app.world_mut().run_schedule(StreamingSchedule);
	app.world_mut().run_schedule(RefreshSchedule);

	assert_eq!(
		app.world().entity(camera).get::<CameraVoxelLoader>().unwrap().tiles.entry(coarse),
		Some(&TileEntry { resolution: TileResolution::Lod(coarse_entity) }),
		"LOD1 coverage was removed before the newly wanted LOD0 chunk loaded",
	);

	app.world_mut().resource_mut::<Messages<ChunkLoadResult>>().write(ChunkLoadResult {
		grid,
		chunk: newly_present,
		generation: 0,
		voxels: Some(Voxels::new()),
	});
	app.world_mut().run_schedule(StreamingSchedule);
	app.world_mut().run_schedule(RefreshSchedule);
	assert!(
		!app.world().entity(camera).get::<CameraVoxelLoader>().unwrap().tiles.contains_source(coarse),
		"LOD1 coverage remained after every wanted LOD0 chunk loaded",
	);
}

#[test]
fn church_flyover_never_opens_a_hole_or_strands_a_request() {
	let church_chunks = load_church_chunks();
	assert!(!church_chunks.is_empty(), "church fixture has no chunks");
	let min = church_chunks.iter().copied().reduce(IVec3::min).unwrap() - IVec3::splat(4);
	let max = church_chunks.iter().copied().reduce(IVec3::max).unwrap() + IVec3::splat(4);

	let mut app = App::new();
	app.add_plugins(VoxelStreamingPlugin)
		.init_schedule(RequestSchedule)
		.init_schedule(ReceiveSchedule)
		.add_systems(RequestSchedule, update_camera_voxel_loader_requests)
		.add_systems(ReceiveSchedule, receive_camera_voxel_loader_results);

	let mut streaming = GridStreaming::default();
	streaming.mark_present_area(min, max - min + IVec3::ONE);
	let grid = app.world_mut().spawn((Grid::default(), streaming, GlobalTransform::default())).id();
	let settings = CameraVoxelLoaderSettings { max_lod: 3, near_radius_chunks: 1, rings_per_lod: 1 };
	let camera = app.world_mut().spawn((Camera3d::default(), CameraVoxelLoader::with_settings(settings), CameraVoxelLoaderConsumer::default())).id();

	let mut loading: HashMap<TileKey, u8> = HashMap::new();
	let mut result_entities: HashMap<TileKey, Entity> = HashMap::new();
	let mut shown_chunks = HashSet::new();
	let mut covers_church_cache = HashMap::new();

	for (frame, center) in smooth_orbit_path(min, max).into_iter().enumerate() {
		*app.world_mut().entity_mut(camera).get_mut::<GlobalTransform>().unwrap() =
			GlobalTransform::from_translation((center * voxel_streaming::CHUNK_SIZE).as_vec3());
		app.world_mut().run_schedule(RequestSchedule);

		{
			let loader = app.world().entity(camera).get::<CameraVoxelLoader>().unwrap();
			for (tile, entry) in loader.tiles.entries() {
				if !tile.is_chunk() && loader.tiles.contains_desired(tile) && entry.resolution == TileResolution::Requested {
					loading.entry(tile).or_insert(LOAD_LATENCY);
				}
			}
		}

		let mut completed = Vec::new();
		for (&tile, remaining) in &mut loading {
			if *remaining == 0 {
				completed.push(tile);
			} else {
				*remaining -= 1;
			}
		}
		for tile in completed {
			loading.remove(&tile);
			let has_data = tile_covers_church(tile, &church_chunks, &mut covers_church_cache);
			let entity = has_data.then(|| *result_entities.entry(tile).or_insert_with(|| app.world_mut().spawn_empty().id()));
			app.world_mut().entity_mut(camera).get_mut::<CameraVoxelLoaderConsumer>().unwrap().push_lod(LodLoadResult {
				grid,
				requester: camera,
				key: LodKey { min: tile.min, size: tile.size(), lod: tile.lod },
				priority: 0.0,
				generation: 0,
				voxels: None,
				entity,
			});
		}
		app.world_mut().run_schedule(ReceiveSchedule);

		let loader = app.world().entity(camera).get::<CameraVoxelLoader>().unwrap();
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
		for tile in loader.tiles.desired().filter(|tile| !tile.is_chunk()) {
			if loader.tiles.entry(tile).is_some_and(|entry| entry.resolution == TileResolution::Requested) {
				assert!(loading.contains_key(&tile), "frame {frame}: desired tile {tile:?} is requested with no simulated result pending");
			}
		}
	}

	assert!(!shown_chunks.is_empty(), "the flyover never displayed the church");
}

#[test]
fn availability_addition_requests_the_newly_present_tile() {
	let mut app = App::new();
	app.add_plugins(VoxelStreamingPlugin)
		.add_message::<VoxelGpuUploadFinished>()
		.add_message::<ChunkLoadResolved>()
		.add_message::<ChunkAvailabilityChanged>()
		.init_schedule(RequestSchedule)
		.init_schedule(RefreshSchedule)
		.add_systems(RequestSchedule, update_camera_voxel_loader_requests)
		.add_systems(RefreshSchedule, refresh_camera_voxel_loader_visibility);

	let grid = app.world_mut().spawn((Grid::default(), GridStreaming::default(), GlobalTransform::default())).id();
	let settings = CameraVoxelLoaderSettings { max_lod: 1, near_radius_chunks: 0, rings_per_lod: 1 };
	let camera = app.world_mut().spawn((Camera3d::default(), CameraVoxelLoader::with_settings(settings))).id();
	app.world_mut().run_schedule(RequestSchedule);

	app.world_mut().entity_mut(grid).get_mut::<GridStreaming>().unwrap().mark_present(IVec3::ZERO);
	assert_eq!(app.world().entity(grid).get::<GridStreaming>().unwrap().state(IVec3::ZERO), Some(ChunkState::Available));
	assert_eq!(app.world().entity(grid).get::<GridStreaming>().unwrap().presence().request_count(IVec3::ZERO), 0);
	app.world_mut().resource_mut::<Messages<ChunkAvailabilityChanged>>().write(ChunkAvailabilityChanged {
		grid,
		min: IVec3::ZERO,
		size: IVec3::ONE,
		kind: ChunkAvailabilityChangeKind::BecamePresent,
	});
	app.world_mut().run_schedule(RefreshSchedule);

	let key = TileKey::chunk(grid, IVec3::ZERO);
	let loader = app.world().entity(camera).get::<CameraVoxelLoader>().unwrap();
	assert!(loader.tiles.contains_desired(key));
	assert_eq!(loader.tiles.entry(key), Some(&TileEntry { resolution: TileResolution::Requested }));
	let streaming = app.world().entity(grid).get::<GridStreaming>().unwrap();
	assert!(streaming.presence().is_present(key.min));
	assert_eq!(streaming.state(key.min), Some(ChunkState::InFlight));
	assert_eq!(streaming.presence().request_count(key.min), 1);
}

#[test]
fn unwanted_in_flight_chunk_releases_before_its_late_empty_result() {
	let mut app = App::new();
	app.add_plugins(VoxelStreamingPlugin)
		.add_message::<VoxelGpuUploadFinished>()
		.init_schedule(RequestSchedule)
		.init_schedule(RefreshSchedule)
		.add_systems(RequestSchedule, update_camera_voxel_loader_requests)
		.add_systems(RefreshSchedule, refresh_camera_voxel_loader_visibility);

	let mut streaming = GridStreaming::default();
	streaming.mark_present(IVec3::ZERO);
	let grid = app.world_mut().spawn((Grid::default(), streaming, GlobalTransform::default())).id();
	let settings = CameraVoxelLoaderSettings { max_lod: 1, near_radius_chunks: 0, rings_per_lod: 1 };
	let camera = app.world_mut().spawn((Camera3d::default(), CameraVoxelLoader::with_settings(settings))).id();
	app.world_mut().run_schedule(RequestSchedule);

	let key = TileKey::chunk(grid, IVec3::ZERO);
	{
		let streaming = app.world().entity(grid).get::<GridStreaming>().unwrap();
		assert_eq!(streaming.state(key.min), Some(ChunkState::InFlight));
		assert_eq!(streaming.presence().request_count(key.min), 1);
	}

	*app.world_mut().entity_mut(camera).get_mut::<GlobalTransform>().unwrap() =
		GlobalTransform::from_translation((IVec3::splat(100) * voxel_streaming::CHUNK_SIZE).as_vec3());
	app.world_mut().run_schedule(RequestSchedule);
	assert_eq!(app.world().entity(grid).get::<GridStreaming>().unwrap().presence().request_count(key.min), 0);
	assert!(!app.world().entity(camera).get::<CameraVoxelLoader>().unwrap().tiles.contains_source(key));

	app.world_mut().resource_mut::<Messages<ChunkLoadResult>>().write(ChunkLoadResult {
		grid,
		chunk: key.min,
		generation: 0,
		voxels: None,
	});
	app.world_mut().run_schedule(StreamingSchedule);
	app.world_mut().run_schedule(RefreshSchedule);

	let streaming = app.world().entity(grid).get::<GridStreaming>().unwrap();
	assert!(!streaming.presence().is_present(key.min));
	assert_eq!(streaming.state(key.min), None);
	assert_eq!(streaming.presence().request_count(key.min), 0);
	assert!(!app.world().entity(camera).get::<CameraVoxelLoader>().unwrap().tiles.contains_source(key));
}

#[test]
fn shared_chunk_presence_balances_each_camera_request() {
	let mut app = App::new();
	app.add_plugins(VoxelStreamingPlugin)
		.add_message::<VoxelGpuUploadFinished>()
		.init_schedule(RequestSchedule)
		.init_schedule(RefreshSchedule)
		.add_systems(RequestSchedule, update_camera_voxel_loader_requests)
		.add_systems(RefreshSchedule, refresh_camera_voxel_loader_visibility);

	let mut streaming = GridStreaming::default();
	streaming.mark_present(IVec3::ZERO);
	let grid = app.world_mut().spawn((Grid::default(), streaming, GlobalTransform::default())).id();
	let settings = CameraVoxelLoaderSettings { max_lod: 1, near_radius_chunks: 0, rings_per_lod: 1 };
	let first = app.world_mut().spawn((Camera3d::default(), CameraVoxelLoader::with_settings(settings.clone()))).id();
	let second = app.world_mut().spawn((Camera3d::default(), CameraVoxelLoader::with_settings(settings))).id();
	app.world_mut().run_schedule(RequestSchedule);

	let key = TileKey::chunk(grid, IVec3::ZERO);
	let streaming = app.world().entity(grid).get::<GridStreaming>().unwrap();
	assert_eq!(streaming.state(key.min), Some(ChunkState::InFlight));
	assert_eq!(streaming.presence().request_count(key.min), 2);

	*app.world_mut().entity_mut(first).get_mut::<GlobalTransform>().unwrap() =
		GlobalTransform::from_translation((IVec3::splat(100) * voxel_streaming::CHUNK_SIZE).as_vec3());
	app.world_mut().run_schedule(RequestSchedule);
	assert_eq!(app.world().entity(grid).get::<GridStreaming>().unwrap().presence().request_count(key.min), 1);

	app.world_mut().resource_mut::<Messages<ChunkLoadResult>>().write(ChunkLoadResult {
		grid,
		chunk: key.min,
		generation: 0,
		voxels: Some(Voxels::new()),
	});
	app.world_mut().run_schedule(StreamingSchedule);
	assert_eq!(app.world().entity(grid).get::<GridStreaming>().unwrap().state(key.min), Some(ChunkState::Loaded));
	assert_eq!(app.world().entity(grid).get::<GridStreaming>().unwrap().presence().request_count(key.min), 1);
	app.world_mut().run_schedule(RefreshSchedule);
	assert_eq!(app.world().entity(grid).get::<GridStreaming>().unwrap().presence().request_count(key.min), 1);
	assert!(!app.world().entity(first).get::<CameraVoxelLoader>().unwrap().tiles.contains_source(key));
	assert!(app.world().entity(second).get::<CameraVoxelLoader>().unwrap().tiles.contains_source(key));

	*app.world_mut().entity_mut(second).get_mut::<GlobalTransform>().unwrap() =
		GlobalTransform::from_translation((IVec3::splat(100) * voxel_streaming::CHUNK_SIZE).as_vec3());
	app.world_mut().run_schedule(RequestSchedule);
	let streaming = app.world().entity(grid).get::<GridStreaming>().unwrap();
	assert_eq!(streaming.state(key.min), Some(ChunkState::Loaded));
	assert_eq!(streaming.presence().request_count(key.min), 0);
	assert!(!app.world().entity(second).get::<CameraVoxelLoader>().unwrap().tiles.contains_source(key));
}

#[test]
fn invisible_chunk_result_is_kept_while_desired_and_retired_after_departure() {
	let mut app = App::new();
	app.add_plugins(VoxelStreamingPlugin)
		.add_message::<VoxelGpuUploadFinished>()
		.add_message::<ChunkLoadResolved>()
		.add_message::<ChunkAvailabilityChanged>()
		.init_schedule(RefreshSchedule)
		.add_systems(RefreshSchedule, refresh_camera_voxel_loader_visibility);

	let grid = app.world_mut().spawn((Grid::default(), GridStreaming::default())).id();
	let key = TileKey::chunk(grid, IVec3::ZERO);
	let mut loader = CameraVoxelLoader::default();
	apply_tile_delta(&mut loader, &[key], &[]);
	let camera = app.world_mut().spawn(loader).id();

	app.world_mut().resource_mut::<Messages<ChunkLoadResolved>>().write(ChunkLoadResolved { grid, chunk: key.min, visible: false });
	app.world_mut().run_schedule(RefreshSchedule);
	assert_eq!(
		app.world().entity(camera).get::<CameraVoxelLoader>().unwrap().tiles.entry(key),
		Some(&TileEntry { resolution: TileResolution::Empty }),
	);

	let release = {
		let mut entity = app.world_mut().entity_mut(camera);
		let mut loader = entity.get_mut::<CameraVoxelLoader>().unwrap();
		apply_tile_delta(&mut loader, &[], &[key]).1
	};
	assert_eq!(release, vec![key]);
	assert!(!app.world().entity(camera).get::<CameraVoxelLoader>().unwrap().tiles.contains_source(key));
}

#[test]
fn availability_removal_retires_visible_lod_and_render_entity() {
	let mut app = App::new();
	app.add_plugins(VoxelStreamingPlugin)
		.add_message::<VoxelGpuUploadFinished>()
		.add_message::<ChunkLoadResolved>()
		.add_message::<ChunkAvailabilityChanged>()
		.init_schedule(RefreshSchedule)
		.add_systems(RefreshSchedule, refresh_camera_voxel_loader_visibility);

	let grid = app.world_mut().spawn((Grid::default(), GridStreaming::default())).id();
	let key = TileKey { grid, lod: 1, min: IVec3::ZERO };
	let render_entity = app.world_mut().spawn_empty().id();
	let mut loader = CameraVoxelLoader::default();
	apply_tile_delta(&mut loader, &[key], &[]);
	let _ = loader.tiles.resolve(key, ResolvedTile::Lod(render_entity));
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
	assert!(!loader.lods_to_render().contains(&render_entity));
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
