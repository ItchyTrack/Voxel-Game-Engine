use std::{collections::{HashMap, HashSet}, thread, time::Duration};

use bevy::{prelude::*, window::WindowResolution};
use camera_voxel_loader::{
	CameraVoxelLoader, CameraVoxelLoaderDefaultSettings, CameraVoxelLoaderSettings, CoverageDebugState, CoverageDebugTile,
};
use voxel_app::build_app_with_mode;
use voxel_data::subgrid::SubGrid;
use voxel_engine::VoxelEngineMode;
use voxel_gpu::{LodVoxels, VoxelGpuFormat, VoxelGpuState};
use voxel_ray_renderer::voxel_camera::VoxelCamera;
use voxel_streaming::CHUNK_SIZE;

/// GPU-backed soak test for the scene that voxel-app currently loads.
///
/// This is a harness-free test binary because macOS requires winit's event loop to be created on
/// the process main thread. Run it manually with:
/// `VOXEL_COVERAGE_SCENE_FUZZ=1 cargo test -p voxel-app --test coverage_scene_fuzz`
fn main() {
	if std::env::var_os("VOXEL_COVERAGE_SCENE_FUZZ").is_none() {
		println!("coverage scene fuzz skipped; set VOXEL_COVERAGE_SCENE_FUZZ=1 to run it");
		return;
	}
	current_scene_keeps_rendered_coverage_during_randomized_camera_flight();
}

fn current_scene_keeps_rendered_coverage_during_randomized_camera_flight() {
	let window = Window {
		title: "coverage scene fuzz".into(),
		visible: false,
		resolution: WindowResolution::new(640, 360),
		..default()
	};
	let mut app = build_app_with_mode(window, VoxelEngineMode::Host);
	app.world_mut().insert_resource(CameraVoxelLoaderDefaultSettings(CameraVoxelLoaderSettings {
		max_lod: 3,
		near_radius_chunks: 2,
		rings_per_lod: 1,
	}));
	app.finish();
	app.cleanup();

	// Keep the default run short enough for local iteration. Set VOXEL_COVERAGE_SCENE_FUZZ_FRAMES
	// to run a longer soak without changing the deterministic path.
	let frames = std::env::var("VOXEL_COVERAGE_SCENE_FUZZ_FRAMES").ok().and_then(|value| value.parse().ok()).unwrap_or(72usize);
	for _ in 0..12 {
		app.update();
		thread::sleep(Duration::from_millis(1));
	}

	let camera = app
		.world_mut()
		.query_filtered::<Entity, (With<Camera3d>, With<CameraVoxelLoader>, With<VoxelCamera>)>()
		.single(app.world())
		.expect("voxel-app did not spawn one ray-rendering camera");
	let mut rng = FuzzRng(0x8f3f_73b5_cf1c_9ade);
	let mut previously_rendered = rendered_chunks(app.world(), camera);
	let mut waiting_age: HashMap<(Entity, u8, IVec3), usize> = HashMap::new();
	let mut max_rendered_chunks = previously_rendered.len();

	// These points exercise the church, both BB8s, the initial camera area, and several
	// approaches to the procedural planet centered around z=-2000.
	let waypoints = [
		Vec3::new(0.0, 0.0, 200.0),
		Vec3::new(0.0, -250.0, 180.0),
		Vec3::new(0.0, -350.0, 40.0),
		Vec3::new(90.0, -330.0, -30.0),
		Vec3::new(30.0, 120.0, 90.0),
		Vec3::new(-30.0, 120.0, 50.0),
		Vec3::new(0.0, 0.0, -1250.0),
		Vec3::new(420.0, 120.0, -1550.0),
		Vec3::new(-380.0, -160.0, -1750.0),
		Vec3::new(0.0, 0.0, 200.0),
	];

	for frame in 0..frames {
		let path_position = frame as f32 * (waypoints.len() - 1) as f32 / frames.saturating_sub(1).max(1) as f32;
		let segment = (path_position.floor() as usize).min(waypoints.len() - 2);
		let t = path_position.fract();
		let jitter = Vec3::new(rng.jitter(10.0), rng.jitter(5.0), rng.jitter(10.0));
		let position = waypoints[segment].lerp(waypoints[segment + 1], t) + jitter;
		app.world_mut().entity_mut(camera).get_mut::<Transform>().unwrap().translation = position;

		app.update();
		thread::sleep(Duration::from_millis(1));

		let world = app.world();
		let loader = world.entity(camera).get::<CameraVoxelLoader>().unwrap();
		let debug_tiles = loader.coverage_debug_tiles();
		let currently_rendered = rendered_chunks(world, camera);
		if (63..=65).contains(&frame) {
			let target = IVec3::new(-1, 0, 1);
			let covering: Vec<_> = debug_tiles.iter().filter(|tile| tile_contains_chunk(tile, target)).collect();
			eprintln!("frame {frame} target coverage: {covering:#?}");
			eprintln!("frame {frame} target rendered grids: {:?}", currently_rendered.iter().filter(|(_, chunk)| *chunk == target).collect::<Vec<_>>());
		}

		assert_pending_regions_keep_prior_rendered_coverage(
			frame,
			&debug_tiles,
			&previously_rendered,
			&currently_rendered,
		);
		assert_render_lists_are_gpu_ready(world, camera, frame);
		update_waiting_ages(frame, frames.max(30), &debug_tiles, &mut waiting_age);
		max_rendered_chunks = max_rendered_chunks.max(currently_rendered.len());
		previously_rendered = currently_rendered;
	}
	assert!(max_rendered_chunks > 0, "scene fuzz observed no GPU-ready rendered voxel chunks");
}

fn assert_pending_regions_keep_prior_rendered_coverage(
	frame: usize,
	debug_tiles: &[CoverageDebugTile],
	previously_rendered: &HashSet<(Entity, IVec3)>,
	currently_rendered: &HashSet<(Entity, IVec3)>,
) {
	let pending: Vec<_> = debug_tiles.iter().filter(|tile| tile.state == CoverageDebugState::Pending).collect();
	for &(grid, chunk) in previously_rendered {
		let still_transitioning = pending.iter().any(|tile| tile.grid == grid && tile_contains_chunk(tile, chunk));
		if still_transitioning {
			if !currently_rendered.contains(&(grid, chunk)) {
				eprintln!("frame {frame} coverage debug: {debug_tiles:#?}");
				eprintln!("previously rendered: {previously_rendered:#?}");
				eprintln!("currently rendered: {currently_rendered:#?}");
			}
			assert!(
				currently_rendered.contains(&(grid, chunk)),
				"frame {frame}: chunk {chunk:?} on grid {grid:?} lost rendered coverage while replacement coverage was pending",
			);
		}
	}
}

fn assert_render_lists_are_gpu_ready(world: &World, camera: Entity, frame: usize) {
	let voxel_camera = world.entity(camera).get::<VoxelCamera>().unwrap();
	for &entity in voxel_camera.subgrids_to_render.iter().chain(&voxel_camera.lods_to_render) {
		let gpu = world.entity(entity).get::<VoxelGpuState>();
		assert!(
			gpu.is_some_and(|state| state.matches(VoxelGpuFormat::Volume)),
			"frame {frame}: camera render list contains non-GPU-ready entity {entity:?}",
		);
	}
}

fn update_waiting_ages(
	frame: usize,
	max_wait_frames: usize,
	debug_tiles: &[CoverageDebugTile],
	waiting_age: &mut HashMap<(Entity, u8, IVec3), usize>,
) {
	let waiting: HashSet<_> = debug_tiles
		.iter()
		.filter(|tile| tile.state == CoverageDebugState::Waiting)
		.map(|tile| (tile.grid, tile.lod, tile.min))
		.collect();
	waiting_age.retain(|key, _| waiting.contains(key));
	for key in waiting {
		let age = waiting_age.entry(key).or_insert(frame);
		assert!(
			frame - *age < max_wait_frames,
			"frame {frame}: coverage tile {key:?} remained waiting for {max_wait_frames} frames",
		);
	}
}

fn rendered_chunks(world: &World, camera: Entity) -> HashSet<(Entity, IVec3)> {
	let voxel_camera = world.entity(camera).get::<VoxelCamera>().unwrap();
	let mut chunks = HashSet::new();

	for &entity in &voxel_camera.subgrids_to_render {
		let entity_ref = world.entity(entity);
		let Some(subgrid) = entity_ref.get::<SubGrid>() else { continue };
		let Some(bounds) = entity_ref.get::<VoxelGpuState>().and_then(VoxelGpuState::bounds) else { continue };
		let min = (subgrid.sub_grid_pos() + bounds.min.as_ivec3()).div_euclid(IVec3::splat(CHUNK_SIZE));
		let max = (subgrid.sub_grid_pos() + bounds.max.as_ivec3()).div_euclid(IVec3::splat(CHUNK_SIZE));
		insert_chunk_box(&mut chunks, subgrid.grid(), min, max);
	}

	for &entity in &voxel_camera.lods_to_render {
		let entity_ref = world.entity(entity);
		let (Some(lod), Some(transform), Some(parent), Some(bounds)) = (
			entity_ref.get::<LodVoxels>(),
			entity_ref.get::<Transform>(),
			entity_ref.get::<ChildOf>(),
			entity_ref.get::<VoxelGpuState>().and_then(VoxelGpuState::bounds),
		) else { continue };
		let scale = 1i32 << lod.lod.max(0.0).floor() as u32;
		let origin = transform.translation.as_ivec3();
		let min_voxel = origin + bounds.min.as_ivec3() * scale;
		let max_voxel = origin + bounds.max.as_ivec3() * scale;
		let min = min_voxel.div_euclid(IVec3::splat(CHUNK_SIZE));
		let max = max_voxel.div_euclid(IVec3::splat(CHUNK_SIZE));
		insert_chunk_box(&mut chunks, parent.parent(), min, max);
	}

	chunks
}

fn insert_chunk_box(out: &mut HashSet<(Entity, IVec3)>, grid: Entity, min: IVec3, max: IVec3) {
	for x in min.x..=max.x {
		for y in min.y..=max.y {
			for z in min.z..=max.z {
				out.insert((grid, IVec3::new(x, y, z)));
			}
		}
	}
}

fn tile_contains_chunk(tile: &CoverageDebugTile, chunk: IVec3) -> bool {
	chunk.cmpge(tile.min).all() && chunk.cmplt(tile.min + tile.size).all()
}

struct FuzzRng(u64);

impl FuzzRng {
	fn next(&mut self) -> u64 {
		self.0 ^= self.0 << 13;
		self.0 ^= self.0 >> 7;
		self.0 ^= self.0 << 17;
		self.0
	}

	fn jitter(&mut self, radius: f32) -> f32 {
		let unit = (self.next() >> 11) as f64 / ((1u64 << 53) - 1) as f64;
		((unit as f32) * 2.0 - 1.0) * radius
	}
}
