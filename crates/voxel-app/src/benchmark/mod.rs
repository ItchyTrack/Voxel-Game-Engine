//! Autonomous camera-flight benchmark. See `scripts/raybench.py` for the driver.
//!
//! `voxel-app --benchmark <scenario> --bench-out <dir> [--bench-settings default|full]`
//! opens a real 1920x1080 window with vsync off, flies the scenario's shots, writes a CSV per
//! shot plus screenshots to `<dir>`, and exits.

pub mod dynamic;
pub mod scenarios;

use std::collections::HashMap;
use std::fmt::Write as _;
use std::path::PathBuf;
use std::time::Instant;

use bevy::app::AppExit;
use bevy::prelude::*;
use bevy::render::renderer::RenderAdapterInfo;
use bevy::render::view::screenshot::{Screenshot, save_to_disk};
use bevy::window::{PresentMode, PrimaryWindow, WindowPosition, WindowResolution};
use voxel_engine::{CameraTileLoader, FreezeCameraVoxelLoader};
use voxel_physics::FreezePhysics;
use voxel_ray_renderer::direction_feedback::RenderStats;
use voxel_ray_renderer::gpu_data::RayWorldGpuData;
use voxel_ray_renderer::gpu_timing::{FrameTimings, GpuPassSample, NO_TAG, RenderCpuSample};
use voxel_ray_renderer::graphics_settings::GraphicsSettings;
use voxel_tasks::AsyncPriorityTaskPool;

use scenarios::{ExtraScene, Scenario, Shot, ShotMode};
use voxel_sources::SourceManager;

use crate::scene::scene::{SceneVoxFileSource, res_path, spawn_static_vox};

pub const BENCH_WIDTH: u32 = 1920;
pub const BENCH_HEIGHT: u32 = 1080;

/// Streaming must stay idle this long before a shot starts. Long enough to cover the gap between
/// a grid spawning and its presence data arriving, when nothing is pending yet.
const WARM_STABLE_SECONDS: f32 = 6.0;
const WARM_TIMEOUT_SECONDS: f32 = 600.0;
/// Lets residency feedback converge on the static first view before recording.
const SETTLE_FRAMES: u32 = 90;
const FIXED_STEP_FPS: f32 = 60.0;
const RSS_SAMPLE_INTERVAL: u32 = 30;
const SCREENSHOT_POINTS: [f32; 3] = [0.0, 0.5, 1.0];
const SCREENSHOT_SETTLE_FRAMES: u32 = 10;
const FINAL_DRAIN_FRAMES: u32 = 30;

#[derive(Resource, Clone)]
pub struct BenchConfig {
	pub scenario: &'static Scenario,
	pub out_dir: PathBuf,
	pub full_settings: bool,
	pub screenshots: bool,
}

/// Returns `None` when `--benchmark` wasn't passed; exits on invalid benchmark arguments.
pub fn parse_config() -> Option<BenchConfig> {
	let args: Vec<String> = std::env::args().collect();
	let value = |flag: &str| args.iter().position(|arg| arg == flag).and_then(|at| args.get(at + 1)).cloned();
	let name = value("--benchmark")?;
	let Some(scenario) = scenarios::find(&name) else {
		let known: Vec<_> = scenarios::SCENARIOS.iter().map(|scenario| scenario.name).collect();
		eprintln!("unknown scenario {name}; known: {known:?}");
		std::process::exit(2);
	};
	let full_settings = match value("--bench-settings").as_deref() {
		None | Some("default") => false,
		Some("full") => true,
		Some(other) => {
			eprintln!("unknown --bench-settings {other}");
			std::process::exit(2);
		}
	};
	Some(BenchConfig {
		scenario,
		out_dir: PathBuf::from(value("--bench-out").unwrap_or_else(|| "target/bench-out".into())),
		full_settings,
		screenshots: !args.iter().any(|arg| arg == "--bench-no-screenshots"),
	})
}

pub fn bench_window() -> Window {
	Window {
		title: "voxel benchmark".into(),
		resolution: WindowResolution::new(BENCH_WIDTH, BENCH_HEIGHT),
		position: WindowPosition::At(IVec2::ZERO),
		present_mode: PresentMode::Immediate,
		decorations: false,
		..default()
	}
}

pub struct BenchmarkPlugin(pub BenchConfig);

impl Plugin for BenchmarkPlugin {
	fn build(&self, app: &mut App) {
		std::fs::create_dir_all(&self.0.out_dir).expect("failed to create benchmark output directory");
		app.insert_resource(self.0.clone())
			.init_resource::<Driver>()
			.init_resource::<dynamic::SwarmClock>()
			.add_systems(Update, dynamic::move_swarm.after(drive))
			.add_systems(Startup, spawn_extra_scene)
			.add_systems(First, stamp_frame_start)
			.add_systems(Update, drive)
			.add_systems(Last, stamp_frame_end);
	}
}

#[derive(Default, Clone, Copy)]
struct FrameRow {
	tag: u32,
	frame_ms: f32,
	main_ms: f32,
	rss_kb: u64,
	tree_bytes: u64,
	voxel_bytes: u64,
	bvh_bytes: u64,
	bvh_item_bytes: u64,
	loaded_tiles: u32,
	pending_tiles: u32,
	queue: u32,
}

#[derive(Default, PartialEq, Clone, Copy)]
enum Phase {
	#[default]
	Start,
	Warm,
	Settle,
	Fly,
	Screenshot,
	Drain,
	Done,
}

#[derive(Resource, Default)]
struct Driver {
	phase: Phase,
	shot: usize,
	frame: u32,
	phase_start: Option<Instant>,
	stable_since: Option<Instant>,
	last_loaded: usize,
	frame_start: Option<Instant>,
	last_frame_start: Option<Instant>,
	frame_ms: f32,
	main_ms: f32,
	rows: Vec<FrameRow>,
	gpu: HashMap<u32, GpuPassSample>,
	cpu: HashMap<u32, RenderCpuSample>,
	next_tag: u32,
	warm_seconds: f32,
	rss_kb: u64,
	shot_point: usize,
}

fn spawn_extra_scene(mut commands: Commands, config: Res<BenchConfig>, mut sources: ResMut<SourceManager>) {
	let (file, position) = match config.scenario.extra {
		ExtraScene::None => return,
		ExtraScene::MovingObjects => {
			dynamic::spawn_moving_objects(&mut commands, &mut sources);
			return;
		}
		ExtraScene::Castle => ("castle.vox", scenarios::CASTLE_ORIGIN),
		ExtraScene::RaySponza => ("sponza.vox", scenarios::RAY_SPONZA_ORIGIN),
	};
	let path = res_path(file).unwrap_or_else(|| panic!("benchmark scene res/{file} is missing"));
	let source = sources.get_source_mut::<SceneVoxFileSource>().expect("vox file source not registered");
	spawn_static_vox(&mut commands, source, path, position);
}

fn stamp_frame_start(mut driver: ResMut<Driver>) {
	let now = Instant::now();
	driver.last_frame_start = driver.frame_start.replace(now);
	if let Some(last) = driver.last_frame_start {
		driver.frame_ms = now.duration_since(last).as_secs_f32() * 1e3;
	}
}

fn stamp_frame_end(mut driver: ResMut<Driver>) {
	if let Some(start) = driver.frame_start {
		driver.main_ms = start.elapsed().as_secs_f32() * 1e3;
	}
}

fn read_rss_kb() -> u64 {
	let Ok(statm) = std::fs::read_to_string("/proc/self/statm") else { return 0 };
	statm.split_whitespace().nth(1).and_then(|pages| pages.parse::<u64>().ok()).unwrap_or(0) * 4
}

fn peak_rss_kb() -> u64 {
	let Ok(status) = std::fs::read_to_string("/proc/self/status") else { return 0 };
	status.lines()
		.find_map(|line| line.strip_prefix("VmHWM:"))
		.and_then(|rest| rest.split_whitespace().next()?.parse().ok())
		.unwrap_or(0)
}

fn drive(
	mut commands: Commands,
	config: Res<BenchConfig>,
	mut driver: ResMut<Driver>,
	render_stats: Res<RenderStats>,
	gpu_data: Option<Res<RayWorldGpuData>>,
	adapter: Option<Res<RenderAdapterInfo>>,
	mut settings: ResMut<GraphicsSettings>,
	mut freeze_loader: ResMut<FreezeCameraVoxelLoader>,
	mut freeze_physics: ResMut<FreezePhysics>,
	mut swarm_clock: ResMut<dynamic::SwarmClock>,
	mut cameras: Query<&mut Transform, With<Camera3d>>,
	loaders: Query<&CameraTileLoader>,
	windows: Query<&Window, With<PrimaryWindow>>,
	mut exit: MessageWriter<AppExit>,
) {
	let timings: &FrameTimings = &render_stats.timing;
	for sample in timings.drain_gpu() { driver.gpu.insert(sample.tag, sample); }
	for sample in timings.drain_cpu() {
		let entry = driver.cpu.entry(sample.tag).or_insert(sample);
		if !sample.readback_ms.is_nan() { entry.readback_ms = sample.readback_ms; }
		if !sample.extract_ms.is_nan() { entry.extract_ms = sample.extract_ms; }
	}

	let Ok(mut camera) = cameras.single_mut() else { return };
	let shot: &Shot = &config.scenario.shots[driver.shot.min(config.scenario.shots.len() - 1)];
	let place = |camera: &mut Transform, t: f32| {
		let (pos, look) = scenarios::sample(shot.keys, t);
		*camera = Transform::from_translation(pos).looking_at(look, Vec3::Y);
	};

	let (loaded, pending) = loaders.iter().fold((0usize, 0usize), |(loaded, pending), loader| {
		(loaded + loader.loaded_tiles().count(), pending + loader.pending_tiles().count())
	});
	let queue = AsyncPriorityTaskPool::get().len();

	match driver.phase {
		Phase::Start => {
			settings.shadows = config.full_settings;
			settings.anti_aliasing = config.full_settings;
			timings.set_enabled(true);
			timings.set_tag(NO_TAG);
			driver.phase = Phase::Warm;
			driver.phase_start = Some(Instant::now());
			driver.stable_since = None;
			freeze_loader.0 = false;
			// Physics stays frozen until recording, so falling objects are still falling in the shot.
			freeze_physics.0 = true;
			swarm_clock.0 = 0.0;
			place(&mut camera, 0.0);
		}
		Phase::Warm => {
			place(&mut camera, 0.0);
			let idle = pending == 0 && queue == 0 && loaded > 0 && loaded == driver.last_loaded;
			driver.last_loaded = loaded;
			if !idle { driver.stable_since = None; }
			else if driver.stable_since.is_none() { driver.stable_since = Some(Instant::now()); }
			let waited = driver.phase_start.unwrap().elapsed().as_secs_f32();
			let stable = driver.stable_since.is_some_and(|since| since.elapsed().as_secs_f32() >= WARM_STABLE_SECONDS);
			if stable || waited >= WARM_TIMEOUT_SECONDS {
				if !stable { warn!("benchmark warm-up timed out for {}: {pending} tiles pending", shot.name); }
				driver.warm_seconds = waited - if stable { WARM_STABLE_SECONDS } else { 0.0 };
				info!("shot {}: warm in {:.1}s with {loaded} tiles", shot.name, driver.warm_seconds);
				freeze_loader.0 = shot.mode == ShotMode::Frozen;
				driver.phase = Phase::Settle;
				driver.frame = 0;
			}
		}
		Phase::Settle => {
			place(&mut camera, 0.0);
			driver.frame += 1;
			if driver.frame >= SETTLE_FRAMES {
				driver.phase = Phase::Fly;
				freeze_physics.0 = shot.mode == ShotMode::Frozen;
				driver.phase_start = Some(Instant::now());
				driver.frame = 0;
				driver.rss_kb = read_rss_kb();
			}
		}
		Phase::Fly => {
			let total_frames = (shot.seconds * FIXED_STEP_FPS) as u32;
			let t = match shot.mode {
				ShotMode::Frozen => driver.frame as f32 / total_frames as f32,
				ShotMode::Live => driver.phase_start.unwrap().elapsed().as_secs_f32() / shot.seconds,
			};
			place(&mut camera, t);
			swarm_clock.0 = t * shot.seconds;
			if driver.frame % RSS_SAMPLE_INTERVAL == 0 { driver.rss_kb = read_rss_kb(); }
			let (tree_bytes, voxel_bytes) = gpu_data.as_ref().map_or((0, 0), |gpu| {
				let gpu = gpu.lock();
				(gpu.trees.held_bytes() as u64, gpu.voxels.held_bytes() as u64)
			});
			let (bvh_bytes, bvh_item_bytes) = render_stats.inner.lock().map(|stats| (stats.bvh_bytes, stats.bvh_leaf_bytes)).unwrap_or_default();
			let tag = driver.next_tag;
			driver.next_tag += 1;
			timings.set_tag(tag);
			// The frame that just ended belongs to the previous tag, so its wall time is stored there.
			let frame_ms = driver.frame_ms;
			let main_ms = driver.main_ms;
			if driver.frame > 0 && let Some(previous) = driver.rows.last_mut() {
				previous.frame_ms = frame_ms;
				previous.main_ms = main_ms;
			}
			let rss_kb = driver.rss_kb;
			driver.rows.push(FrameRow {
				tag,
				frame_ms: 0.0,
				main_ms: 0.0,
				rss_kb,
				tree_bytes,
				voxel_bytes,
				bvh_bytes,
				bvh_item_bytes,
				loaded_tiles: loaded as u32,
				pending_tiles: pending as u32,
				queue: queue as u32,
			});
			driver.frame += 1;
			let finished = match shot.mode {
				ShotMode::Frozen => driver.frame > total_frames,
				ShotMode::Live => t >= 1.0,
			};
			if finished {
				timings.set_tag(NO_TAG);
				driver.frame = 0;
				driver.shot_point = 0;
				driver.phase = if config.screenshots && shot.mode == ShotMode::Frozen { Phase::Screenshot } else { Phase::Drain };
			}
		}
		Phase::Screenshot => {
			place(&mut camera, SCREENSHOT_POINTS[driver.shot_point]);
			swarm_clock.0 = SCREENSHOT_POINTS[driver.shot_point] * shot.seconds;
			driver.frame += 1;
			if driver.frame == SCREENSHOT_SETTLE_FRAMES {
				let path = config.out_dir.join(format!("{}_{}.png", shot.name, driver.shot_point));
				commands.spawn(Screenshot::primary_window()).observe(save_to_disk(path));
			} else if driver.frame >= SCREENSHOT_SETTLE_FRAMES * 2 {
				driver.frame = 0;
				driver.shot_point += 1;
				if driver.shot_point == SCREENSHOT_POINTS.len() { driver.phase = Phase::Drain; }
			}
		}
		Phase::Drain => {
			driver.frame += 1;
			if driver.frame < FINAL_DRAIN_FRAMES { return; }
			let window = windows.single().ok();
			write_shot(&config, &driver, shot, adapter.as_deref(), window);
			driver.rows.clear();
			driver.gpu.clear();
			driver.cpu.clear();
			driver.shot += 1;
			driver.frame = 0;
			if driver.shot >= config.scenario.shots.len() {
				driver.phase = Phase::Done;
				exit.write(AppExit::Success);
			} else {
				driver.phase = Phase::Start;
			}
		}
		Phase::Done => {}
	}
}

fn write_shot(config: &BenchConfig, driver: &Driver, shot: &Shot, adapter: Option<&RenderAdapterInfo>, window: Option<&Window>) {
	let mut csv = String::new();
	let adapter_name = adapter.map_or("unknown".to_string(), |adapter| adapter.name.clone());
	let (width, height, present) = window.map_or((0, 0, "unknown".to_string()), |window| {
		(window.physical_width(), window.physical_height(), format!("{:?}", window.present_mode))
	});
	let _ = writeln!(csv, "# scenario={} shot={} settings={} mode={} adapter={} window={}x{} present={} warm_seconds={:.2} peak_rss_kb={}",
		config.scenario.name, shot.name, if config.full_settings { "full" } else { "default" },
		if shot.mode == ShotMode::Frozen { "frozen" } else { "live" },
		adapter_name.replace(' ', "_"), width, height, present, driver.warm_seconds, peak_rss_kb());
	csv.push_str("frame,frame_ms,main_ms,readback_ms,extract_ms,gpu_beam_ms,gpu_ray_ms,gpu_color_ms,rss_kb,tree_bytes,voxel_bytes,bvh_bytes,bvh_item_bytes,loaded_tiles,pending_tiles,queue\n");
	for (index, row) in driver.rows.iter().enumerate() {
		// The last row's frame never finished inside the flight.
		if row.frame_ms == 0.0 { continue; }
		let gpu = driver.gpu.get(&row.tag);
		let cpu = driver.cpu.get(&row.tag);
		let opt = |value: Option<f32>| value.filter(|value| !value.is_nan()).map_or(String::new(), |value| format!("{value:.4}"));
		let _ = writeln!(csv, "{index},{:.4},{:.4},{},{},{},{},{},{},{},{},{},{},{},{},{}",
			row.frame_ms, row.main_ms,
			opt(cpu.map(|sample| sample.readback_ms)), opt(cpu.map(|sample| sample.extract_ms)),
			opt(gpu.map(|sample| sample.beam_ms)), opt(gpu.map(|sample| sample.ray_ms)), opt(gpu.map(|sample| sample.color_ms)),
			row.rss_kb, row.tree_bytes, row.voxel_bytes, row.bvh_bytes, row.bvh_item_bytes,
			row.loaded_tiles, row.pending_tiles, row.queue);
	}
	let path = config.out_dir.join(format!("{}.csv", shot.name));
	std::fs::write(&path, csv).expect("failed to write benchmark csv");
	info!("wrote {}", path.display());
}
