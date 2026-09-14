use std::{path::PathBuf, time::Instant};

use basic_voxel::{BasicVoxel, BasicVoxelPlugin};
use bevy::{camera::{Hdr, RenderTarget}, prelude::*, render::{RenderPlugin, settings::{RenderCreation, WgpuSettings}, view::{Msaa, screenshot::{Screenshot, ScreenshotCaptured, save_to_disk}}}, window::WindowResolution};
use tile_data::{NonZeroChunkRegion, TileBuildingParameters};
use voxel_content::{StreamingVoxels, VoxelStoreSource, VoxelStoreSourcePlugin};
use voxel_data::{grid::Grid, voxels::VoxelType};
use voxel_engine::{VoxelEngineMode, VoxelEnginePlugins};
use voxel_gpu::RenderingContext;
use voxel_ray_renderer::{RayRenderingType, direction_feedback::RenderStats, graphics_settings::GraphicsSettings};
use voxel_sources::{SourceManager, edit::GridEditIdManager};
use voxel_streaming::GridStreaming;

#[derive(Component)]
struct TestRoom;

#[derive(Resource)]
struct Smoke {
	phase: u32,
	ticks: u32,
	captures: u32,
	started: Instant,
	reference: Option<Vec<u8>>,
	resized: Option<Vec<u8>>,
	output: PathBuf,
	target: Handle<Image>,
}

fn main() {
	let runtime = tokio::runtime::Runtime::new().unwrap();
	let _guard = runtime.enter();
	let args = std::env::args().skip(1).collect::<Vec<_>>();
	let overflow = args.iter().any(|arg| arg == "--overflow");
	let output = args.iter().find(|arg| !arg.starts_with("--")).map(PathBuf::from)
		.unwrap_or_else(|| std::env::temp_dir().join(if overflow { "face-gi-overflow" } else { "face-gi-smoke" }));
	std::fs::create_dir_all(&output).unwrap();
	if args.iter().any(|arg| arg == "--app") {
		let mut app = voxel_app::build_app(Window { resolution: WindowResolution::new(800, 600), ..default() });
		app.insert_resource(GraphicsSettings { shadows: true, anti_aliasing: false, face_gi: true })
			.insert_resource(voxel_physics::FreezePhysics(true))
			.insert_resource(Smoke { phase: 0, ticks: 0, captures: 0, started: Instant::now(), reference: None, resized: None, output, target: default() })
			.add_systems(Update, advance_app);
		app.run();
		return;
	}
	let constrained_limits = overflow.then(|| wgpu::Limits {
		max_compute_workgroups_per_dimension: 192,
		max_binding_array_elements_per_shader_stage: u32::MAX,
		max_binding_array_sampler_elements_per_shader_stage: u32::MAX,
		..default()
	});
	let mut app = App::new();
	app.add_plugins(DefaultPlugins.set(WindowPlugin {
			primary_window: Some(Window { title: "Face GI smoke test".into(), resolution: WindowResolution::new(800, 600), ..default() }),
			..default()
		}).set(RenderPlugin {
			render_creation: RenderCreation::Automatic(Box::new(WgpuSettings { constrained_limits, ..default() })),
			..default()
		}))
		.add_plugins(VoxelEnginePlugins { mode: VoxelEngineMode::Host })
		.add_plugins((BasicVoxelPlugin, VoxelStoreSourcePlugin))
		.insert_resource(GraphicsSettings { shadows: true, anti_aliasing: false, face_gi: true })
		.insert_resource(Smoke { phase: 0, ticks: 0, captures: 0, started: Instant::now(), reference: None, resized: None, output, target: default() })
		.add_systems(Startup, setup);
	if overflow { app.add_systems(Update, advance_overflow); }
	else { app.add_systems(Update, advance); }
	app.run();
}

fn setup(mut commands: Commands, mut sources: ResMut<SourceManager>, ray: Res<RayRenderingType>, mut images: ResMut<Assets<Image>>, mut smoke: ResMut<Smoke>) {
	// An image target makes resize checks independent of the window manager.
	smoke.target = images.add(Image::new_target_texture(800, 600, wgpu::TextureFormat::Rgba8UnormSrgb, None));
	commands.spawn(Camera2d);
	commands.spawn((ImageNode::new(smoke.target.clone()), Node { width: percent(100), height: percent(100), ..default() }));
	commands.spawn((
		Camera3d::default(), Camera { order: -1, ..default() }, Hdr, Msaa::Off,
		RenderTarget::Image(smoke.target.clone().into()),
		Transform::from_xyz(0.0, 10.0, 30.0).looking_at(Vec3::new(0.0, 5.0, 0.0), Vec3::Y),
	));
	let mut voxels = StreamingVoxels::new::<BasicVoxel>();
	for x in -12..=12 { for y in 0..=16 { for z in -12..=12 {
		let color = if y == 0 || z == -12 { [220, 220, 220, 255] }
			else if x == -12 { [230, 35, 25, 255] }
			else if x == 12 { [35, 210, 45, 255] }
			else if (-3..=3).contains(&x) && y <= 5 && (-2..=3).contains(&z) { [160, 160, 160, 255] }
			else { continue; };
		voxels.add_voxel(IVec3::new(x, y, z), BasicVoxel { color, mass: 1 }.get_ref());
	}}}
	let mut streaming = GridStreaming::default();
	for chunk in voxels.chunk_positions() {
		streaming.mark_present_area(NonZeroChunkRegion::from_single(chunk));
	}
	let grid = commands.spawn((
		TestRoom, Transform::IDENTITY, Grid::new::<BasicVoxel>(), GridEditIdManager::default(), streaming,
		TileBuildingParameters::new(RenderingContext { rendering_type: ray.0 }),
	)).id();
	sources.get_source_mut::<VoxelStoreSource>().unwrap().insert_chunk_data(grid, voxels.into_chunk_data());
}

fn capture(commands: &mut Commands, smoke: &Smoke, label: &'static str) {
	commands.spawn(Screenshot::image(smoke.target.clone()))
		.observe(save_to_disk(smoke.output.join(format!("{label}.png"))))
		.observe(move |event: On<ScreenshotCaptured>, mut smoke: ResMut<Smoke>| {
			let pixels = event.image.data.as_ref().expect("screenshot must have pixels");
			assert!(pixels.chunks_exact(4).any(|pixel| pixel != &pixels[..4]), "blank screenshot");
			let expected_size = if matches!(label, "gi-resized" | "gi-moved" | "gi-unmoved") { (640, 480) } else { (800, 600) };
			assert_eq!((event.image.width(), event.image.height()), expected_size, "render target did not resize");
			match label {
				"gi-on" | "overflow" => smoke.reference = Some(pixels.clone()),
				"gi-repeat" | "gi-restored" | "overflow-legacy" => assert!(pixels == smoke.reference.as_ref().unwrap(), "rendered output did not match reference"),
				"gi-off" => {
					let lit = smoke.reference.as_ref().unwrap();
					assert!(pixels != lit, "GI toggle had no visible effect");
					let tinted = pixels.chunks_exact(4).zip(lit.chunks_exact(4)).filter(|(direct, bounced)| {
						let gray = direct[..3].iter().max().unwrap() - direct[..3].iter().min().unwrap() <= 1;
						let color = bounced[..3].iter().max().unwrap() - bounced[..3].iter().min().unwrap() >= 5;
						gray && direct[0] > 30 && direct[0] < 240 && color
					}).count();
					assert!(tinted > 100, "colored bounce lighting did not reach neutral surfaces");
					println!("GI_SMOKE color bleed: {tinted} neutral-surface pixels");
				},
				"gi-aa" => assert!(pixels != smoke.reference.as_ref().unwrap(), "AA had no visible effect"),
				"gi-resized" => smoke.resized = Some(pixels.clone()),
				"gi-moved" => assert!(pixels != smoke.resized.as_ref().unwrap(), "grid transform had no visible effect"),
				"gi-unmoved" => assert!(pixels == smoke.resized.as_ref().unwrap(), "lighting stayed stale after restoring the grid without resizing"),
				_ => {},
			}
			smoke.captures += 1;
			println!("GI_SMOKE capture {label}: {} bytes", pixels.len());
		});
}

fn advance_app(
	mut commands: Commands,
	mut smoke: ResMut<Smoke>,
	stats: Res<RenderStats>,
	mut settings: ResMut<GraphicsSettings>,
	mut exit: MessageWriter<AppExit>,
) {
	assert!(smoke.started.elapsed().as_secs() < 180, "application smoke test timed out");
	let gi = stats.inner.lock().unwrap().face_gi;
	smoke.ticks += 1;
	if smoke.phase == 0 && gi.visible_faces == 0 { smoke.ticks = 0; return; }
	if smoke.ticks < 120 { return; }
	println!("GI_APP phase {} after {:.2}s: {gi:?}", smoke.phase, smoke.started.elapsed().as_secs_f32());
	match smoke.phase {
		0 | 2 | 4 => {
			let label = match smoke.phase { 0 => "app-gi-on", 2 => "app-gi-aa", _ => "app-gi-off" };
			commands.spawn(Screenshot::primary_window()).observe(save_to_disk(smoke.output.join(format!("{label}.png"))));
			assert_eq!(gi.enabled, settings.face_gi);
			if gi.enabled { assert!(gi.visible_faces > 0); }
			else { assert_eq!((gi.visible_faces, gi.total_faces, gi.overflow_flags), (0, 0, 0)); }
		},
		1 => settings.anti_aliasing = true,
		3 => settings.face_gi = false,
		5 => { println!("GI_APP PASS: application rendered GI on, AA, and GI off"); exit.write(AppExit::Success); },
		_ => {},
	}
	smoke.phase += 1;
	smoke.ticks = 0;
}

fn advance_overflow(
	mut commands: Commands,
	mut smoke: ResMut<Smoke>,
	stats: Res<RenderStats>,
	mut settings: ResMut<GraphicsSettings>,
	mut exit: MessageWriter<AppExit>,
) {
	assert!(smoke.started.elapsed().as_secs() < 180, "overflow smoke test timed out");
	let gi = stats.inner.lock().unwrap().face_gi;
	smoke.ticks += 1;
	if smoke.phase == 0 && gi.overflow_flags == 0 { smoke.ticks = 0; return; }
	if smoke.ticks < 60 { return; }
	println!("GI_OVERFLOW phase {}: {gi:?}", smoke.phase);
	match smoke.phase {
		0 => {
			assert_eq!(gi.visible_capacity, 1024);
			assert_ne!(gi.overflow_flags & 2, 0);
			capture(&mut commands, &smoke, "overflow");
		},
		1 => settings.face_gi = false,
		2 => {
			assert!(!gi.enabled);
			assert_eq!(gi.overflow_flags, 0);
			capture(&mut commands, &smoke, "overflow-legacy");
		},
		3 => settings.face_gi = true,
		4 => {
			assert_ne!(gi.overflow_flags & 2, 0);
			assert_eq!(smoke.captures, 2);
			println!("GI_OVERFLOW PASS: capacity fallback matches legacy rendering byte-for-byte");
			exit.write(AppExit::Success);
		},
		_ => {},
	}
	smoke.phase += 1;
	smoke.ticks = 0;
}

fn advance(
	mut commands: Commands,
	mut smoke: ResMut<Smoke>,
	stats: Res<RenderStats>,
	mut settings: ResMut<GraphicsSettings>,
	mut images: ResMut<Assets<Image>>,
	mut rooms: Query<&mut Transform, With<TestRoom>>,
	mut exit: MessageWriter<AppExit>,
) {
	assert!(smoke.started.elapsed().as_secs() < 180, "GI smoke test timed out");
	let gi = stats.inner.lock().unwrap().face_gi;
	smoke.ticks += 1;
	if smoke.phase == 0 && (gi.visible_faces == 0 || gi.total_faces <= gi.visible_faces || gi.incomplete_rays != 0) {
		smoke.ticks = 0;
		return;
	}
	let delay = if matches!(smoke.phase, 0 | 11 | 13) { 60 } else { 20 };
	if smoke.ticks < delay { return; }
	if settings.face_gi {
		assert!(gi.enabled && gi.visible_faces > 0, "GI did not render");
		assert_eq!(gi.overflow_flags, 0, "small test scene overflowed");
		assert!(gi.total_faces >= gi.visible_faces && gi.total_faces <= gi.table_capacity);
		assert_eq!(gi.secondary_hits + gi.sky_misses + gi.incomplete_rays, gi.visible_faces * 8);
	}
	println!("GI_SMOKE phase {} after {:.2}s: {gi:?}", smoke.phase, smoke.started.elapsed().as_secs_f32());
	match smoke.phase {
		0 => capture(&mut commands, &smoke, "gi-on"),
		1 => capture(&mut commands, &smoke, "gi-repeat"),
		2 => settings.face_gi = false,
		3 => {
			assert!(!gi.enabled);
			assert_eq!((gi.total_faces, gi.visible_faces, gi.overflow_flags), (0, 0, 0));
			capture(&mut commands, &smoke, "gi-off");
		},
		4 => { settings.face_gi = true; settings.anti_aliasing = true; },
		5 => capture(&mut commands, &smoke, "gi-aa"),
		6 => images.get_mut(&smoke.target).unwrap().resize(wgpu::Extent3d { width: 640, height: 480, depth_or_array_layers: 1 }),
		7 => capture(&mut commands, &smoke, "gi-resized"),
		8 => {
			*rooms.single_mut().unwrap() = Transform::from_xyz(0.75, 0.0, 0.0)
				.with_rotation(Quat::from_rotation_y(0.25)).with_scale(Vec3::splat(1.25));
		},
		9 => capture(&mut commands, &smoke, "gi-moved"),
		10 => *rooms.single_mut().unwrap() = Transform::IDENTITY,
		11 => capture(&mut commands, &smoke, "gi-unmoved"),
		12 => {
			settings.anti_aliasing = false;
			images.get_mut(&smoke.target).unwrap().resize(wgpu::Extent3d { width: 800, height: 600, depth_or_array_layers: 1 });
		},
		13 => capture(&mut commands, &smoke, "gi-restored"),
		14 => {
			assert_eq!(smoke.captures, 8);
			println!("GI_SMOKE PASS: color bleed, GI toggle, AA, resize, transforms, and repeatable frame-local output");
			exit.write(AppExit::Success);
		},
		_ => {},
	}
	smoke.phase += 1;
	smoke.ticks = 0;
}
