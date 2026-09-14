use std::{collections::{HashMap, HashSet}, path::PathBuf, time::Instant};

use basic_voxel::{BasicVoxel, BasicVoxelPlugin};
use bevy::{camera::{Hdr, RenderTarget}, prelude::*, render::{Render, RenderApp, RenderSystems, renderer::RenderQueue, view::{Msaa, screenshot::{Screenshot, ScreenshotCaptured, save_to_disk}}}, window::WindowResolution};
use tile_data::{NonZeroChunkRegion, TileBuildingParameters};
use voxel_content::{StreamingVoxels, VoxelStoreSource, VoxelStoreSourcePlugin};
use voxel_data::{grid::Grid, voxels::VoxelType};
use voxel_engine::{VoxelEngineMode, VoxelEnginePlugins};
use voxel_gpu::RenderingContext;
use voxel_ray_renderer::{RayRenderingType, direction_feedback::RenderStats, graphics_settings::{DirectRayCount, GiRayCount, GraphicsSettings, LightingMode}, render_node::prepare_voxel_view_bind_groups, voxel_renderer_resource::VoxelViewResources};
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
	images: HashMap<&'static str, Vec<u8>>,
	output: PathBuf,
	target: Handle<Image>,
}

#[derive(Resource)]
struct ForceVisibleLimit(Option<u32>);

fn main() {
	let runtime = tokio::runtime::Runtime::new().unwrap();
	let _guard = runtime.enter();
	let args = std::env::args().skip(1).collect::<Vec<_>>();
	let overflow = args.iter().any(|arg| arg == "--overflow");
	let output = args.iter().find(|arg| !arg.starts_with("--")).map(PathBuf::from)
		.unwrap_or_else(|| std::env::temp_dir().join(if overflow { "face-gi-overflow" } else { "face-gi-smoke" }));
	std::fs::create_dir_all(&output).unwrap();
	let smoke = Smoke { phase: 0, ticks: 0, captures: 0, started: Instant::now(), images: default(), output, target: default() };
	let settings = GraphicsSettings { lighting: LightingMode::Indirect, ..default() };
	if args.iter().any(|arg| arg == "--app" || arg == "--angles") {
		let angles = args.iter().any(|arg| arg == "--angles");
		let mut app = voxel_app::build_app(Window { resolution: WindowResolution::new(800, 600), ..default() });
		app.insert_resource(settings)
			.insert_resource(voxel_physics::FreezePhysics(true))
			.insert_resource(smoke);
		if angles {
			app.add_systems(PostStartup, setup_angle_target).add_systems(Update, advance_angles);
		} else { app.add_systems(Update, advance_app); }
		app.run();
		return;
	}
	let mut app = App::new();
	app.add_plugins(DefaultPlugins.set(WindowPlugin {
			primary_window: Some(Window { title: "Face lighting smoke test".into(), resolution: WindowResolution::new(800, 600), ..default() }),
			..default()
		}))
		.add_plugins(VoxelEnginePlugins { mode: VoxelEngineMode::Host })
		.add_plugins((BasicVoxelPlugin, VoxelStoreSourcePlugin))
		.insert_resource(settings)
		.insert_resource(smoke)
		.add_systems(Startup, setup);
	app.sub_app_mut(RenderApp)
		.insert_resource(ForceVisibleLimit(overflow.then_some(1024)))
		.add_systems(Render, limit_test_arena.after(prepare_voxel_view_bind_groups).in_set(RenderSystems::PrepareBindGroups));
	if overflow { app.add_systems(Update, advance_overflow); }
	else { app.add_systems(Update, advance); }
	app.run();
}

fn limit_test_arena(limit: Res<ForceVisibleLimit>, queue: Res<RenderQueue>, mut views: Query<&mut VoxelViewResources>) {
	for mut view in &mut views {
		let Some(renderer) = view.voxel_renderer.as_mut() else { continue; };
		let gi = &mut renderer.face_gi;
		// Exercise 2D dispatches, and optionally overflow, without huge test scenes.
		gi.arena.params.settings[1] = gi.arena.params.settings[1].min(192);
		if let Some(capacity) = limit.0 { gi.arena.params.sizes[1] = gi.arena.params.sizes[1].min(capacity); }
		let params = gi.arena.params;
		let bytes = params.sizes.into_iter().chain(params.offsets).chain(params.settings)
			.flat_map(u32::to_le_bytes).collect::<Vec<_>>();
		queue.write_buffer(&gi.params, 0, &bytes);
	}
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

fn capture(commands: &mut Commands, smoke: &Smoke, label: &'static str, equal_to: Option<&'static str>) {
	commands.spawn(Screenshot::image(smoke.target.clone()))
		.observe(save_to_disk(smoke.output.join(format!("{label}.png"))))
		.observe(move |event: On<ScreenshotCaptured>, mut smoke: ResMut<Smoke>| {
			let pixels = event.image.data.as_ref().expect("screenshot must have pixels");
			assert!(pixels.chunks_exact(4).any(|pixel| pixel != &pixels[..4]), "blank screenshot");
			let expected_size = if matches!(label, "gi-resized" | "gi-moved" | "gi-unmoved") { (640, 480) } else { (800, 600) };
			assert_eq!((event.image.width(), event.image.height()), expected_size, "render target did not resize");
			if let Some(reference) = equal_to {
				assert!(pixels == &smoke.images[reference], "{label} did not match {reference}");
			}
			match label {
				"direct" => {
					let lit = &smoke.images["gi-on"];
					let tinted = pixels.chunks_exact(4).zip(lit.chunks_exact(4)).filter(|(direct, bounced)| {
						let gray = direct[..3].iter().max().unwrap() - direct[..3].iter().min().unwrap() <= 1;
						let color = bounced[..3].iter().max().unwrap() - bounced[..3].iter().min().unwrap() >= 5;
						gray && direct[0] > 30 && direct[0] < 240 && color
					}).count();
					assert!(tinted > 100, "colored bounce lighting did not reach neutral surfaces");
					println!("GI_SMOKE color bleed: {tinted} neutral-surface pixels");
					assert!(!pixels.chunks_exact(4).any(|pixel| pixel[..3] == [0, 0, 0]), "Direct lost its ambient fill");
				},
				"direct-4" | "direct-9" => {
					let colors = smoke.images["direct"].chunks_exact(4)
						.map(|pixel| [pixel[0], pixel[1], pixel[2]]).collect::<HashSet<_>>();
					let partial = pixels.chunks_exact(4)
						.filter(|pixel| !colors.contains(&[pixel[0], pixel[1], pixel[2]])).count();
					assert!(partial > 100, "extra direct rays did not produce partial shadow coverage");
					if label == "direct-9" { assert!(pixels != &smoke.images["direct-4"], "nine rays matched four"); }
					println!("GI_SMOKE {label}: {partial} partial-shadow pixels");
				},
				"gi-direct-4" | "gi-direct-9" => assert!(pixels != &smoke.images["gi-on"], "direct ray count had no effect in GI"),
				"gi-direct-9-aa" => assert!(pixels != &smoke.images["gi-direct-9"], "AA had no effect with nine direct rays"),
				"normal-shading" => {
					assert!(pixels != &smoke.images["direct"], "Nothing still includes shadows");
					// The gray block has the same material on its top and front faces.
					let top = pixels[(288 * 800 + 400) * 4];
					let front = pixels[(340 * 800 + 400) * 4];
					assert!(top > front.saturating_add(10), "Nothing did not shade by sun normal");
				},
				"gi-aa" => assert!(pixels != &smoke.images["gi-on"], "AA had no visible effect"),
				"gi-moved" => assert!(pixels != &smoke.images["gi-resized"], "grid transform had no visible effect"),
				"gi-8" | "gi-16" | "gi-64" => assert!(pixels != &smoke.images["gi-on"], "ray count had no visible effect"),
				_ => {},
			}
			smoke.images.insert(label, pixels.clone());
			smoke.captures += 1;
			println!("GI_SMOKE capture {label}: {} bytes", pixels.len());
		});
}

fn assert_mode(stats: &RenderStats, settings: &GraphicsSettings) {
	let gi = stats.inner.lock().unwrap().face_gi;
	assert_eq!(gi.enabled, settings.lighting != LightingMode::None);
	assert_eq!(gi.indirect, settings.lighting == LightingMode::Indirect);
	assert_eq!(gi.rays_per_face, settings.gi_rays.count());
	assert_eq!(gi.overflow_flags, 0, "small test scene overflowed");
	match settings.lighting {
		LightingMode::None => assert_eq!((gi.total_faces, gi.visible_faces, gi.secondary_hits, gi.sky_misses, gi.incomplete_rays), (0, 0, 0, 0, 0)),
		LightingMode::Direct => {
			assert!(gi.visible_faces > 0);
			assert_eq!(gi.total_faces, gi.visible_faces);
			assert_eq!((gi.secondary_hits, gi.sky_misses, gi.incomplete_rays), (0, 0, 0));
		},
		LightingMode::Indirect => {
			assert!(gi.visible_faces > 0 && gi.total_faces >= gi.visible_faces && gi.total_faces <= gi.table_capacity);
			assert_eq!(gi.secondary_hits + gi.sky_misses + gi.incomplete_rays, gi.visible_faces * settings.gi_rays.count());
		},
	}
}

fn setup_angle_target(mut commands: Commands, mut smoke: ResMut<Smoke>, mut images: ResMut<Assets<Image>>, cameras: Query<Entity, With<Camera3d>>) {
	smoke.target = images.add(Image::new_target_texture(1686, 948, wgpu::TextureFormat::Rgba8UnormSrgb, None));
	for camera in &cameras { commands.entity(camera).insert(RenderTarget::Image(smoke.target.clone().into())); }
}

fn advance_angles(mut commands: Commands, mut smoke: ResMut<Smoke>, stats: Res<RenderStats>, mut cameras: Query<&mut Transform, With<Camera3d>>, mut exit: MessageWriter<AppExit>) {
	assert!(smoke.started.elapsed().as_secs() < 180, "angle check timed out");
	if smoke.phase == 24 {
		smoke.ticks += 1;
		if smoke.ticks >= 30 {
			println!("GI_ANGLE finished: {} views overflowed", smoke.captures);
			exit.write(AppExit::Success);
		}
		return;
	}
	let yaw = [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0][smoke.phase as usize % 8];
	let pitch = [-0.35, -0.5, -0.7][(smoke.phase as usize / 8) % 3];
	let position = Vec3::new(0.0, 150.0, 400.0);
	for mut camera in &mut cameras {
		*camera = Transform::from_translation(position).with_rotation(Quat::from_euler(EulerRot::YXZ, yaw, pitch, 0.0));
	}
	smoke.ticks += 1;
	let gi = stats.inner.lock().unwrap().face_gi;
	if gi.visible_faces == 0 && smoke.phase == 0 { smoke.ticks = 0; return; }
	if smoke.ticks < 90 { return; }
	println!("GI_ANGLE position={position:?} yaw={yaw} pitch={pitch}: {gi:?}");
	if gi.overflow_flags != 0 { smoke.captures += 1; }
	commands.spawn(Screenshot::image(smoke.target.clone()))
		.observe(save_to_disk(smoke.output.join(format!("angle-{}.png", smoke.phase))));
	smoke.phase += 1;
	smoke.ticks = 0;
}

fn advance_app(mut commands: Commands, mut smoke: ResMut<Smoke>, stats: Res<RenderStats>, mut settings: ResMut<GraphicsSettings>, mut exit: MessageWriter<AppExit>) {
	assert!(smoke.started.elapsed().as_secs() < 180, "application smoke test timed out");
	let gi = stats.inner.lock().unwrap().face_gi;
	smoke.ticks += 1;
	if smoke.phase == 0 && gi.visible_faces == 0 { smoke.ticks = 0; return; }
	if smoke.ticks < 120 { return; }
	println!("GI_APP phase {} after {:.2}s: {gi:?}", smoke.phase, smoke.started.elapsed().as_secs_f32());
	assert_mode(&stats, &settings);
	match smoke.phase {
		0 | 2 | 4 | 6 | 8 => {
			let label = match smoke.phase { 0 => "app-gi-on", 2 => "app-direct-4", 4 => "app-normal-shading", 6 => "app-gi-aa", _ => "app-gi64-direct9-aa" };
			commands.spawn(Screenshot::primary_window()).observe(save_to_disk(smoke.output.join(format!("{label}.png"))));
		},
		1 => { settings.lighting = LightingMode::Direct; settings.direct_rays = DirectRayCount::Four; },
		3 => settings.lighting = LightingMode::None,
		5 => { settings.lighting = LightingMode::Indirect; settings.anti_aliasing = true; },
		7 => { settings.gi_rays = GiRayCount::SixtyFour; settings.direct_rays = DirectRayCount::Nine; },
		9 => { println!("GI_APP PASS: all lighting modes, AA, 64 GI rays, and 1/4/9 direct rays"); exit.write(AppExit::Success); },
		_ => {},
	}
	smoke.phase += 1;
	smoke.ticks = 0;
}

fn advance_overflow(mut commands: Commands, mut smoke: ResMut<Smoke>, stats: Res<RenderStats>, mut settings: ResMut<GraphicsSettings>, mut rooms: Query<&mut Transform, With<TestRoom>>, mut exit: MessageWriter<AppExit>) {
	assert!(smoke.started.elapsed().as_secs() < 180, "overflow smoke test timed out");
	let gi = stats.inner.lock().unwrap().face_gi;
	smoke.ticks += 1;
	let batch = smoke.phase / 11;
	let phase = smoke.phase % 11;
	if phase == 0 && gi.overflow_flags == 0 { smoke.ticks = 0; return; }
	if smoke.ticks < 60 { return; }
	println!("GI_OVERFLOW phase {}, direct rays {}: {gi:?}", smoke.phase, settings.direct_rays.count());
	let labels = [
		["overflow", "overflow-direct", "overflow-aa", "overflow-direct-aa"],
		["overflow-4", "overflow-direct-4", "overflow-4-aa", "overflow-direct-4-aa"],
		["overflow-9", "overflow-direct-9", "overflow-9-aa", "overflow-direct-9-aa"],
		["moved-overflow", "moved-direct", "moved-overflow-aa", "moved-direct-aa"],
		["moved-overflow-4", "moved-direct-4", "moved-overflow-4-aa", "moved-direct-4-aa"],
		["moved-overflow-9", "moved-direct-9", "moved-overflow-9-aa", "moved-direct-9-aa"],
	][batch as usize];
	if settings.lighting == LightingMode::Indirect {
		assert_eq!(gi.visible_capacity, 1024);
		assert_ne!(gi.overflow_flags & 2, 0);
	} else { assert_mode(&stats, &settings); }
	match phase {
		0 => capture(&mut commands, &smoke, labels[0], None),
		1 => settings.lighting = LightingMode::Direct,
		2 => capture(&mut commands, &smoke, labels[1], Some(labels[0])),
		3 => settings.lighting = LightingMode::None,
		4 => {},
		5 => { settings.lighting = LightingMode::Indirect; settings.anti_aliasing = true; },
		6 => capture(&mut commands, &smoke, labels[2], None),
		7 => settings.lighting = LightingMode::Direct,
		8 => capture(&mut commands, &smoke, labels[3], Some(labels[2])),
		9 => settings.lighting = LightingMode::Indirect,
		10 => {
			assert_eq!(smoke.captures, (batch + 1) * 4);
			if batch == 5 {
				println!("GI_OVERFLOW PASS: fallback matches Direct byte-for-byte for 1/4/9 rays, with AA and grid transforms");
				exit.write(AppExit::Success);
			} else {
				settings.direct_rays = [DirectRayCount::One, DirectRayCount::Four, DirectRayCount::Nine][(batch as usize + 1) % 3];
				settings.anti_aliasing = false;
				if batch == 2 {
					*rooms.single_mut().unwrap() = Transform::from_xyz(0.75, 0.0, 0.0)
						.with_rotation(Quat::from_euler(EulerRot::YXZ, 0.25, 0.15, -0.1)).with_scale(Vec3::splat(1.25));
				}
			}
		},
		_ => {},
	}
	smoke.phase += 1;
	smoke.ticks = 0;
}

fn advance(mut commands: Commands, mut smoke: ResMut<Smoke>, stats: Res<RenderStats>, mut settings: ResMut<GraphicsSettings>, mut images: ResMut<Assets<Image>>, mut rooms: Query<&mut Transform, With<TestRoom>>, mut exit: MessageWriter<AppExit>) {
	assert!(smoke.started.elapsed().as_secs() < 180, "GI smoke test timed out");
	let gi = stats.inner.lock().unwrap().face_gi;
	smoke.ticks += 1;
	if smoke.phase == 0 && (gi.visible_faces == 0 || gi.total_faces <= gi.visible_faces || gi.incomplete_rays != 0) {
		smoke.ticks = 0;
		return;
	}
	let delay = if matches!(smoke.phase, 0 | 21 | 23) { 60 } else { 20 };
	if smoke.ticks < delay { return; }
	assert_mode(&stats, &settings);
	println!("GI_SMOKE phase {} after {:.2}s: {gi:?}", smoke.phase, smoke.started.elapsed().as_secs_f32());
	match smoke.phase {
		0 => capture(&mut commands, &smoke, "gi-on", None),
		1 => capture(&mut commands, &smoke, "gi-repeat", Some("gi-on")),
		2 => settings.lighting = LightingMode::Direct,
		3 => capture(&mut commands, &smoke, "direct", None),
		4 => settings.lighting = LightingMode::None,
		5 => capture(&mut commands, &smoke, "normal-shading", None),
		6 => { settings.lighting = LightingMode::Indirect; settings.gi_rays = GiRayCount::Eight; },
		7 => capture(&mut commands, &smoke, "gi-8", None),
		8 => settings.gi_rays = GiRayCount::Sixteen,
		9 => capture(&mut commands, &smoke, "gi-16", None),
		10 => settings.gi_rays = GiRayCount::SixtyFour,
		11 => capture(&mut commands, &smoke, "gi-64", None),
		12 => settings.gi_rays = GiRayCount::ThirtyTwo,
		13 => capture(&mut commands, &smoke, "gi-32-restored", Some("gi-on")),
		14 => settings.anti_aliasing = true,
		15 => capture(&mut commands, &smoke, "gi-aa", None),
		16 => images.get_mut(&smoke.target).unwrap().resize(wgpu::Extent3d { width: 640, height: 480, depth_or_array_layers: 1 }),
		17 => capture(&mut commands, &smoke, "gi-resized", None),
		18 => {
			*rooms.single_mut().unwrap() = Transform::from_xyz(0.75, 0.0, 0.0)
				.with_rotation(Quat::from_rotation_y(0.25)).with_scale(Vec3::splat(1.25));
		},
		19 => capture(&mut commands, &smoke, "gi-moved", None),
		20 => *rooms.single_mut().unwrap() = Transform::IDENTITY,
		21 => capture(&mut commands, &smoke, "gi-unmoved", Some("gi-resized")),
		22 => {
			settings.anti_aliasing = false;
			images.get_mut(&smoke.target).unwrap().resize(wgpu::Extent3d { width: 800, height: 600, depth_or_array_layers: 1 });
		},
		23 => capture(&mut commands, &smoke, "gi-restored", Some("gi-on")),
		24 => { settings.lighting = LightingMode::Direct; settings.direct_rays = DirectRayCount::Four; },
		25 => capture(&mut commands, &smoke, "direct-4", None),
		26 => capture(&mut commands, &smoke, "direct-4-repeat", Some("direct-4")),
		27 => settings.direct_rays = DirectRayCount::Nine,
		28 => capture(&mut commands, &smoke, "direct-9", None),
		29 => capture(&mut commands, &smoke, "direct-9-repeat", Some("direct-9")),
		30 => settings.lighting = LightingMode::None,
		31 => capture(&mut commands, &smoke, "normal-shading-rays9", Some("normal-shading")),
		32 => settings.lighting = LightingMode::Indirect,
		33 => capture(&mut commands, &smoke, "gi-direct-9", None),
		34 => capture(&mut commands, &smoke, "gi-direct-9-repeat", Some("gi-direct-9")),
		35 => settings.anti_aliasing = true,
		36 => capture(&mut commands, &smoke, "gi-direct-9-aa", None),
		37 => { settings.anti_aliasing = false; settings.direct_rays = DirectRayCount::Four; },
		38 => capture(&mut commands, &smoke, "gi-direct-4", None),
		39 => settings.direct_rays = DirectRayCount::One,
		40 => capture(&mut commands, &smoke, "gi-direct-1-restored", Some("gi-on")),
		41 => settings.lighting = LightingMode::Direct,
		42 => capture(&mut commands, &smoke, "direct-1-restored", Some("direct")),
		43 => {
			assert_eq!(smoke.captures, 24);
			println!("GI_SMOKE PASS: lighting modes, all GI/direct ray counts, partial shadows, color bleed, 2D dispatches, AA, resize, transforms, and frame-local output");
			exit.write(AppExit::Success);
		},
		_ => {},
	}
	smoke.phase += 1;
	smoke.ticks = 0;
}
