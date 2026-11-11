//! Camera flights. World layout: church at y=-350 around the origin, ray-rendered planet of
//! radius 512 centred at (0, 0, -2000) with the tree on top, marching-cubes Sponza at x=1500.
//! Benchmark-only extras: the castle at x=-6000 and a ray-traced Sponza at x=6000.

use bevy::math::Vec3;

#[derive(Clone, Copy)]
pub struct Key {
	pub pos: Vec3,
	pub look: Vec3,
}

pub const fn key(pos: [f32; 3], look: [f32; 3]) -> Key {
	Key {
		pos: Vec3::new(pos[0], pos[1], pos[2]),
		look: Vec3::new(look[0], look[1], look[2]),
	}
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ShotMode {
	/// Streaming frozen after warm-up and a fixed step per frame: every run renders the same
	/// frames, so the pure ray tracing cost is directly comparable.
	Frozen,
	/// Streaming and physics live, camera driven by wall clock, like a player flying around.
	Live,
}

/// Grids the default scene doesn't contain, spawned only for the scenarios that need them.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ExtraScene {
	None,
	Castle,
	RaySponza,
	MovingObjects,
}

pub const CASTLE_ORIGIN: Vec3 = Vec3::new(-6000.0, -250.0, 0.0);
pub const RAY_SPONZA_ORIGIN: Vec3 = Vec3::new(6000.0, 0.0, 0.0);

#[derive(Clone, Copy)]
pub struct Shot {
	pub name: &'static str,
	pub mode: ShotMode,
	pub seconds: f32,
	pub keys: &'static [Key],
}

#[derive(Clone, Copy)]
pub struct Scenario {
	pub name: &'static str,
	pub extra: ExtraScene,
	pub shots: &'static [Shot],
}

const fn frozen(name: &'static str, seconds: f32, keys: &'static [Key]) -> Shot {
	Shot { name, mode: ShotMode::Frozen, seconds, keys }
}

pub const PLANET_CENTER: [f32; 3] = [0.0, 0.0, -2000.0];

pub const SCENARIOS: &[Scenario] = &[
	Scenario {
		name: "church",
		extra: ExtraScene::None,
		shots: &[
			// High pass over the whole village: lots of distant LOD tiles in view.
			frozen("church_flyover", 8.0, &[
				key([-900.0, 260.0, 900.0], [0.0, -330.0, 0.0]),
				key([0.0, 300.0, 600.0], [0.0, -330.0, -200.0]),
				key([900.0, 260.0, 0.0], [-200.0, -330.0, -300.0]),
			]),
			// Low between the houses: dense near geometry, long grazing rays along the streets.
			frozen("church_street", 8.0, &[
				key([0.0, 10.0, 450.0], [0.0, -30.0, 0.0]),
				key([-30.0, 0.0, 200.0], [-200.0, -40.0, 0.0]),
				key([-200.0, 10.0, 0.0], [-400.0, -30.0, -300.0]),
			]),
		],
	},
	Scenario {
		name: "planet",
		extra: ExtraScene::None,
		shots: &[
			// Half orbit around the whole planet.
			frozen("planet_orbit", 8.0, &[
				key([1700.0, 300.0, -2000.0], PLANET_CENTER),
				key([0.0, 500.0, -300.0], PLANET_CENTER),
				key([-1700.0, 300.0, -2000.0], PLANET_CENTER),
			]),
			// Circling the tree on top of the planet: fine foliage, many small cells.
			frozen("planet_tree", 8.0, &[
				key([160.0, 700.0, -2000.0], [0.0, 690.0, -2000.0]),
				key([0.0, 720.0, -1840.0], [0.0, 690.0, -2000.0]),
				key([-160.0, 700.0, -2000.0], [0.0, 690.0, -2000.0]),
			]),
		],
	},
	Scenario {
		name: "castle",
		extra: ExtraScene::Castle,
		shots: &[
			frozen("castle_flyaround", 8.0, &[
				key([-6000.0 - 1100.0, 350.0, 0.0], [-6000.0, 0.0, 0.0]),
				key([-6000.0, 450.0, 1100.0], [-6000.0, 0.0, 0.0]),
				key([-6000.0 + 1100.0, 350.0, 0.0], [-6000.0, 0.0, 0.0]),
			]),
		],
	},
	Scenario {
		name: "sponza",
		extra: ExtraScene::RaySponza,
		shots: &[
			// Along the atrium of a ray-traced Sponza copy: enclosed space, every ray hits close.
			frozen("sponza_atrium", 8.0, &[
				key([6000.0 - 250.0, -60.0, 0.0], [6000.0, -80.0, 0.0]),
				key([6000.0, -40.0, 30.0], [6000.0 + 250.0, -80.0, 0.0]),
				key([6000.0 + 180.0, -60.0, 0.0], [6000.0 + 400.0, -70.0, 0.0]),
			]),
		],
	},
	Scenario {
		name: "dynamic",
		extra: ExtraScene::MovingObjects,
		shots: &[
			// ~100 grids orbiting and spinning over the village; scripted, so frames are identical
			// between runs. Every one of them changes the BVH every frame.
			frozen("swarm", 10.0, &[
				key([450.0, 240.0, 0.0], [0.0, 100.0, 0.0]),
				key([0.0, 300.0, 450.0], [0.0, 100.0, 0.0]),
				key([-450.0, 240.0, 0.0], [0.0, 100.0, 0.0]),
			]),
			// Physics balls falling onto the roofs with the solver live, at real-time speed.
			Shot {
				name: "physics_drop",
				mode: ShotMode::Live,
				seconds: 12.0,
				keys: &[
					key([700.0, 160.0, 0.0], [450.0, 40.0, -250.0]),
					key([650.0, 120.0, -50.0], [450.0, 0.0, -250.0]),
				],
			},
		],
	},
	Scenario {
		name: "tour",
		extra: ExtraScene::None,
		shots: &[Shot {
			// Fast real-time tour with live streaming and physics: stutter, load, and memory.
			name: "streaming_tour",
			mode: ShotMode::Live,
			seconds: 25.0,
			keys: &[
				key([0.0, 0.0, 600.0], [0.0, -200.0, 0.0]),
				key([600.0, -150.0, 0.0], [0.0, -250.0, -300.0]),
				key([300.0, 200.0, -900.0], [0.0, 400.0, -2000.0]),
				key([200.0, 760.0, -1800.0], [0.0, 690.0, -2000.0]),
				key([-700.0, 300.0, -2400.0], PLANET_CENTER),
				key([800.0, 0.0, -700.0], [1500.0, 0.0, 0.0]),
				key([1400.0, 0.0, 200.0], [1500.0, -50.0, 0.0]),
			],
		}],
	},
];

pub fn find(name: &str) -> Option<&'static Scenario> {
	SCENARIOS.iter().find(|scenario| scenario.name == name)
}

/// Catmull-Rom through the keys for `t` in 0..=1.
pub fn sample(keys: &[Key], t: f32) -> (Vec3, Vec3) {
	if keys.len() == 1 { return (keys[0].pos, keys[0].look); }
	let segments = keys.len() - 1;
	let scaled = t.clamp(0.0, 1.0) * segments as f32;
	let index = (scaled as usize).min(segments - 1);
	let local = scaled - index as f32;
	let at = |offset: isize| keys[(index as isize + offset).clamp(0, segments as isize) as usize];
	let spline = |p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3| {
		0.5 * ((2.0 * p1)
			+ (p2 - p0) * local
			+ (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * local * local
			+ (3.0 * p1 - p0 - 3.0 * p2 + p3) * local * local * local)
	};
	(
		spline(at(-1).pos, at(0).pos, at(1).pos, at(2).pos),
		spline(at(-1).look, at(0).look, at(1).look, at(2).look),
	)
}
