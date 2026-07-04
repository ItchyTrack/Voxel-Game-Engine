use bevy::prelude::*;

use super::config::TERRAIN_HEIGHT;

#[derive(Clone, Copy)]
pub(super) struct TerrainSample {
	pub(super) height: f32,
	pub(super) shade: f32,
}

pub(super) fn terrain_sample(unit: Vec3) -> TerrainSample {
	let height = terrain_height(unit);
	TerrainSample {
		height,
		shade: 1.0,
	}
}

fn terrain_height(unit: Vec3) -> f32 {
	0.0
}

pub(super) fn terrain_color(column: TerrainSample, altitude: f32) -> [u8; 4] {
	[
		200,
		100,
		30,
		255
	]
}

fn hsv_to_rgb(h: f32, s: f32, v: f32) -> Vec3 {
	let h = h * 6.0;
	let i = h.floor();
	let f = h - i;
	let p = v * (1.0 - s);
	let q = v * (1.0 - s * f);
	let t = v * (1.0 - s * (1.0 - f));
	let (r, g, b) = match i as i32 % 6 {
		0 => (v, t, p),
		1 => (q, v, p),
		2 => (p, v, t),
		3 => (p, q, v),
		4 => (t, p, v),
		_ => (v, p, q),
	};
	Vec3::new(r * 255.0, g * 255.0, b * 255.0)
}

fn fract(value: f32) -> f32 {
	value - value.floor()
}
