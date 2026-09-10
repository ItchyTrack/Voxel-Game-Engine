use voxel_math::{Fixed, FixedVec3};

use super::config::TERRAIN_HEIGHT;

#[derive(Clone, Copy)]
pub(super) struct TerrainSample {
	pub(super) height: Fixed,
	pub(super) shade: Fixed,
}

pub(super) fn terrain_sample(unit: FixedVec3) -> TerrainSample {
	let height = terrain_height(unit);
	TerrainSample {
		height,
		shade: Fixed::ONE,
	}
}

fn terrain_height(unit: FixedVec3) -> Fixed {
	Fixed::ZERO
}

pub(super) fn terrain_color(column: TerrainSample, altitude: Fixed) -> [u8; 4] {
	[
		200,
		100,
		30,
		255
	]
}

fn hsv_to_rgb(h: Fixed, s: Fixed, v: Fixed) -> FixedVec3 {
	let h = h * Fixed::from_num(6);
	let i = h.floor();
	let f = h - i;
	let p = v * (Fixed::ONE - s);
	let q = v * (Fixed::ONE - s * f);
	let t = v * (Fixed::ONE - s * (Fixed::ONE - f));
	let (r, g, b) = match i.to_num::<i32>() % 6 {
		0 => (v, t, p),
		1 => (q, v, p),
		2 => (p, v, t),
		3 => (p, q, v),
		4 => (t, p, v),
		_ => (v, p, q),
	};
	FixedVec3::new(r, g, b) * Fixed::from_num(255)
}

fn fract(value: Fixed) -> Fixed {
	value - value.floor()
}
