use bevy::prelude::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct BandBounds {
	pub(crate) min: IVec3,
	pub(crate) max: IVec3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct LodBand {
	pub(crate) lod: u8,
	pub(crate) outer: BandBounds,
	pub(crate) inner: Option<BandBounds>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
struct Box3 {
	min: IVec3,
	max: IVec3,
}

impl Box3 {
	const EMPTY: Box3 = Box3 { min: IVec3::ZERO, max: IVec3::ZERO };

	fn is_empty(self) -> bool {
		self.min.cmpge(self.max).any()
	}

	fn intersect(self, other: Box3) -> Box3 {
		Box3 { min: self.min.max(other.min), max: self.max.min(other.max) }
	}
}

pub(crate) fn run_over_diff(old_bands: &[LodBand], new_bands: &[LodBand], mut f: impl FnMut(u8, IVec3, bool)) {
	let Some(max_lod) = old_bands.iter().chain(new_bands.iter()).map(|band| band.lod).max() else { return };

	for lod in 0..=max_lod {
		let (old_outer, old_inner) = band_boxes(old_bands, lod);
		let (new_outer, new_inner) = band_boxes(new_bands, lod);
		if new_outer == old_outer && new_inner == old_inner { continue; }
		let size = 1i32 << lod;
		emit_minus_minus(new_outer, old_outer, new_inner, lod, size, true, &mut f);
		emit_minus(new_outer.intersect(old_inner), new_inner, lod, size, true, &mut f);

		emit_minus_minus(old_outer, new_outer, old_inner, lod, size, false, &mut f);
		emit_minus(old_outer.intersect(new_inner), old_inner, lod, size, false, &mut f);
	}
}

fn band_boxes(bands: &[LodBand], lod: u8) -> (Box3, Box3) {
	match bands.iter().find(|band| band.lod == lod) {
		Some(band) => {
			let outer = Box3 { min: band.outer.min, max: band.outer.max };
			let inner = band.inner.map_or(Box3::EMPTY, |inner| Box3 { min: inner.min, max: inner.max });
			(outer, inner)
		}
		None => (Box3::EMPTY, Box3::EMPTY),
	}
}

fn emit_minus<F: FnMut(u8, IVec3, bool)>(region: Box3, cut: Box3, lod: u8, size: i32, added: bool, f: &mut F) {
	for slab in box_minus_box(region, cut) {
		emit_box(slab, lod, size, added, f);
	}
}

fn emit_minus_minus<F: FnMut(u8, IVec3, bool)>(region: Box3, cut_a: Box3, cut_b: Box3, lod: u8, size: i32, added: bool, f: &mut F) {
	for slab in box_minus_box(region, cut_a) {
		for piece in box_minus_box(slab, cut_b) {
			emit_box(piece, lod, size, added, f);
		}
	}
}

fn box_minus_box(a: Box3, b: Box3) -> [Box3; 6] {
	if a.is_empty() {
		return [Box3::EMPTY; 6];
	}
	let i = a.intersect(b);
	if i.is_empty() {
		return [a, Box3::EMPTY, Box3::EMPTY, Box3::EMPTY, Box3::EMPTY, Box3::EMPTY];
	}
	[
		Box3 { min: a.min, max: IVec3::new(i.min.x, a.max.y, a.max.z) },
		Box3 { min: IVec3::new(i.max.x, a.min.y, a.min.z), max: a.max },
		Box3 { min: IVec3::new(i.min.x, a.min.y, a.min.z), max: IVec3::new(i.max.x, i.min.y, a.max.z) },
		Box3 { min: IVec3::new(i.min.x, i.max.y, a.min.z), max: IVec3::new(i.max.x, a.max.y, a.max.z) },
		Box3 { min: IVec3::new(i.min.x, i.min.y, a.min.z), max: IVec3::new(i.max.x, i.max.y, i.min.z) },
		Box3 { min: IVec3::new(i.min.x, i.min.y, i.max.z), max: IVec3::new(i.max.x, i.max.y, a.max.z) },
	]
}

fn emit_box<F: FnMut(u8, IVec3, bool)>(b: Box3, lod: u8, size: i32, added: bool, f: &mut F) {
	if b.is_empty() {
		return;
	}
	let mut x = b.min.x;
	while x < b.max.x {
		let mut y = b.min.y;
		while y < b.max.y {
			let mut z = b.min.z;
			while z < b.max.z {
				f(lod, IVec3::new(x, y, z), added);
				z += size;
			}
			y += size;
		}
		x += size;
	}
}
