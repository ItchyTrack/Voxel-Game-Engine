use bevy::prelude::*;
use tile_data::{ChunkRegion, NonZeroChunkRegion};
use voxel_streaming::GridStreaming;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct LodBand {
	pub(crate) lod: u8,
	pub(crate) outer: ChunkRegion,
	pub(crate) inner: Option<ChunkRegion>,
}

/// Runs `f(lod, tile_min)` for every tile that lies within a band and overlaps the area
/// `[area_min, area_min + area_size)`.
pub(crate) fn for_each_tile_in_bands(bands: &[LodBand], area_region: NonZeroChunkRegion, mut f: impl FnMut(u8, IVec3)) {
	let area_min = area_region.min();
	let area_max = area_region.end();
	for band in bands {
		let tile_size = 1i32 << band.lod;
		let overlap_min = area_min.max(band.outer.min());
		let overlap_max = area_max.min(band.outer.end());
		if overlap_min.cmpge(overlap_max).any() {
			continue;
		}
		let mut x = overlap_min.x.div_euclid(tile_size) * tile_size;
		while x < overlap_max.x {
			let mut y = overlap_min.y.div_euclid(tile_size) * tile_size;
			while y < overlap_max.y {
				let mut z = overlap_min.z.div_euclid(tile_size) * tile_size;
				while z < overlap_max.z {
					let tile_min = IVec3::new(x, y, z);
					if is_tile_in_band(*band, tile_min) {
						f(band.lod, tile_min);
					}
					z += tile_size;
				}
				y += tile_size;
			}
			x += tile_size;
		}
	}
}

fn is_tile_in_band(band: LodBand, min: IVec3) -> bool {
	let tile_size = 1i32 << band.lod;
	let tile_max = min + IVec3::splat(tile_size);
	let in_outer = min.cmpge(band.outer.min()).all() && tile_max.cmple(band.outer.end()).all();
	let outside_inner = band.inner.is_none_or(|inner| !(min.cmpge(inner.min()).all() && tile_max.cmple(inner.end()).all()));
	in_outer && outside_inner
}

pub(crate) fn run_over_diff(
	old_bands: &[LodBand],
	new_bands: &[LodBand],
	streaming: &GridStreaming,
	mut f: impl FnMut(u8, IVec3, bool),
) {
	let Some(max_lod) = old_bands.iter().chain(new_bands.iter()).map(|band| band.lod).max() else { return };

	for lod in 0..=max_lod {
		let (old_outer, old_inner) = band_boxes(old_bands, lod);
		let (new_outer, new_inner) = band_boxes(new_bands, lod);
		if new_outer == old_outer && new_inner == old_inner { continue; }
		let size = 1i32 << lod;
		emit_minus_minus(new_outer, old_outer, new_inner, lod, size, true, streaming, &mut f);
		emit_minus(intersection(new_outer, old_inner), new_inner, lod, size, true, streaming, &mut f);

		emit_minus_minus(old_outer, new_outer, old_inner, lod, size, false, streaming, &mut f);
		emit_minus(intersection(old_outer, new_inner), old_inner, lod, size, false, streaming, &mut f);
	}
}

fn empty_region() -> ChunkRegion {
	ChunkRegion::new(IVec3::ZERO, UVec3::ZERO)
}

fn intersection(a: ChunkRegion, b: ChunkRegion) -> ChunkRegion {
	a.intersection(b).map(Into::into).unwrap_or_else(empty_region)
}

fn band_boxes(bands: &[LodBand], lod: u8) -> (ChunkRegion, ChunkRegion) {
	match bands.iter().find(|band| band.lod == lod) {
		Some(band) => (band.outer, band.inner.unwrap_or_else(empty_region)),
		None => (empty_region(), empty_region()),
	}
}

fn emit_minus<F: FnMut(u8, IVec3, bool)>(region: ChunkRegion, cut: ChunkRegion, lod: u8, size: i32, added: bool, streaming: &GridStreaming, f: &mut F) {
	for slab in region_minus_region(region, cut) {
		emit_region(slab, lod, size, added, streaming, f);
	}
}

fn emit_minus_minus<F: FnMut(u8, IVec3, bool)>(
	region: ChunkRegion,
	cut_a: ChunkRegion,
	cut_b: ChunkRegion,
	lod: u8,
	size: i32,
	added: bool,
	streaming: &GridStreaming,
	f: &mut F,
) {
	for slab in region_minus_region(region, cut_a) {
		for piece in region_minus_region(slab, cut_b) {
			emit_region(piece, lod, size, added, streaming, f);
		}
	}
}

fn region_minus_region(a: ChunkRegion, b: ChunkRegion) -> [ChunkRegion; 6] {
	if a.is_empty() {
		return [empty_region(); 6];
	}
	let Some(i) = a.intersection(b) else {
		return [a, empty_region(), empty_region(), empty_region(), empty_region(), empty_region()];
	};
	[
		ChunkRegion::from_min_end(a.min(), IVec3::new(i.min().x, a.end().y, a.end().z)).unwrap(),
		ChunkRegion::from_min_end(IVec3::new(i.end().x, a.min().y, a.min().z), a.end()).unwrap(),
		ChunkRegion::from_min_end(IVec3::new(i.min().x, a.min().y, a.min().z), IVec3::new(i.end().x, i.min().y, a.end().z)).unwrap(),
		ChunkRegion::from_min_end(IVec3::new(i.min().x, i.end().y, a.min().z), IVec3::new(i.end().x, a.end().y, a.end().z)).unwrap(),
		ChunkRegion::from_min_end(IVec3::new(i.min().x, i.min().y, a.min().z), IVec3::new(i.end().x, i.end().y, i.min().z)).unwrap(),
		ChunkRegion::from_min_end(IVec3::new(i.min().x, i.min().y, i.end().z), IVec3::new(i.end().x, i.end().y, a.end().z)).unwrap(),
	]
}

fn emit_region<F: FnMut(u8, IVec3, bool)>(region: ChunkRegion, lod: u8, size: i32, added: bool, streaming: &GridStreaming, f: &mut F) {
	if region.is_empty() {
		return;
	}
	streaming.presence().for_each_occupied_tile_cover(
		NonZeroChunkRegion::try_from(region).unwrap(),
		size,
		|tile_min| f(lod, tile_min, added),
	);
}
