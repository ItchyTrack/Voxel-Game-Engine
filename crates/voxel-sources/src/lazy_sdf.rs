use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Mutex, OnceLock};

use bevy::math::{I16Vec3, IVec3, Vec3};

use voxel_data::grid::GridId;
use voxel_data::voxels::{Voxel, Voxels};

use crate::{ChunkSource, SourceHandle};

/// Procedural SDF contract used by [`LazySdfSource`].
///
/// Coordinates are grid-local voxel coordinates. Negative distances are solid,
/// positive distances are empty, and zero is treated as solid.
pub trait LazyVoxelSdf: Send + Sync + 'static {
	fn sample(&self, pos: Vec3) -> f32;

	/// Material for a solid voxel sampled at `pos`.
	fn voxel(&self, pos: Vec3) -> Voxel;

	/// Optional finite grid-local bounds of the SDF in base-voxel coordinates.
	/// If present, the source uses it for source presence and cheap rejection.
	fn bounds(&self) -> Option<(Vec3, Vec3)> {
		None
	}
}

#[derive(Clone, Copy, Debug)]
pub struct LazySdfSourceOptions {
	/// Cost reported to the source router when the requested chunk/tile overlaps
	/// this SDF. Lower costs win over other sources.
	pub cost: u32,
	/// Number of base voxels per streaming chunk.
	pub chunk_size: i32,
	/// Maximum number of chunk/LOD results kept in the in-memory cache.
	pub cache_capacity: usize,
	/// Multiplier for the sampled cell's half-diagonal. `0.0` is pure center
	/// sampling (`sdf(center) <= 0`). `1.0` conservatively includes cells whose
	/// center is outside but close enough that the zero surface may cross them.
	pub sample_radius_scale: f32,
	/// Recursively skip regions whose center distance proves the whole block is
	/// empty. Best for expensive SDFs with reasonably conservative positive
	/// distance estimates.
	pub empty_pruning: bool,
}

impl Default for LazySdfSourceOptions {
	fn default() -> Self {
		Self { cost: 10, chunk_size: 64, cache_capacity: 512, sample_radius_scale: 0.0, empty_pruning: false }
	}
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct LazySdfCacheKey {
	grid: GridId,
	min: IVec3,
	size: IVec3,
	lod: u8,
}

#[derive(Default)]
struct LazySdfCache {
	clock: u64,
	entries: HashMap<LazySdfCacheKey, (Option<Voxels>, u64)>,
	order: VecDeque<(LazySdfCacheKey, u64)>,
}

impl LazySdfCache {
	fn get(&mut self, key: LazySdfCacheKey) -> Option<Option<Voxels>> {
		let value = self.entries.get(&key).map(|(value, _)| value.clone())?;
		self.touch(key, value.clone());
		Some(value)
	}

	fn insert(&mut self, key: LazySdfCacheKey, value: Option<Voxels>, capacity: usize) {
		if capacity == 0 {
			return;
		}
		self.touch(key, value);
		while self.entries.len() > capacity {
			let Some((old_key, old_stamp)) = self.order.pop_front() else { break };
			if self.entries.get(&old_key).is_some_and(|(_, stamp)| *stamp == old_stamp) {
				self.entries.remove(&old_key);
			}
		}
	}

	fn touch(&mut self, key: LazySdfCacheKey, value: Option<Voxels>) {
		self.clock = self.clock.wrapping_add(1);
		let stamp = self.clock;
		self.entries.insert(key, (value, stamp));
		self.order.push_back((key, stamp));
	}
}

/// A lazy, on-demand procedural SDF source.
///
/// The source serves normal chunk requests by sampling the SDF in base voxel
/// cells. It serves LOD requests directly at coarse resolution, so a high LOD
/// does not require generating full-resolution chunks first.
///
/// `grid` is a shared cell that should be set to the spawned grid entity, mirroring
/// the existing source pattern used by the app sources.
pub struct LazySdfSource<S: LazyVoxelSdf> {
	sdf: Arc<S>,
	grid: Arc<OnceLock<GridId>>,
	handle: OnceLock<SourceHandle>,
	options: LazySdfSourceOptions,
	cache: Mutex<LazySdfCache>,
}

impl<S: LazyVoxelSdf> LazySdfSource<S> {
	pub fn new(grid: Arc<OnceLock<GridId>>, sdf: S) -> Self {
		Self::with_options(grid, sdf, LazySdfSourceOptions::default())
	}

	pub fn with_options(grid: Arc<OnceLock<GridId>>, sdf: S, options: LazySdfSourceOptions) -> Self {
		Self { sdf: Arc::new(sdf), grid, handle: OnceLock::new(), options, cache: Mutex::new(LazySdfCache::default()) }
	}

	pub fn grid_cell(&self) -> Arc<OnceLock<GridId>> {
		self.grid.clone()
	}

	fn is_mine(&self, grid: GridId) -> bool {
		self.grid.get() == Some(&grid)
	}

	fn chunk_origin(&self, chunk: IVec3) -> IVec3 {
		chunk * self.options.chunk_size
	}

	fn chunk_of(&self, voxel: IVec3) -> IVec3 {
		voxel.div_euclid(IVec3::splat(self.options.chunk_size))
	}

	fn region_bounds(&self, min: IVec3, size: IVec3) -> (Vec3, Vec3) {
		let lo = self.chunk_origin(min).as_vec3();
		let hi = self.chunk_origin(min + size).as_vec3();
		(lo, hi)
	}

	fn might_intersect_region(&self, min: IVec3, size: IVec3) -> bool {
		let Some((bounds_min, bounds_max)) = self.sdf.bounds() else { return true };
		if !bounds_min.cmplt(bounds_max).all() {
			return false;
		}
		let (region_min, region_max) = self.region_bounds(min, size);
		region_min.cmplt(bounds_max).all() && region_max.cmpgt(bounds_min).all()
	}

	fn cached_or_build(&self, grid: GridId, min: IVec3, size: IVec3, lod: f32) -> Option<Voxels> {
		let lod_u8 = lod.max(0.0).floor().clamp(0.0, u8::MAX as f32) as u8;
		let key = LazySdfCacheKey { grid, min, size, lod: lod_u8 };
		if let Some(value) = self.cache.lock().unwrap().get(key) {
			return value;
		}

		let value = self.build_region(min, size, lod);
		self.cache.lock().unwrap().insert(key, value.clone(), self.options.cache_capacity);
		value
	}

	fn build_region(&self, min: IVec3, size: IVec3, lod: f32) -> Option<Voxels> {
		if size.cmple(IVec3::ZERO).any() || self.options.chunk_size <= 0 || !self.might_intersect_region(min, size) {
			return None;
		}
		let step = step_for_lod(lod)?;
		let full_extent = size * self.options.chunk_size;
		let extent = full_extent / step;
		if extent.cmple(IVec3::ZERO).any() || extent.cmpgt(IVec3::splat(i16::MAX as i32)).any() {
			return None;
		}

		let base_origin = self.chunk_origin(min);
		let mut solid = Vec::new();
		let sample_radius = self.sample_radius(step);
		if self.options.empty_pruning {
			self.sample_region_recurse(base_origin, step, IVec3::ZERO, extent, sample_radius, &mut solid);
		} else {
			self.sample_region_linear(base_origin, step, extent, sample_radius, &mut solid);
		}

		if solid.is_empty() {
			None
		} else {
			let mut voxels = Voxels::new();
			voxels.add_voxels(&solid);
			Some(voxels)
		}
	}

	fn sample_radius(&self, step: i32) -> f32 {
		let scale = self.options.sample_radius_scale.max(0.0);
		if scale == 0.0 {
			0.0
		} else {
			Vec3::splat(step as f32).length() * 0.5 * scale
		}
	}

	fn sample_region_linear(&self, base_origin: IVec3, step: i32, extent: IVec3, sample_radius: f32, solid: &mut Vec<(I16Vec3, Voxel)>) {
		let sample_offset = Vec3::splat(step as f32 * 0.5);
		for z in 0..extent.z {
			for y in 0..extent.y {
				for x in 0..extent.x {
					self.sample_cell(base_origin, step, IVec3::new(x, y, z), sample_offset, sample_radius, solid);
				}
			}
		}
	}

	fn sample_region_recurse(
		&self,
		base_origin: IVec3,
		step: i32,
		local_min: IVec3,
		extent: IVec3,
		sample_radius: f32,
		solid: &mut Vec<(I16Vec3, Voxel)>,
	) {
		let block_size = (extent * step).as_vec3();
		let block_center = (base_origin + local_min * step).as_vec3() + block_size * 0.5;
		let block_radius = block_size.length() * 0.5;
		if self.sdf.sample(block_center) > block_radius + sample_radius {
			return;
		}

		if extent == IVec3::ONE {
			self.sample_cell(base_origin, step, local_min, Vec3::splat(step as f32 * 0.5), sample_radius, solid);
			return;
		}

		let axis = longest_axis(extent);
		let split = (extent[axis] / 2).max(1);
		let mut first_extent = extent;
		first_extent[axis] = split;
		self.sample_region_recurse(base_origin, step, local_min, first_extent, sample_radius, solid);

		let second_len = extent[axis] - split;
		if second_len > 0 {
			let mut second_min = local_min;
			second_min[axis] += split;
			let mut second_extent = extent;
			second_extent[axis] = second_len;
			self.sample_region_recurse(base_origin, step, second_min, second_extent, sample_radius, solid);
		}
	}

	fn sample_cell(
		&self,
		base_origin: IVec3,
		step: i32,
		local: IVec3,
		sample_offset: Vec3,
		sample_radius: f32,
		solid: &mut Vec<(I16Vec3, Voxel)>,
	) {
		let sample = (base_origin + local * step).as_vec3() + sample_offset;
		if self.sdf.sample(sample) <= sample_radius {
			solid.push((I16Vec3::new(local.x as i16, local.y as i16, local.z as i16), self.sdf.voxel(sample)));
		}
	}
}

impl<S: LazyVoxelSdf> ChunkSource for LazySdfSource<S> {
	fn init(&self, handle: SourceHandle) {
		let _ = self.handle.set(handle);
	}

	fn cost(&self, grid: GridId, chunk: IVec3) -> Option<u32> {
		(self.is_mine(grid) && self.might_intersect_region(chunk, IVec3::ONE)).then_some(self.options.cost)
	}

	fn request_load(&self, grid: GridId, chunk: IVec3, generation: u64) {
		let voxels = self.cached_or_build(grid, chunk, IVec3::ONE, 0.0);
		if let Some(handle) = self.handle.get() {
			handle.loaded(grid, chunk, generation, voxels);
		}
	}

	fn cost_lod(&self, grid: GridId, min: IVec3, size: IVec3, _lod: f32) -> Option<u32> {
		(self.is_mine(grid) && self.might_intersect_region(min, size)).then_some(self.options.cost)
	}

	fn request_load_lod(&self, grid: GridId, min: IVec3, size: IVec3, lod: f32, generation: u64) {
		let voxels = self.cached_or_build(grid, min, size, lod);
		if let Some(handle) = self.handle.get() {
			handle.loaded_lod(grid, min, size, lod, generation, voxels);
		}
	}

	fn request_available_area(&self, grid: GridId) {
		let Some(handle) = self.handle.get() else { return };
		if self.is_mine(grid) {
			if let Some((bounds_min, bounds_max)) = self.sdf.bounds() {
				if bounds_min.cmplt(bounds_max).all() {
					let voxel_min = bounds_min.floor().as_ivec3();
					let voxel_max = bounds_max.ceil().as_ivec3() - IVec3::ONE;
					let chunk_min = self.chunk_of(voxel_min);
					let chunk_max = self.chunk_of(voxel_max);
					handle.claim(grid, chunk_min, chunk_max - chunk_min + IVec3::ONE);
				}
			}
		}
		handle.presence_loaded(grid);
	}
}

fn step_for_lod(lod: f32) -> Option<i32> {
	if !lod.is_finite() {
		return None;
	}
	let exp = lod.max(0.0).floor() as u32;
	1i32.checked_shl(exp).filter(|step| *step > 0)
}

fn longest_axis(v: IVec3) -> usize {
	if v.x >= v.y && v.x >= v.z {
		0
	} else if v.y >= v.z {
		1
	} else {
		2
	}
}

#[cfg(test)]
mod tests {
	use super::*;
	use crossbeam_channel::unbounded;

	#[derive(Clone, Copy)]
	struct Sphere;

	impl LazyVoxelSdf for Sphere {
		fn sample(&self, pos: Vec3) -> f32 {
			(pos - Vec3::splat(8.0)).length() - 4.0
		}

		fn voxel(&self, _pos: Vec3) -> Voxel {
			Voxel { color: [255, 0, 0, 255], mass: 1 }
		}

		fn bounds(&self) -> Option<(Vec3, Vec3)> {
			Some((Vec3::splat(4.0), Vec3::splat(12.0)))
		}
	}

	#[test]
	fn builds_center_sampled_chunk() {
		let grid = Arc::new(OnceLock::new());
		let source = LazySdfSource::with_options(grid.clone(), Sphere, LazySdfSourceOptions { chunk_size: 16, cache_capacity: 8, cost: 1, ..Default::default() });
		let (tx, rx) = unbounded();
		let entity = GridId::PLACEHOLDER;
		let _ = grid.set(entity);
		source.init(SourceHandle { id: crate::SourceId(0), messages: tx });
		source.request_load(entity, IVec3::ZERO, 7);
		let crate::handle::SourceMessage::Chunk(result) = rx.recv().unwrap() else { panic!("expected chunk") };
		let voxels = result.voxels.unwrap();
		assert!(voxels.voxel(&I16Vec3::new(8, 8, 8)).is_some());
		assert!(voxels.voxel(&I16Vec3::ZERO).is_none());
	}
}
