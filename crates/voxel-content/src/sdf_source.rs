use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Mutex, OnceLock, RwLock};

use bevy::math::{I16Vec3, IVec3, Vec3};

use voxel_data::grid::GridId;
use voxel_data::sdf::Sdf;
use voxel_data::voxels::{Voxel, Voxels};
use voxel_sources::{ChunkSource, SourceHandle};

/// Procedural SDF contract used by [`SdfSource`].
///
/// Coordinates are grid-local voxel coordinates. Negative distances are solid,
/// positive distances are empty, and zero is treated as solid.
pub trait VoxelSdf: Sdf + Send + Sync + 'static {
	fn voxel(&self, pos: Vec3) -> Voxel;

	fn bounds(&self) -> Option<(Vec3, Vec3)> {
		None
	}
}

#[derive(Clone, Copy, Debug)]
pub struct SdfSourceOptions {
	pub cost: u32,
	pub chunk_size: i32,
	pub cache_capacity: usize,
	pub sample_radius_scale: f32,
	pub empty_pruning: bool,
}

impl Default for SdfSourceOptions {
	fn default() -> Self {
		Self { cost: 10, chunk_size: 64, cache_capacity: 512, sample_radius_scale: 0.0, empty_pruning: false }
	}
}

#[derive(Clone)]
pub struct SdfSource {
	inner: Arc<SdfSourceInner>,
}

struct SdfSourceInner {
	handle: OnceLock<SourceHandle>,
	bindings: RwLock<HashMap<GridId, GridBinding>>,
	cache: Mutex<SdfCache>,
}

#[derive(Clone)]
struct GridBinding {
	sdf: Arc<dyn VoxelSdf>,
	options: SdfSourceOptions,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct SdfCacheKey {
	grid: GridId,
	min: IVec3,
	size: IVec3,
	lod: u8,
}

#[derive(Default)]
struct SdfCache {
	clock: u64,
	entries: HashMap<SdfCacheKey, (Option<Voxels>, u64)>,
	order: VecDeque<(SdfCacheKey, u64)>,
}

impl SdfCache {
	fn get(&mut self, key: SdfCacheKey) -> Option<Option<Voxels>> {
		let value = self.entries.get(&key).map(|(value, _)| value.clone())?;
		self.touch(key, value.clone());
		Some(value)
	}

	fn insert(&mut self, key: SdfCacheKey, value: Option<Voxels>, capacity: usize) {
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

	fn touch(&mut self, key: SdfCacheKey, value: Option<Voxels>) {
		self.clock = self.clock.wrapping_add(1);
		let stamp = self.clock;
		self.entries.insert(key, (value, stamp));
		self.order.push_back((key, stamp));
	}

	fn clear_grid(&mut self, grid: GridId) {
		self.entries.retain(|key, _| key.grid != grid);
		self.order.retain(|(key, _)| key.grid != grid);
	}
}

impl SdfSource {
	pub fn new() -> Self {
		Self {
			inner: Arc::new(SdfSourceInner {
				handle: OnceLock::new(),
				bindings: RwLock::new(HashMap::new()),
				cache: Mutex::new(SdfCache::default()),
			}),
		}
	}

	pub fn set_grid_sdf<S: VoxelSdf>(&self, grid: GridId, sdf: S) {
		self.set_grid_sdf_with_options(grid, sdf, SdfSourceOptions::default());
	}

	pub fn set_grid_sdf_with_options<S: VoxelSdf>(&self, grid: GridId, sdf: S, options: SdfSourceOptions) {
		self.inner.bindings.write().unwrap().insert(grid, GridBinding {
			sdf: Arc::new(sdf),
			options,
		});
		self.inner.cache.lock().unwrap().clear_grid(grid);
	}

	fn binding(&self, grid: GridId) -> Option<GridBinding> {
		self.inner.bindings.read().unwrap().get(&grid).cloned()
	}

	fn chunk_origin(options: SdfSourceOptions, chunk: IVec3) -> IVec3 {
		chunk * options.chunk_size
	}

	fn chunk_of(options: SdfSourceOptions, voxel: IVec3) -> IVec3 {
		voxel.div_euclid(IVec3::splat(options.chunk_size))
	}

	fn region_bounds(options: SdfSourceOptions, min: IVec3, size: IVec3) -> (Vec3, Vec3) {
		let lo = Self::chunk_origin(options, min).as_vec3();
		let hi = Self::chunk_origin(options, min + size).as_vec3();
		(lo, hi)
	}

	fn might_intersect_region(binding: &GridBinding, min: IVec3, size: IVec3) -> bool {
		let Some((bounds_min, bounds_max)) = binding.sdf.bounds() else { return true };
		if !bounds_min.cmplt(bounds_max).all() {
			return false;
		}
		let (region_min, region_max) = Self::region_bounds(binding.options, min, size);
		region_min.cmplt(bounds_max).all() && region_max.cmpgt(bounds_min).all()
	}

	fn cached_or_build(&self, grid: GridId, binding: &GridBinding, min: IVec3, size: IVec3, lod: f32) -> Option<Voxels> {
		let lod_u8 = lod.max(0.0).floor().clamp(0.0, u8::MAX as f32) as u8;
		let key = SdfCacheKey { grid, min, size, lod: lod_u8 };
		if let Some(value) = self.inner.cache.lock().unwrap().get(key) {
			return value;
		}

		let value = self.build_region(binding, min, size, lod);
		self.inner.cache.lock().unwrap().insert(key, value.clone(), binding.options.cache_capacity);
		value
	}

	fn build_region(&self, binding: &GridBinding, min: IVec3, size: IVec3, lod: f32) -> Option<Voxels> {
		if size.cmple(IVec3::ZERO).any() || binding.options.chunk_size <= 0 || !Self::might_intersect_region(binding, min, size) {
			return None;
		}
		let step = step_for_lod(lod)?;
		let full_extent = size * binding.options.chunk_size;
		let extent = full_extent / step;
		if extent.cmple(IVec3::ZERO).any() || extent.cmpgt(IVec3::splat(i16::MAX as i32)).any() {
			return None;
		}

		let base_origin = Self::chunk_origin(binding.options, min);
		let mut solid = Vec::new();
		let sample_radius = sample_radius(binding.options, step);
		if binding.options.empty_pruning {
			sample_region_recurse(binding, base_origin, step, IVec3::ZERO, extent, sample_radius, &mut solid);
		} else {
			sample_region_linear(binding, base_origin, step, extent, sample_radius, &mut solid);
		}

		if solid.is_empty() {
			None
		} else {
			let mut voxels = Voxels::new();
			voxels.add_voxels(&solid);
			Some(voxels)
		}
	}
}

impl Default for SdfSource {
	fn default() -> Self {
		Self::new()
	}
}

impl ChunkSource for SdfSource {
	fn init(&self, handle: SourceHandle) {
		let _ = self.inner.handle.set(handle);
	}

	fn cost(&self, grid: GridId, chunk: IVec3) -> Option<u32> {
		let binding = self.binding(grid)?;
		Self::might_intersect_region(&binding, chunk, IVec3::ONE).then_some(binding.options.cost)
	}

	fn request_load(&self, grid: GridId, chunk: IVec3, generation: u64) {
		let voxels = self.binding(grid).and_then(|binding| self.cached_or_build(grid, &binding, chunk, IVec3::ONE, 0.0));
		if let Some(handle) = self.inner.handle.get() {
			handle.loaded(grid, chunk, generation, voxels);
		}
	}

	fn cost_lod(&self, grid: GridId, min: IVec3, size: IVec3, _lod: f32) -> Option<u32> {
		let binding = self.binding(grid)?;
		Self::might_intersect_region(&binding, min, size).then_some(binding.options.cost)
	}

	fn request_load_lod(&self, grid: GridId, min: IVec3, size: IVec3, lod: f32, generation: u64) {
		let voxels = self.binding(grid).and_then(|binding| self.cached_or_build(grid, &binding, min, size, lod));
		if let Some(handle) = self.inner.handle.get() {
			handle.loaded_lod(grid, min, size, lod, generation, voxels);
		}
	}

	fn request_available_area(&self, grid: GridId) {
		let Some(handle) = self.inner.handle.get() else { return };
		if let Some(binding) = self.binding(grid)
			&& let Some((bounds_min, bounds_max)) = binding.sdf.bounds()
			&& bounds_min.cmplt(bounds_max).all() {
			let voxel_min = bounds_min.floor().as_ivec3();
			let voxel_max = bounds_max.ceil().as_ivec3() - IVec3::ONE;
			let chunk_min = Self::chunk_of(binding.options, voxel_min);
			let chunk_max = Self::chunk_of(binding.options, voxel_max);
			handle.claim(grid, chunk_min, chunk_max - chunk_min + IVec3::ONE);
		}
		handle.presence_loaded(grid);
	}
}

pub fn sdf_source() -> SdfSource {
	SdfSource::new()
}

fn sample_radius(options: SdfSourceOptions, step: i32) -> f32 {
	let scale = options.sample_radius_scale.max(0.0);
	if scale == 0.0 {
		0.0
	} else {
		Vec3::splat(step as f32).length() * 0.5 * scale
	}
}

fn sample_region_linear(binding: &GridBinding, base_origin: IVec3, step: i32, extent: IVec3, sample_radius: f32, solid: &mut Vec<(I16Vec3, Voxel)>) {
	let sample_offset = Vec3::splat(step as f32 * 0.5);
	for z in 0..extent.z {
		for y in 0..extent.y {
			for x in 0..extent.x {
				sample_cell(binding, base_origin, step, IVec3::new(x, y, z), sample_offset, sample_radius, solid);
			}
		}
	}
}

fn sample_region_recurse(
	binding: &GridBinding,
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
	if binding.sdf.sample(block_center) > block_radius + sample_radius {
		return;
	}

	if extent == IVec3::ONE {
		sample_cell(binding, base_origin, step, local_min, Vec3::splat(step as f32 * 0.5), sample_radius, solid);
		return;
	}

	let axis = longest_axis(extent);
	let split = (extent[axis] / 2).max(1);
	let mut first_extent = extent;
	first_extent[axis] = split;
	sample_region_recurse(binding, base_origin, step, local_min, first_extent, sample_radius, solid);

	let second_len = extent[axis] - split;
	if second_len > 0 {
		let mut second_min = local_min;
		second_min[axis] += split;
		let mut second_extent = extent;
		second_extent[axis] = second_len;
		sample_region_recurse(binding, base_origin, step, second_min, second_extent, sample_radius, solid);
	}
}

fn sample_cell(
	binding: &GridBinding,
	base_origin: IVec3,
	step: i32,
	local: IVec3,
	sample_offset: Vec3,
	sample_radius: f32,
	solid: &mut Vec<(I16Vec3, Voxel)>,
) {
	let sample = (base_origin + local * step).as_vec3() + sample_offset;
	if binding.sdf.sample(sample) <= sample_radius {
		solid.push((I16Vec3::new(local.x as i16, local.y as i16, local.z as i16), binding.sdf.voxel(sample)));
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
