use std::collections::HashMap;
use std::sync::{Arc, OnceLock, RwLock};

use bevy::ecs::resource::Resource;
use bevy::math::{IVec2, IVec3, Vec3};

use voxel_data::grid::GridId;
use voxel_data::sdf::Sdf;
use voxel_data::voxels::{VoxelRef, VoxelTypeId, Voxels};
use voxel_sources::{CancellationToken, ChunkSource, SourceCoverage, SourceHandle, TakeJob};
use tile_data::CHUNK_SIZE;
use voxel_streaming::ForgottenChunks;

/// Procedural SDF contract used by [`SdfSource`].
///
/// Coordinates are grid-local voxel coordinates. Negative distances are solid,
/// positive distances are empty, and zero is treated as solid.
pub trait VoxelSdf: Sdf + Send + Sync + 'static {
	fn voxel(&self) -> VoxelRef<'_>;

	fn lod_voxel(&self) -> VoxelRef<'_> {
		self.voxel()
	}

	fn bounds(&self) -> Option<(Vec3, Vec3)> {
		None
	}
}

#[derive(Clone, Copy, Debug)]
pub struct SdfSourceOptions {
	pub cost: u32,
	pub sample_radius_scale: f32,
}

impl Default for SdfSourceOptions {
	fn default() -> Self {
		Self { cost: 10, sample_radius_scale: 0.0 }
	}
}

#[derive(Resource, Clone)]
pub struct SdfSource {
	inner: Arc<SdfSourceInner>,
}

struct SdfSourceInner {
	handle: OnceLock<SourceHandle>,
	bindings: RwLock<HashMap<GridId, GridBinding>>,
	forgotten: ForgottenChunks,
}

#[derive(Clone)]
struct GridBinding {
	sdf: Arc<dyn VoxelSdf>,
	options: SdfSourceOptions,
}


impl SdfSource {
	pub fn new() -> Self {
		Self {
			inner: Arc::new(SdfSourceInner {
				handle: OnceLock::new(),
				bindings: RwLock::new(HashMap::new()),
				forgotten: ForgottenChunks::default(),
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
	}

	fn binding(&self, grid: GridId) -> Option<GridBinding> {
		self.inner.bindings.read().unwrap().get(&grid).cloned()
	}

	fn chunk_origin(chunk: IVec3) -> IVec3 {
		chunk * CHUNK_SIZE
	}

	fn chunk_of(voxel: IVec3) -> IVec3 {
		voxel.div_euclid(IVec3::splat(CHUNK_SIZE))
	}

	fn region_bounds(min: IVec3, size: IVec3) -> (Vec3, Vec3) {
		let lo = Self::chunk_origin(min).as_vec3();
		let hi = Self::chunk_origin(min + size).as_vec3();
		(lo, hi)
	}

	fn might_intersect_region(binding: &GridBinding, min: IVec3, size: IVec3) -> bool {
		let Some((bounds_min, bounds_max)) = binding.sdf.bounds() else { return true };
		if !bounds_min.cmplt(bounds_max).all() {
			return false;
		}
		let (region_min, region_max) = Self::region_bounds(min, size);
		region_min.cmplt(bounds_max).all() && region_max.cmpgt(bounds_min).all()
	}

	fn build_region(
		&self,
		binding: &GridBinding,
		min: IVec3,
		size: IVec3,
		lod: Option<f32>,
		voxel_type: Option<VoxelTypeId>,
		cancellation: Option<&CancellationToken>,
	) -> Option<Voxels> {
		if size.cmple(IVec3::ZERO).any() || !Self::might_intersect_region(binding, min, size) {
			return None;
		}
		let step = step_for_lod(lod.unwrap_or(0.0))?;
		let full_extent = size * CHUNK_SIZE;
		let extent = full_extent / step;
		if extent.cmple(IVec3::ZERO).any() || extent.cmpgt(IVec3::splat(i16::MAX as i32)).any() {
			return None;
		}

		let base_origin = Self::chunk_origin(min);
		let sample_radius = sample_radius(binding.options, step);
		let step_f32 = step as f32;
		let origin = base_origin.as_vec3();
		let local_sdf = |p: Vec3| {
			if cancellation.is_some_and(CancellationToken::is_cancelled) {
				f32::MAX
			} else {
				(binding.sdf.sample(origin + p * step_f32) - sample_radius) / step_f32
			}
		};

		let voxel = match voxel_type {
			Some(type_id) if binding.sdf.lod_voxel().type_id() == type_id => binding.sdf.lod_voxel(),
			Some(type_id) if binding.sdf.voxel().type_id() == type_id => binding.sdf.voxel(),
			Some(_) => return None,
			None => binding.sdf.voxel(),
		};
		let mut voxels = Voxels::new_with_type(voxel.type_info());
		voxels.apply_sdf(
			Vec3::ZERO,
			extent.as_vec3(),
			&local_sdf,
			IVec2::splat(9),
			8,
			voxel,
		);
		(!voxels.is_empty()).then_some(voxels)
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

	fn request_load(&self, grid: GridId, chunk: IVec3, generation: u64, cancellation: CancellationToken) -> SourceCoverage {
		let Some(binding) = self.binding(grid) else { return SourceCoverage::None };
		if cancellation.is_cancelled() || self.inner.forgotten.contains(grid, chunk)
			|| !Self::might_intersect_region(&binding, chunk, IVec3::ONE) { return SourceCoverage::None; }
		let voxels = self.build_region(&binding, chunk, IVec3::ONE, None, None, Some(&cancellation));
		if cancellation.is_cancelled() { return SourceCoverage::None; }
		if let Some(handle) = self.inner.handle.get() { handle.loaded(grid, chunk, generation, voxels); }
		SourceCoverage::All
	}

	fn request_voxel_area(
		&self,
		grid: GridId,
		min: IVec3,
		size: IVec3,
		lod: f32,
		voxel_type: VoxelTypeId,
		generation: u64,
		cancellation: CancellationToken,
	) -> SourceCoverage {
		if cancellation.is_cancelled() { return SourceCoverage::None; }
		let Some(binding) = self.binding(grid) else { return SourceCoverage::None };
		let mut owned = 0;
		for z in 0..size.z { for y in 0..size.y { for x in 0..size.x {
			let chunk = min + IVec3::new(x, y, z);
			owned += (!self.inner.forgotten.contains(grid, chunk)
				&& Self::might_intersect_region(&binding, chunk, IVec3::ONE)) as usize;
		}}}
		let coverage = SourceCoverage::from_count(owned, (size.x * size.y * size.z) as usize);
		if coverage == SourceCoverage::None { return coverage; }
		assert!(
			binding.sdf.voxel().type_id() == voxel_type || binding.sdf.lod_voxel().type_id() == voxel_type,
			"SDF source does not support requested voxel type",
		);
		let voxels = Some(binding).and_then(|binding| {
			let step = step_for_lod(lod)?;
			let chunk_extent = IVec3::splat(CHUNK_SIZE / step);
			let mut out: Option<Voxels> = None;
			for z in 0..size.z { for y in 0..size.y { for x in 0..size.x {
				let offset = IVec3::new(x, y, z);
				let chunk = min + offset;
				if self.inner.forgotten.contains(grid, chunk) { continue; }
				let Some(voxels) = self.build_region(&binding, chunk, IVec3::ONE, Some(lod), Some(voxel_type), Some(&cancellation)) else { continue };
				out.get_or_insert_with(|| Voxels::new_with_type(voxels.voxel_type_info())).merge_from(&voxels, offset * chunk_extent);
			}}}
			out.filter(|voxels| !voxels.is_empty())
		});
		if cancellation.is_cancelled() { return SourceCoverage::None; }
		if let Some(handle) = self.inner.handle.get() { handle.voxels_loaded(grid, min, size, lod, voxel_type, generation, voxels); }
		coverage
	}

	fn request_available_area(&self, grid: GridId) {
		let Some(handle) = self.inner.handle.get() else { return };
		if let Some(binding) = self.binding(grid)
			&& let Some((bounds_min, bounds_max)) = binding.sdf.bounds()
			&& bounds_min.cmplt(bounds_max).all() {
			let voxel_min = bounds_min.floor().as_ivec3();
			let voxel_max = bounds_max.ceil().as_ivec3() - IVec3::ONE;
			let chunk_min = Self::chunk_of(voxel_min);
			let chunk_max = Self::chunk_of(voxel_max);
			handle.presence(grid, chunk_min, chunk_max - chunk_min + IVec3::ONE);
		}
		handle.presence_loaded(grid);
	}

	fn take(&self, destination: voxel_sources::SourceId, grid: GridId, min: IVec3, size: IVec3, generation: u64) -> Vec<TakeJob> {
		let Some(binding) = self.binding(grid) else { return Vec::new() };
		let chunks = self.inner.forgotten.forget_area_where(grid, min, size, |chunk| Self::might_intersect_region(&binding, chunk, IVec3::ONE));
		let handle = self.inner.handle.get().expect("SDF source was not initialized").clone();
		let source = self.clone();
		chunks.into_iter().map(|chunk| TakeJob::new(chunk, {
			let handle = handle.clone();
			let source = source.clone();
			let binding = binding.clone();
			move || {
				let cancellation = CancellationToken::new();
				let voxels = source.build_region(&binding, chunk, IVec3::ONE, None, None, Some(&cancellation));
				handle.transferred(destination, grid, chunk, generation, voxels);
			}
		})).collect()
	}

	fn forget(&self, grid: GridId, chunk: IVec3) {
		self.inner.forgotten.forget(grid, chunk);
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

fn step_for_lod(lod: f32) -> Option<i32> {
	if !lod.is_finite() {
		return None;
	}
	let exp = lod.max(0.0).floor() as u32;
	1i32.checked_shl(exp).filter(|step| *step > 0)
}

