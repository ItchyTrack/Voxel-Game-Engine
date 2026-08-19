use std::collections::HashMap;
use std::sync::{Arc, OnceLock, RwLock};

use bevy::ecs::resource::Resource;
use bevy::math::{IVec2, IVec3, Vec3};

use voxel_data::grid::GridId;
use voxel_data::sdf::Sdf;
use voxel_data::voxels::{VoxelRef, VoxelTypeId, Voxels};
use voxel_sources::{CancellationToken, ChunkSource, RequestId, SourceCoverage, SourceHandle};
use tile_data::{CHUNK_SIZE, NonZeroChunkRegion, nonzero_voxel_region_from_chunks};
use voxel_streaming::ForgottenChunks;
use voxel_tasks::AsyncPriorityTaskPool;

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

	fn might_intersect_region(binding: &GridBinding, region: NonZeroChunkRegion) -> bool {
		let Some((bounds_min, bounds_max)) = binding.sdf.bounds() else { return true };
		if !bounds_min.cmplt(bounds_max).all() {
			return false;
		}
		let (region_min, region_max) = Self::region_bounds(min, size);
		region_min.cmplt(bounds_max).all() && region_max.cmpgt(bounds_min).all()
	}

	fn chunk_owned(&self, grid: GridId, binding: &GridBinding, chunk: IVec3) -> bool {
		!self.inner.forgotten.contains(grid, chunk) && Self::might_intersect_region(binding, NonZeroChunkRegion::from_single(chunk))
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

	fn request_voxels(
		&self,
		request_id: RequestId,
		cancellation: &CancellationToken,
		grid: GridId,
		region: NonZeroChunkRegion,
		lod: u8,
		voxel_type: Option<VoxelTypeId>,
	) -> SourceCoverage {
		let binding = self.binding(grid)?;
		let mut has_one = None;
		let mut source_coverage = SourceCoverage::All;
		'outer:for x in region.min().x..region.end().x {
			for y in region.min().y..region.end().y {
				for z in region.min().z..region.end().z {
					if self.chunk_owned(grid, &binding, chunk) {
						if let Some(has_one_val) = has_one {
							if has_one_val {
								source_coverage = SourceCoverage::Some;
								break 'outer;
							}
						} else {
							has_one = Some(true)
						}
					} else {
						if let Some(has_one_val) = has_one {
							if has_one_val {
								source_coverage = SourceCoverage::Some;
								break 'outer;
							}
						} else {
							has_one = Some(false)
						}
					}
				}
			}
		}
		assert!(has_one.is_some()); // has to happen as region is non zero
		if !has_one.unwrap() {
			return None;
		}

		let source = self.clone();
		let cancellation = cancellation.clone();

		AsyncPriorityTaskPool::get().spawn(1.0, async move || {
			if cancellation.is_cancelled() { return; }
			let step = step_for_lod(lod.unwrap_or(0));
			let voxel_region = nonzero_voxel_region_from_chunks(region);

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

			let voxel = if lod == 0 { binding.sdf.voxel() } else { binding.sdf.lod_voxel() };
			let mut voxels = Voxels::new_with_type(voxel.type_info());
			if cancellation.is_cancelled() { return; }
			voxels.apply_sdf(
				Vec3::ZERO,
				(voxel_region.size() / step).as_vec3(),
				&local_sdf,
				IVec2::splat(9),
				8,
				voxel,
			);
			if cancellation.is_cancelled() { return; }
			let handle = source.inner.handle.wait();
			handle.voxels(request_id, grid, region, lod, 0, voxels);
			handle.voxels_loaded(request_id);
		});

		source_coverage
	}

	fn request_presence(&self, request_id: RequestId, _cancellation: CancellationToken, grid: GridId) {
		let Some(handle) = self.inner.handle.get() else { return };
		if let Some(binding) = self.binding(grid)
			&& let Some((bounds_min, bounds_max)) = binding.sdf.bounds()
			&& bounds_min.cmplt(bounds_max).all() {
			let voxel_min = bounds_min.floor().as_ivec3();
			let voxel_max = bounds_max.ceil().as_ivec3() - IVec3::ONE;
			let chunk_min = Self::chunk_of(voxel_min);
			let chunk_max = Self::chunk_of(voxel_max);
			if let Some(region) = NonZeroChunkRegion::from_min_max(chunk_min, chunk_max) {
				handle.presence(request_id, grid, region);
			}
		}
		handle.presence_loaded(request_id);
	}

	fn take_ownership(&self, grid: GridId, region: NonZeroChunkRegion) {
		let Some(binding) = self.binding(grid) else { return };
		self.inner.forgotten.forget_area_where(grid, region.into(), |chunk| Self::might_intersect_region(&binding, NonZeroChunkRegion::from_single(chunk)));
	}
}

pub fn sdf_source() -> SdfSource {
	SdfSource::new()
}

fn sample_radius(options: SdfSourceOptions, step: u32) -> f32 {
	let scale = options.sample_radius_scale.max(0.0);
	if scale == 0.0 {
		0.0
	} else {
		Vec3::splat(step as f32).length() * 0.5 * scale
	}
}

fn step_for_lod(lod: u8) -> u32 {
	1u32 << lod
}
