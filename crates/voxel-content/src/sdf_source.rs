use std::collections::HashMap;
use std::sync::{Arc, OnceLock, RwLock};

use bevy::ecs::resource::Resource;
use bevy::math::{IVec2, IVec3, Vec3};

use voxel_data::grid::GridId;
use voxel_data::sdf::Sdf;
use voxel_data::voxels::{VoxelRef, VoxelTypeId, Voxels};
use voxel_sources::{ChunkSource, RequestId, SourceCoverage, SourceHandle};
use tile_data::{CHUNK_SIZE, NonZeroChunkRegion};
use voxel_streaming::ForgottenChunks;
use voxel_tasks::{AsyncPriorityTaskPool, CancellationToken};

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
		let (region_min, region_max) = Self::region_bounds(region.min(), region.size().as_ivec3());
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
		if cancellation.is_cancelled() {
			return SourceCoverage::None;
		}
		let Some(binding) = self.binding(grid) else { return SourceCoverage::None };
		let mut owned_chunks = Vec::new();
		for z in region.min().z..region.end().z {
			for y in region.min().y..region.end().y {
				for x in region.min().x..region.end().x {
					let chunk = IVec3::new(x, y, z);
					if self.chunk_owned(grid, &binding, chunk) {
						owned_chunks.push(chunk);
					}
				}
			}
		}
		let coverage = if owned_chunks.is_empty() {
			return SourceCoverage::None;
		} else if owned_chunks.len() == region.area() as usize {
			SourceCoverage::All
		} else {
			SourceCoverage::Some
		};

		let handle = self.inner.handle.get().expect("SDF source was not initialized").clone();
		let cancellation = cancellation.clone();
		AsyncPriorityTaskPool::get().spawn(1.0, async move {
			let _span = bevy::log::info_span!("SdfSource build").entered();
			let step = step_for_lod(lod);
			let sample_radius = sample_radius(binding.options, step);
			let step_f32 = step as f32;
			let voxel = match voxel_type {
				Some(id) if binding.sdf.voxel().type_id() == id => binding.sdf.voxel(),
				Some(id) if binding.sdf.lod_voxel().type_id() == id => binding.sdf.lod_voxel(),
				_ if lod == 0 => binding.sdf.voxel(),
				_ => binding.sdf.lod_voxel(),
			};
			let chunk_extent = IVec3::splat(CHUNK_SIZE / step as i32);
			let mut merged: Option<Voxels> = None;

			for chunk in owned_chunks {
				if cancellation.is_cancelled() {
					break;
				}
				let origin = Self::chunk_origin(chunk).as_vec3();
				let local_sdf = |p: Vec3| {
					if cancellation.is_cancelled() {
						f32::MAX
					} else {
						(binding.sdf.sample(origin + p * step_f32) - sample_radius) / step_f32
					}
				};
				let mut voxels = Voxels::new_with_type(voxel.type_info());
				voxels.apply_sdf(
					Vec3::ZERO,
					chunk_extent.as_vec3(),
					&local_sdf,
					IVec2::splat(9),
					8,
					voxel,
				);
				if !voxels.is_empty() {
					let offset = (chunk - region.min()) * chunk_extent;
					merged.get_or_insert_with(|| Voxels::new_with_type(voxel.type_info())).merge_from(&voxels, offset);
				}
			}
			if !cancellation.is_cancelled() && let Some(voxels) = merged {
				handle.voxels(request_id, grid, region, lod, 0, voxels);
			}
			handle.voxels_loaded(request_id);
		});

		coverage
	}

	fn request_presence(&self, request_id: RequestId, _cancellation: CancellationToken, grid: GridId) {
		let handle = self.inner.handle.get().expect("SDF source was not initialized");
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
