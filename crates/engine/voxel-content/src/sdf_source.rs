use std::collections::HashMap;
use std::sync::Arc;

use bevy::math::{IVec2, IVec3, Vec3};

use voxel_data::{grid::GridId, voxels::{VoxelRef, VoxelTypeId, Voxels}};
use voxel_trees::sdf::Sdf;
use voxel_sources::{ForgottenChunks, ChunkSource, RequestId, SourceCoverage, SourceHandle, edit::GridGeneration};
use bile_math::NonZeroRegion;
use tile_data::{CHUNK_SIZE, chunk_aligned, chunk_origin, chunk_region, chunks_in};
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

pub struct SdfSource {
	handle: Option<SourceHandle>,
	bindings: HashMap<GridId, GridBinding>,
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
			handle: None,
			bindings: HashMap::new(),
			forgotten: ForgottenChunks::default(),
		}
	}

	pub fn set_grid_sdf<S: VoxelSdf>(&mut self, grid: GridId, sdf: S) {
		self.set_grid_sdf_with_options(grid, sdf, SdfSourceOptions::default());
	}

	pub fn set_grid_sdf_with_options<S: VoxelSdf>(&mut self, grid: GridId, sdf: S, options: SdfSourceOptions) {
		self.bindings.insert(grid, GridBinding {
			sdf: Arc::new(sdf),
			options,
		});
	}

	fn binding(&self, grid: GridId) -> Option<GridBinding> {
		self.bindings.get(&grid).cloned()
	}

	fn might_intersect_region(binding: &GridBinding, region: NonZeroRegion) -> bool {
		let Some((bounds_min, bounds_max)) = binding.sdf.bounds() else { return true };
		if !bounds_min.cmplt(bounds_max).all() {
			return false;
		}
		let (region_min, region_max) = (region.min().as_vec3(), region.end().as_vec3());
		region_min.cmplt(bounds_max).all() && region_max.cmpgt(bounds_min).all()
	}

	fn chunk_owned(&self, grid: GridId, binding: &GridBinding, chunk: IVec3) -> bool {
		!self.forgotten.contains(grid, chunk) && Self::might_intersect_region(binding, chunk_region(chunk))
	}
}

impl Default for SdfSource {
	fn default() -> Self {
		Self::new()
	}
}

impl ChunkSource for SdfSource {
	fn init(&mut self, handle: SourceHandle) {
		self.handle = Some(handle);
	}

	fn request_voxels(
		&mut self,
		request_id: RequestId,
		cancellation: &CancellationToken,
		grid: GridId,
		region: NonZeroRegion,
		lod: u8,
		voxel_type: Option<VoxelTypeId>,
		generation: GridGeneration,
	) -> SourceCoverage {
		if cancellation.is_cancelled() {
			return SourceCoverage::None;
		}
		let Some(binding) = self.binding(grid) else { return SourceCoverage::None };
		let owned_chunks: Vec<_> = chunks_in(region).filter(|&chunk| self.chunk_owned(grid, &binding, chunk)).collect();
		let coverage = if owned_chunks.is_empty() {
			return SourceCoverage::None;
		} else if owned_chunks.len() == chunks_in(region).count() {
			SourceCoverage::All
		} else {
			SourceCoverage::Some
		};

		let handle = self.handle.as_ref().expect("SDF source was not initialized").clone();
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
			let chunk_extent = IVec3::splat(CHUNK_SIZE as i32 / step as i32);
			let mut merged: Option<Voxels> = None;

			for chunk in owned_chunks {
				if cancellation.is_cancelled() {
					break;
				}
				let origin = chunk_origin(chunk).as_vec3();
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
					let offset = (chunk_origin(chunk) - region.min()) / step as i32;
					merged.get_or_insert_with(|| Voxels::new_with_type(voxel.type_info())).merge_from(&voxels, offset);
				}
			}
			if !cancellation.is_cancelled() && let Some(voxels) = merged {
				handle.voxels(request_id, grid, region, lod, generation, voxels);
			}
			handle.voxels_loaded(request_id, generation);
		});

		coverage
	}

	fn request_presence(&mut self, request_id: RequestId, _cancellation: CancellationToken, grid: GridId) {
		let handle = self.handle.as_ref().expect("SDF source was not initialized");
		if let Some(binding) = self.binding(grid)
			&& let Some((bounds_min, bounds_max)) = binding.sdf.bounds()
			&& bounds_min.cmplt(bounds_max).all() {
			if let Some(region) = NonZeroRegion::from_min_end(bounds_min.floor().as_ivec3(), bounds_max.ceil().as_ivec3()) {
				handle.presence(request_id, grid, chunk_aligned(region));
			}
		}
		handle.presence_loaded(request_id);
	}

	fn acquire_ownership(&mut self, grid: GridId, region: NonZeroRegion) {
		if self.binding(grid).is_none() { return; }
		self.forgotten.remember_area(grid, region);
	}

	fn relinquish_ownership(&mut self, grid: GridId, region: NonZeroRegion) {
		if self.binding(grid).is_none() { return; }
		self.forgotten.forget_area(grid, region);
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
