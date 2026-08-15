use std::collections::HashMap;
use std::marker::PhantomData;
use std::path::PathBuf;
use std::sync::{Arc, OnceLock, RwLock};

use bevy::ecs::resource::Resource;
use bevy::math::{IVec3, Quat, U16Vec3, Vec3};
use voxel_data::compressed_voxels::CompressedVoxels;
use voxel_data::grid::GridId;
use voxel_data::grid_tree::NonZeroVoxelRegion;
use voxel_data::voxels::{VoxelType, VoxelTypeId, Voxels};
use voxel_sources::{CancellationToken, ChunkSource, SourceCoverage, SourceHandle, TakeJob};
use tile_data::{ChunkRegion, NonZeroChunkRegion, chunk_of};
use tile_data::CHUNK_SIZE;
use voxel_streaming::ForgottenChunks;


#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VoxMaterial {
	pub palette_index: u8,
	pub color: [u8; 4],
}

pub trait VoxMaterialVoxel: VoxelType {
	fn from_vox_material(material: VoxMaterial) -> Self;
}

#[derive(Resource, Clone)]
pub struct VoxFileSource<T: VoxMaterialVoxel> {
	inner: Arc<VoxFileSourceInner<T>>,
}

struct VoxFileSourceInner<T: VoxMaterialVoxel> {
	_voxel_type: PhantomData<T>,
	handle: OnceLock<SourceHandle>,
	bindings: RwLock<HashMap<GridId, GridBinding>>,
	files: RwLock<HashMap<PathBuf, FileCache>>,
	forgotten: ForgottenChunks,
}

#[derive(Clone)]
struct GridBinding {
	path: PathBuf,
	offset: IVec3,
}

#[derive(Default)]
struct FileCache {
	chunks: HashMap<IVec3, CompressedVoxels>,
	available_area: Option<ChunkRegion>,
	load_attempted: bool,
	load_failed: bool,
}

impl<T: VoxMaterialVoxel> VoxFileSource<T> {
	pub fn new() -> Self {
		Self {
			inner: Arc::new(VoxFileSourceInner {
				_voxel_type: PhantomData,
				handle: OnceLock::new(),
				bindings: RwLock::new(HashMap::new()),
				files: RwLock::new(HashMap::new()),
				forgotten: ForgottenChunks::default(),
			}),
		}
	}

	pub fn set_grid_vox_file(&self, grid: GridId, pos: Vec3, path: impl Into<PathBuf>) {
		self.inner.bindings.write().unwrap().insert(grid, GridBinding {
			path: path.into(),
			offset: pos.as_ivec3(),
		});
	}

	fn binding(&self, grid: GridId) -> Option<GridBinding> {
		self.inner.bindings.read().unwrap().get(&grid).cloned()
	}

	fn ensure_file_loaded(&self, path: &PathBuf) {
		if let Some(cache)= self.inner.files.read().unwrap().get(path) && cache.load_attempted{
			return;
		}

		let mut files = self.inner.files.write().unwrap();
		let cache = files.entry(path.clone()).or_default();
		cache.load_attempted = true;

		let Ok(bytes) = std::fs::read(path) else {
			cache.load_failed = true;
			return;
		};
		let Ok(dot_vox_data) = dot_vox::load_bytes(&bytes) else {
			cache.load_failed = true;
			return;
		};

		#[derive(Clone, Copy)]
		struct Frame {
			translation: Vec3,
			rotation: Quat,
			flip: IVec3,
		}

		let mut chunk_points: HashMap<IVec3, Vec<(U16Vec3, T)>> = HashMap::new();
		let mut current_chunk: Option<IVec3> = None;
		let mut current_points = Vec::new();
		let mut touched_bounds: Option<(IVec3, IVec3)> = None;
		let mut stack: Vec<(u32, Frame)> = vec![(0, Frame {
			translation: Vec3::ZERO,
			rotation: Quat::IDENTITY,
			flip: IVec3::new(1, 1, -1),
		})];
		while let Some((scene_id, pose)) = stack.pop() {
			let flush_current_chunk = |
				chunk_points: &mut HashMap<IVec3, Vec<(U16Vec3, T)>>,
				current_chunk: &mut Option<IVec3>,
				current_points: &mut Vec<(U16Vec3, T)>
			| {
				if let Some(chunk) = current_chunk.take() {
					chunk_points.insert(chunk, std::mem::take(current_points));
				}
			};
			let Some(node) = dot_vox_data.scenes.get(scene_id as usize) else { continue };
			match node {
				dot_vox::SceneNode::Transform { frames, child, .. } => {
					let Some(frame) = frames.first() else { continue };
					let pos = frame.position().unwrap_or(dot_vox::Position { x: 0, y: 0, z: 0 });
					let (rot, flip_vec) = frame.orientation().map(|q| {
						let (qarr, varr) = q.to_quat_scale();
						let q = Quat::from_array(qarr);
						let q = Quat::from_xyzw(q.x, q.z, -q.y, q.w);
						(q, Vec3::from_array(varr).as_ivec3())
					}).unwrap_or((Quat::IDENTITY, IVec3::ONE));
					stack.push((*child, Frame {
						translation: pose.translation + pose.rotation * Vec3::new(pos.x as f32, pos.z as f32, -pos.y as f32),
						rotation: pose.rotation * rot,
						flip: pose.flip * IVec3::new(flip_vec.x, flip_vec.z, flip_vec.y),
					}));
				}
				dot_vox::SceneNode::Group { children, .. } => {
					for child in children {
						stack.push((*child, pose));
					}
				}
				dot_vox::SceneNode::Shape { models, .. } => {
					for shape_model in models {
						let Some(model) = dot_vox_data.models.get(shape_model.model_id as usize) else { continue };
						let half = Vec3::new(
							model.size.x as f32 / 2.0,
							model.size.z as f32 / 2.0,
							model.size.y as f32 / 2.0,
						).floor();
						let base = pose.translation - pose.rotation * (half * pose.flip.as_vec3());
						let flip_min = pose.flip.min(IVec3::ZERO);
						for voxel in &model.voxels {
							let local = IVec3::new(voxel.x as i32, voxel.z as i32, voxel.y as i32) * pose.flip + flip_min;
							let source_pos = (base + pose.rotation * local.as_vec3()).as_ivec3();
							let chunk = chunk_of(source_pos);
							if current_chunk != Some(chunk) {
								flush_current_chunk(&mut chunk_points, &mut current_chunk, &mut current_points);
								current_chunk = Some(chunk);
								current_points = chunk_points.remove(&chunk).unwrap_or_default();
							}
							let local = source_pos.rem_euclid(IVec3::splat(CHUNK_SIZE)).as_u16vec3();
							let palette = dot_vox_data.palette[voxel.i as usize];
							let material = VoxMaterial {
								palette_index: voxel.i,
								color: [palette.r, palette.g, palette.b, palette.a],
							};
							current_points.push((local, T::from_vox_material(material)));
							touched_bounds = Some(match touched_bounds {
								Some((min, max)) => (min.min(chunk), max.max(chunk)),
								None => (chunk, chunk),
							});
						}
					}
				}
			}
		}

		if let Some(chunk) = current_chunk.take() {
			chunk_points.insert(chunk, std::mem::take(&mut current_points));
		}

		cache.available_area = touched_bounds.and_then(|(min, max)| ChunkRegion::from_min_max(min, max));

		for (chunk, points) in chunk_points {
			let voxel_refs: Vec<_> = points.iter().map(|(pos, voxel)| (*pos, voxel.get_ref())).collect();
			let mut voxels = Voxels::new::<T>();
			voxels.add_voxels(&voxel_refs);
			if let Ok(compressed) = CompressedVoxels::new(&voxels, 0) {
				cache.chunks.insert(chunk, compressed);
			}
		}
	}

	fn source_chunk(&self, path: &PathBuf, chunk: IVec3) -> Option<Voxels> {
		self.ensure_file_loaded(path);
		self.inner.files.read().unwrap().get(path)?.chunks.get(&chunk)?.decompress().ok()
	}

	fn translated_chunk(&self, binding: &GridBinding, grid_chunk: IVec3) -> Option<Voxels> {
		let grid_chunk_origin = grid_chunk * CHUNK_SIZE;
		let source_min = grid_chunk_origin - binding.offset;
		let source_max_exclusive = source_min + IVec3::splat(CHUNK_SIZE);
		let source_chunk_min = source_min.div_euclid(IVec3::splat(CHUNK_SIZE));
		let source_chunk_max = (source_max_exclusive - IVec3::ONE).div_euclid(IVec3::splat(CHUNK_SIZE));

		let mut out = Voxels::new::<T>();
		for z in source_chunk_min.z..=source_chunk_max.z {
			for y in source_chunk_min.y..=source_chunk_max.y {
				for x in source_chunk_min.x..=source_chunk_max.x {
					let source_chunk = IVec3::new(x, y, z);
					let source_chunk_origin = source_chunk * CHUNK_SIZE;
					let local_min = source_min.max(source_chunk_origin) - source_chunk_origin;
					let local_max = source_max_exclusive.min(source_chunk_origin + IVec3::splat(CHUNK_SIZE)) - source_chunk_origin;
					let local_size = local_max - local_min;
					if local_size.cmple(IVec3::ZERO).any() { continue; }
					let Some(source_voxels) = self.source_chunk(&binding.path, source_chunk) else { continue };
					out.merge_region_from(
						&source_voxels,
						NonZeroVoxelRegion::from_min_size(local_min, local_size),
						source_chunk_origin + binding.offset - grid_chunk_origin,
					);
				}
			}
		}
		if out.is_empty() { None } else { Some(out) }
	}

	fn translated_available_area(&self, binding: &GridBinding) -> Option<ChunkRegion> {
		self.ensure_file_loaded(&binding.path);
		let files = self.inner.files.read().unwrap();
		let area = files.get(&binding.path)?.available_area?;
		let min_voxel = area.min() * CHUNK_SIZE + binding.offset;
		let max_voxel_exclusive = area.end() * CHUNK_SIZE + binding.offset;
		let min_chunk = min_voxel.div_euclid(IVec3::splat(CHUNK_SIZE));
		let max_chunk = (max_voxel_exclusive - IVec3::ONE).div_euclid(IVec3::splat(CHUNK_SIZE));
		ChunkRegion::from_min_max(min_chunk, max_chunk)
	}
}

impl<T: VoxMaterialVoxel> ChunkSource for VoxFileSource<T> {
	fn init(&self, handle: SourceHandle) {
		let _ = self.inner.handle.set(handle);
	}

	fn request_load(&self, grid: GridId, chunk: IVec3, generation: u64, cancellation: CancellationToken) -> SourceCoverage {
		if cancellation.is_cancelled() || self.inner.forgotten.contains(grid, chunk) { return SourceCoverage::None; }
		let Some(voxels) = self.binding(grid).and_then(|binding| self.translated_chunk(&binding, chunk)) else { return SourceCoverage::None };
		if cancellation.is_cancelled() { return SourceCoverage::None; }
		if let Some(handle) = self.inner.handle.get() { handle.loaded(grid, chunk, generation, Some(voxels)); }
		SourceCoverage::All
	}

	fn request_voxel_area(
		&self,
		grid: GridId,
		region: NonZeroChunkRegion,
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
			owned += (!self.inner.forgotten.contains(grid, chunk) && self.translated_chunk(&binding, chunk).is_some()) as usize;
		}}}
		let coverage = SourceCoverage::from_count(owned, (size.x * size.y * size.z) as usize);
		if coverage == SourceCoverage::None { return coverage; }
		assert!(
			lod <= 0.0 && T::TYPE_ID == voxel_type
				|| self.inner.handle.get().and_then(|handle| handle.voxel_lod_generator(T::TYPE_ID, voxel_type)).is_some(),
			"VOX source does not support requested voxel type or LOD",
		);
		// Forgotten chunks are skipped rather than punched out of the result, so the
		// hole is exact at every LOD instead of snapped to the output grid.
		let fetch = |binding: &GridBinding, chunk: IVec3| {
			(!self.inner.forgotten.contains(grid, chunk)).then(|| self.translated_chunk(binding, chunk)).flatten()
		};
		let voxels = Some(binding).and_then(|binding| {
			if lod <= 0.0 && T::TYPE_ID == voxel_type {
				let mut merged = Voxels::new::<T>();
				for z in 0..size.z {
					for y in 0..size.y {
						for x in 0..size.x {
							let local = IVec3::new(x, y, z);
							if let Some(chunk) = fetch(&binding, min + local) {
								merged.merge_from(&chunk, local * CHUNK_SIZE);
							}
						}
					}
				}
				return (!merged.is_empty()).then_some(merged);
			}
			let handle = self.inner.handle.get()?;
			let generator = handle.voxel_lod_generator(T::TYPE_ID, voxel_type)?;
			generator.generate(min, size, lod, &|chunk| fetch(&binding, chunk))
		});
		if cancellation.is_cancelled() { return SourceCoverage::None; }
		if let Some(handle) = self.inner.handle.get() { handle.voxels_loaded(grid, region, lod, voxel_type, generation, voxels); }
		coverage
	}

	fn request_available_area(&self, grid: GridId) {
		let Some(handle) = self.inner.handle.get() else { return };
		if let Some(binding) = self.binding(grid)
			&& let Some(area) = self.translated_available_area(&binding) {
			handle.presence(grid, area.min(), area.size().as_ivec3());
		}
		handle.presence_loaded(grid);
	}

	fn take(&self, destination: voxel_sources::SourceId, grid: GridId, region: ChunkRegion, generation: u64) -> Vec<TakeJob> {
		let Some(binding) = self.binding(grid) else { return Vec::new() };
		let chunks = self.inner.forgotten.forget_area_where(grid, region, |chunk| self.translated_chunk(&binding, chunk).is_some());
		let handle = self.inner.handle.get().expect("VOX source was not initialized").clone();
		let source = VoxFileSource { inner: self.inner.clone() };
		chunks.into_iter().map(|chunk| TakeJob::new(chunk, {
			let handle = handle.clone();
			let source = VoxFileSource { inner: source.inner.clone() };
			let binding = binding.clone();
			move || {
				let voxels = source.translated_chunk(&binding, chunk);
				handle.transferred(destination, grid, chunk, generation, voxels);
			}
		})).collect()
	}

	fn forget(&self, grid: GridId, chunk: IVec3) {
		self.inner.forgotten.forget(grid, chunk);
	}
}

impl<T: VoxMaterialVoxel> Default for VoxFileSource<T> {
	fn default() -> Self {
		Self::new()
	}
}

pub fn vox_file_source<T: VoxMaterialVoxel>() -> VoxFileSource<T> {
	VoxFileSource::new()
}

#[cfg(test)]
mod tests {
	use super::*;

	#[repr(C)]
	#[derive(Clone, Copy, Debug, PartialEq, Eq, bytemuck::Pod, bytemuck::Zeroable)]
	struct TestVoxel {
		color: [u8; 4],
	}

	impl VoxelType for TestVoxel {
		const TYPE_ID: voxel_data::voxels::VoxelTypeId = voxel_data::voxels::VoxelTypeId(42);
	}

	impl VoxMaterialVoxel for TestVoxel {
		fn from_vox_material(material: VoxMaterial) -> Self {
			Self { color: material.color }
		}
	}

	fn church_path() -> PathBuf {
		PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../res/Church_Of_St_Sophia.vox")
	}

	#[test]
	fn zero_offset_translation_preserves_source_chunk_positions_for_church() {
		let source = VoxFileSource::<TestVoxel>::new();
		let path = church_path();
		source.ensure_file_loaded(&path);
		let binding = GridBinding { path: path.clone(), offset: IVec3::ZERO };
		let files = source.inner.files.read().unwrap();
		let cache = files.get(&path).expect("church cache");
		let sample_chunks: Vec<_> = cache.chunks.keys().copied().take(8).collect();
		drop(files);

		for chunk in sample_chunks {
			let raw = source.source_chunk(&path, chunk).expect("raw chunk");
			let translated = source.translated_chunk(&binding, chunk).expect("translated chunk");
			let raw_voxels: std::collections::HashMap<_, _> = raw
				.grid_tree()
				.iter()
				.map(|(pos, _size, voxel)| (pos, voxel))
				.collect();
			let translated_voxels: std::collections::HashMap<_, _> = translated
				.grid_tree()
				.iter()
				.map(|(pos, _size, voxel)| (pos, voxel))
				.collect();
			assert_eq!(raw_voxels, translated_voxels, "chunk {chunk:?} changed under zero-offset translation");
		}
	}

	#[test]
	fn zero_offset_available_area_matches_source_chunk_bounds_for_church() {
		let source = VoxFileSource::<TestVoxel>::new();
		let path = church_path();
		source.ensure_file_loaded(&path);
		let binding = GridBinding { path: path.clone(), offset: IVec3::ZERO };
		let files = source.inner.files.read().unwrap();
		let area = files.get(&path).and_then(|cache| cache.available_area).expect("church area");
		drop(files);
		assert_eq!(source.translated_available_area(&binding), Some(area));
	}
}
