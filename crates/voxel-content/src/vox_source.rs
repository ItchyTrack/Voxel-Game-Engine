use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::{Arc, OnceLock, RwLock};

use bevy::ecs::resource::Resource;
use bevy::math::{IVec3, Quat, U16Vec3, Vec3};
use voxel_data::compressed_voxels::CompressedVoxels;
use voxel_data::grid::GridId;
use voxel_data::grid_tree::GridRegion;
use voxel_data::voxels::{Voxel, VoxelPalette, VoxelTypeInfo, Voxels};
use voxel_sources::{CancellationToken, ChunkSource, SourceHandle};
use voxel_streaming::{CHUNK_SIZE, chunk_of};

const COST: u32 = 10;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VoxMaterial {
	pub palette_index: u8,
	pub color: [u8; 4],
}

pub trait VoxMaterialMapper: Clone + Send + Sync + 'static {
	fn voxel_type_info(&self) -> VoxelTypeInfo;
	fn voxel(&self, material: VoxMaterial) -> Voxel;
}

#[derive(Resource, Clone)]
pub struct VoxFileSource<M: VoxMaterialMapper> {
	inner: Arc<VoxFileSourceInner<M>>,
}

struct VoxFileSourceInner<M: VoxMaterialMapper> {
	mapper: M,
	handle: OnceLock<SourceHandle>,
	bindings: RwLock<HashMap<GridId, GridBinding>>,
	files: RwLock<HashMap<PathBuf, FileCache>>,
}

#[derive(Clone)]
struct GridBinding {
	path: PathBuf,
	offset: IVec3,
}

#[derive(Default)]
struct FileCache {
	chunks: HashMap<IVec3, CompressedVoxels>,
	available_area: Option<(IVec3, IVec3)>,
	load_attempted: bool,
	load_failed: bool,
}

impl<M: VoxMaterialMapper> VoxFileSource<M> {
	pub fn new(mapper: M) -> Self {
		Self {
			inner: Arc::new(VoxFileSourceInner {
				mapper,
				handle: OnceLock::new(),
				bindings: RwLock::new(HashMap::new()),
				files: RwLock::new(HashMap::new()),
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
		let mut files = self.inner.files.write().unwrap();
		let cache = files.entry(path.clone()).or_default();
		if cache.load_attempted {
			return;
		}
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

		let mut chunk_points: HashMap<IVec3, Vec<(U16Vec3, Voxel)>> = HashMap::new();
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
				chunk_points: &mut HashMap<IVec3, Vec<(U16Vec3, Voxel)>>,
				current_chunk: &mut Option<IVec3>,
				current_points: &mut Vec<(U16Vec3, Voxel)>
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
							current_points.push((local, self.inner.mapper.voxel(material)));
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

		cache.available_area = touched_bounds.map(|(min, max)| (min, max - min + IVec3::ONE));

		for (chunk, points) in chunk_points {
			let type_info = self.inner.mapper.voxel_type_info();
			let mut palette = VoxelPalette::new_with_type(type_info);
			let palette_points: Vec<_> = points.iter().map(|(pos, voxel)| (*pos, palette.palette_id(voxel.get_ref()))).collect();
			let mut voxels = Voxels::new_with_type(type_info);
			voxels.add_voxels(&palette_points, &palette);
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

		let mut out = Voxels::new_with_type(self.inner.mapper.voxel_type_info());
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
						GridRegion::from_min_size(local_min, local_size),
						source_chunk_origin + binding.offset - grid_chunk_origin,
					);
				}
			}
		}
		if out.is_empty() { None } else { Some(out) }
	}

	fn load_lod0_region(&self, binding: &GridBinding, min: IVec3, size: IVec3) -> Option<Voxels> {
		let mut out = Voxels::new_with_type(self.inner.mapper.voxel_type_info());
		for z in 0..size.z {
			for y in 0..size.y {
				for x in 0..size.x {
					let local = IVec3::new(x, y, z);
					let Some(voxels) = self.translated_chunk(binding, min + local) else { continue };
					out.merge_from(&voxels, local * CHUNK_SIZE);
				}
			}
		}
		(!out.is_empty()).then_some(out)
	}

	fn translated_available_area(&self, binding: &GridBinding) -> Option<(IVec3, IVec3)> {
		self.ensure_file_loaded(&binding.path);
		let files = self.inner.files.read().unwrap();
		let area = files.get(&binding.path)?.available_area?;
		let min_voxel = area.0 * CHUNK_SIZE + binding.offset;
		let max_voxel_exclusive = (area.0 + area.1) * CHUNK_SIZE + binding.offset;
		let min_chunk = min_voxel.div_euclid(IVec3::splat(CHUNK_SIZE));
		let max_chunk = (max_voxel_exclusive - IVec3::ONE).div_euclid(IVec3::splat(CHUNK_SIZE));
		Some((min_chunk, max_chunk - min_chunk + IVec3::ONE))
	}
}

impl<M: VoxMaterialMapper> ChunkSource for VoxFileSource<M> {
	fn init(&self, handle: SourceHandle) {
		let _ = self.inner.handle.set(handle);
	}

	fn cost(&self, grid: GridId, chunk: IVec3) -> Option<u32> {
		let binding = self.binding(grid)?;
		self.translated_chunk(&binding, chunk).map(|_| COST)
	}

	fn request_load(&self, grid: GridId, chunk: IVec3, generation: u64, cancellation: CancellationToken) {
		if cancellation.is_cancelled() { return; }
		let voxels = self.binding(grid).and_then(|binding| self.translated_chunk(&binding, chunk));
		if cancellation.is_cancelled() { return; }
		if let Some(handle) = self.inner.handle.get() {
			handle.loaded(grid, chunk, generation, voxels);
		}
	}

	fn cost_lod(&self, grid: GridId, min: IVec3, size: IVec3, lod: f32) -> Option<u32> {
		if lod > 0.0 { return None; }
		let binding = self.binding(grid)?;
		let region_has_data = (0..size.z).any(|z| (0..size.y).any(|y| (0..size.x).any(|x| {
			self.translated_chunk(&binding, min + IVec3::new(x, y, z)).is_some()
		})));
		region_has_data.then_some(COST)
	}

	fn request_load_lod(&self, grid: GridId, min: IVec3, size: IVec3, lod: f32, generation: u64, cancellation: CancellationToken) {
		if cancellation.is_cancelled() { return; }
		let voxels = self.binding(grid).and_then(|binding| self.load_lod0_region(&binding, min, size));
		if cancellation.is_cancelled() { return; }
		if let Some(handle) = self.inner.handle.get() {
			handle.loaded_lod(grid, min, size, lod, generation, voxels);
		}
	}

	fn request_available_area(&self, grid: GridId) {
		let Some(handle) = self.inner.handle.get() else { return };
		if let Some(binding) = self.binding(grid)
			&& let Some((min, size)) = self.translated_available_area(&binding) {
			handle.claim(grid, min, size);
		}
		handle.presence_loaded(grid);
	}
}

pub fn vox_file_source<M: VoxMaterialMapper>(mapper: M) -> VoxFileSource<M> {
	VoxFileSource::new(mapper)
}

#[cfg(test)]
mod tests {
	use super::*;

	#[derive(Clone, Copy)]
	struct TestMapper;

	impl VoxMaterialMapper for TestMapper {
		fn voxel_type_info(&self) -> VoxelTypeInfo {
			VoxelTypeInfo { id: voxel_data::voxels::VoxelTypeId(42), size_bytes: 4 }
		}

		fn voxel(&self, material: VoxMaterial) -> Voxel {
			Voxel::new(self.voxel_type_info().id, material.color)
		}
	}

	fn church_path() -> PathBuf {
		PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../res/Church_Of_St_Sophia.vox")
	}

	#[test]
	fn zero_offset_translation_preserves_source_chunk_positions_for_church() {
		let source = VoxFileSource::new(TestMapper);
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
				.map(|(pos, _size, id)| (pos, raw.voxel_for_palette_id(id).expect("raw palette")))
				.collect();
			let translated_voxels: std::collections::HashMap<_, _> = translated
				.grid_tree()
				.iter()
				.map(|(pos, _size, id)| (pos, translated.voxel_for_palette_id(id).expect("translated palette")))
				.collect();
			assert_eq!(raw_voxels, translated_voxels, "chunk {chunk:?} changed under zero-offset translation");
		}
	}

	#[test]
	fn zero_offset_available_area_matches_source_chunk_bounds_for_church() {
		let source = VoxFileSource::new(TestMapper);
		let path = church_path();
		source.ensure_file_loaded(&path);
		let binding = GridBinding { path: path.clone(), offset: IVec3::ZERO };
		let files = source.inner.files.read().unwrap();
		let area = files.get(&path).and_then(|cache| cache.available_area).expect("church area");
		drop(files);
		assert_eq!(source.translated_available_area(&binding), Some(area));
	}
}
