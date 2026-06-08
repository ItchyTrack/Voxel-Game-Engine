
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};

use bevy::math::I16Vec3;
use bevy::prelude::*;

use voxel_data::grid::Grid;
use voxel_data::voxels::{Voxel, Voxels};
use voxel_edit::GridEdits;
use voxel_sources::{ChunkSource, GridKey, SourceHandle, VoxelSourcesAppExt};
use voxel_streaming::{chunk_of, GridStreaming, CHUNK_SIZE};

use crate::lod_downsample::downsample_region;

const ORIGINAL_COST: u32 = 10;

type ChunkData = Vec<(I16Vec3, Voxel)>;
type Chunks = Arc<Mutex<HashMap<(GridKey, IVec3), ChunkData>>>;

#[derive(Resource, Clone, Default)]
pub struct WorldStore {
	chunks: Chunks,
	next_key: Arc<AtomicU64>,
}

impl WorldStore {
	pub fn alloc_key(&self) -> GridKey {
		GridKey(self.next_key.fetch_add(1, Ordering::Relaxed))
	}

	fn insert(&self, key: GridKey, chunk: IVec3, local: I16Vec3, voxel: Voxel) {
		self.chunks.lock().unwrap().entry((key, chunk)).or_default().push((local, voxel));
	}
}

struct WorldSource {
	chunks: Chunks,
	handle: OnceLock<SourceHandle>,
}

impl WorldSource {
	fn build_chunk(&self, grid: GridKey, chunk: IVec3) -> Option<Voxels> {
		self.chunks.lock().unwrap().get(&(grid, chunk)).map(|list| {
			let mut voxels = Voxels::new();
			for (local, voxel) in list {
				voxels.add_voxel(*local, *voxel);
			}
			voxels
		})
	}
}

impl ChunkSource for WorldSource {
	fn init(&self, handle: SourceHandle) {
		let _ = self.handle.set(handle);
	}

	fn cost(&self, grid: GridKey, chunk: IVec3) -> Option<u32> {
		self.chunks.lock().unwrap().contains_key(&(grid, chunk)).then_some(ORIGINAL_COST)
	}

	fn request_load(&self, grid: GridKey, chunk: IVec3) {
		let voxels = self.build_chunk(grid, chunk);
		if let Some(handle) = self.handle.get() {
			handle.loaded(grid, chunk, voxels);
		}
	}

	fn cost_lod(&self, grid: GridKey, min: IVec3, size: IVec3, _lod: f32) -> Option<u32> {
		let chunks = self.chunks.lock().unwrap();
		let region_has_data = (0..size.z).any(|z| (0..size.y).any(|y| (0..size.x).any(|x| {
			chunks.contains_key(&(grid, min + IVec3::new(x, y, z)))
		})));
		region_has_data.then_some(ORIGINAL_COST)
	}

	fn request_load_lod(&self, grid: GridKey, min: IVec3, size: IVec3, lod: f32) {
		let region = downsample_region(min, size, lod, |chunk| self.build_chunk(grid, chunk));
		let voxels = (!region.is_empty()).then_some(region);
		if let Some(handle) = self.handle.get() {
			handle.loaded_lod(grid, min, size, lod, voxels);
		}
	}

	fn forget(&self, grid: GridKey, chunk: IVec3) {
		self.chunks.lock().unwrap().remove(&(grid, chunk));
	}
}

pub struct StreamingTestPlugin;

impl Plugin for StreamingTestPlugin {
	fn build(&self, app: &mut App) {
		let store = WorldStore::default();
		app.register_source(WorldSource { chunks: store.chunks.clone(), handle: OnceLock::new() });
		app.insert_resource(store);
	}
}

/// Collects voxels destined for the streaming store before a grid is spawned.
#[derive(Default)]
pub struct StreamingVoxels(Vec<(IVec3, Voxel)>);

impl StreamingVoxels {
	pub fn new() -> Self {
		Self::default()
	}

	pub fn add_voxel(&mut self, pos: &IVec3, voxel: &Voxel) {
		self.0.push((*pos, *voxel));
	}
}

pub fn spawn_grid(
	commands: &mut Commands,
	store: &mut WorldStore,
	parent: Entity,
	transform: Transform,
	voxels: StreamingVoxels,
	extra: impl Bundle,
) {
	let key = store.alloc_key();
	let child = commands
		.spawn((transform, Grid::new(), GridEdits::default(), key, extra))
		.id();
	commands.entity(parent).add_child(child);

	let mut streaming = GridStreaming::default();
	for (voxel_pos, voxel) in voxels.0 {
		let chunk = chunk_of(voxel_pos);
		let local = voxel_pos.rem_euclid(IVec3::splat(CHUNK_SIZE)).as_i16vec3();
		streaming.presence_mut().mark_present(chunk);
		store.insert(key, chunk, local, voxel);
	}
	commands.entity(child).insert(streaming);
}
