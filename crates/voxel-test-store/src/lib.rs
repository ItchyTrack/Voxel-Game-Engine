use std::collections::HashMap;

use bevy::math::I16Vec3;
use bevy::prelude::*;

use voxel_data::voxels::{Voxel, Voxels};
use voxel_sources::{ChunkSource, GridKey, SourceHandle, VoxelSourcesAppExt};

const LOAD_COST: u32 = 0;

type ChunkData = Vec<(I16Vec3, Voxel)>;

#[derive(Default)]
struct MemoryStore {
	chunks: HashMap<(GridKey, IVec3), ChunkData>,
	handle: Option<SourceHandle>,
}

impl ChunkSource for MemoryStore {
	fn init(&mut self, handle: SourceHandle) {
		self.handle = Some(handle);
	}

	fn cost(&self, grid: GridKey, chunk: IVec3) -> Option<u32> {
		self.chunks.contains_key(&(grid, chunk)).then_some(LOAD_COST)
	}

	fn request_load(&mut self, grid: GridKey, chunk: IVec3) {
		let voxels = self.chunks.get(&(grid, chunk)).map(|list| {
			let mut voxels = Voxels::new();
			for (local, voxel) in list {
				voxels.add_voxel(*local, *voxel);
			}
			voxels
		});
		if let Some(handle) = &self.handle {
			handle.loaded(grid, chunk, voxels);
		}
	}

	fn can_save(&self) -> bool {
		true
	}

	fn save(&mut self, grid: GridKey, chunk: IVec3, voxels: &Voxels) {
		let key = (grid, chunk);
		if voxels.is_empty() {
			self.chunks.remove(&key);
			return;
		}
		let palette = voxels.palette();
		let mut list = Vec::new();
		for (pos, size, id) in voxels.grid_tree().iter() {
			let Some(voxel) = palette.voxel(id) else { continue };
			let voxel = *voxel;
			for dx in 0..size as i16 {
				for dy in 0..size as i16 {
					for dz in 0..size as i16 {
						list.push((pos + I16Vec3::new(dx, dy, dz), voxel));
					}
				}
			}
		}
		self.chunks.insert(key, list);
	}

	fn forget(&mut self, grid: GridKey, chunk: IVec3) {
		self.chunks.remove(&(grid, chunk));
	}
}

#[derive(Default)]
pub struct MemoryStorePlugin;

impl Plugin for MemoryStorePlugin {
	fn build(&self, app: &mut App) {
		app.register_source(MemoryStore::default());
	}
}
