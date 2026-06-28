use std::collections::HashMap;

use bevy::prelude::*;
use voxel_data::voxels::{Voxel, Voxels};
use voxel_streaming::{CHUNK_SIZE, chunk_of};

use crate::VoxelSdf;

#[derive(Default)]
pub struct StreamingVoxels {
	chunks: HashMap<IVec3, Voxels>,
}

impl StreamingVoxels {
	pub fn new() -> Self {
		Self::default()
	}

	pub fn reserve(&mut self, _additional: usize) {}

	pub fn add_voxel(&mut self, pos: &IVec3, voxel: &Voxel) {
		let chunk = chunk_of(*pos);
		let local = pos.rem_euclid(IVec3::splat(CHUNK_SIZE)).as_i16vec3();
		self.chunks.entry(chunk).or_insert_with(Voxels::new).add_voxel(local, *voxel);
	}

	pub fn add_sdf<S: VoxelSdf>(&mut self, min: IVec3, size: IVec3, sdf: &S) {
		if size.cmple(IVec3::ZERO).any() {
			return;
		}
		let max = min + size;
		let chunk_min = chunk_of(min);
		let chunk_max = chunk_of(max - IVec3::ONE);
		for z in chunk_min.z..=chunk_max.z {
			for y in chunk_min.y..=chunk_max.y {
				for x in chunk_min.x..=chunk_max.x {
					let chunk = IVec3::new(x, y, z);
					let chunk_origin = chunk * CHUNK_SIZE;
					let local_min = min.max(chunk_origin) - chunk_origin;
					let local_max = max.min(chunk_origin + IVec3::splat(CHUNK_SIZE)) - chunk_origin;
					let local_size = local_max - local_min;
					if local_size.cmple(IVec3::ZERO).any() {
						continue;
					}

					let mut chunk_voxels = Voxels::new();
					let local_sdf = |p: Vec3| sdf.sample(p + chunk_origin.as_vec3());
					chunk_voxels.apply_sdf(
						local_min.as_vec3(),
						local_max.as_vec3(),
						&local_sdf,
						IVec2::splat(9),
						8,
						sdf.voxel((chunk_origin + local_min).as_vec3()),
					);
					if !chunk_voxels.is_empty() {
						self.chunks.entry(chunk).or_insert_with(Voxels::new).merge_from(&chunk_voxels, IVec3::ZERO);
					}
				}
			}
		}
	}

	pub fn chunk_positions(&self) -> impl Iterator<Item = IVec3> + '_ {
		self.chunks.keys().copied()
	}

	pub fn into_chunk_data(self) -> HashMap<IVec3, Voxels> {
		self.chunks
	}
}
