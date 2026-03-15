use glam::{I16Vec3, IVec2};
use noise::{NoiseFn, Perlin};

use crate::{physics::physics_body::PhysicsBodySubGrid, voxels::{Voxel}};


pub struct WorldGenerator {
	perlin: Perlin,
}

impl WorldGenerator {
	pub const CHUNK_SIZE: u16 = 32;

	pub fn new(seed: u32) -> Self {
		Self {
			perlin: Perlin::new(seed),
		}
	}

	pub fn create_chunk(&self, chunk: IVec2, voxels: &mut PhysicsBodySubGrid) {
		for x in 0..Self::CHUNK_SIZE {
			for z in 0..Self::CHUNK_SIZE {
				let height = ((self.perlin.get([(x as i32 + chunk.x * Self::CHUNK_SIZE as i32) as f64 * 0.03, (z as i32 + chunk.y * Self::CHUNK_SIZE as i32) as f64 * 0.03]) + 1.0) * 10.0) as u16;
				for y in 0..height {
					voxels.add_voxel(I16Vec3::new(x as i16, y as i16, z as i16), Voxel { color: [124, 63, 0, 255], mass: 10 });
				}
				voxels.add_voxel(I16Vec3::new(x as i16, height as i16, z as i16), Voxel { color: [0, 160, 0, 255], mass: 10 });
			}
		}
	}
}
