use glam::{I16Vec3, IVec2};
use noise::{NoiseFn, SuperSimplex};

use crate::{physics::physics_body::PhysicsBodyGrid, voxels::{Voxel}};

pub struct WorldGenerator {
	noise: SuperSimplex,
}

impl WorldGenerator {
	pub const CHUNK_SIZE: u16 = 32;

	pub fn new(seed: u32) -> Self {
		Self {
			noise: SuperSimplex::new(seed),
		}
	}

	pub fn create_chunk(&self, chunk: IVec2, voxels: &mut PhysicsBodyGrid) {
		for x in 0..Self::CHUNK_SIZE {
			for z in 0..Self::CHUNK_SIZE {
				let noise0 = self.noise.get([
					(x as i32 + (chunk.x - 10000) * Self::CHUNK_SIZE as i32) as f64 * 0.005,
					(z as i32 + (chunk.y - 10000) * Self::CHUNK_SIZE as i32) as f64 * 0.005
				]);
				let noise1 = self.noise.get([
					(x as i32 + (chunk.x) * Self::CHUNK_SIZE as i32) as f64 * 0.03,
					(z as i32 + (chunk.y) * Self::CHUNK_SIZE as i32) as f64 * 0.03
				]);
				let noise = self.noise.get([
					(x as i32 + (chunk.x) * Self::CHUNK_SIZE as i32) as f64 * 0.04,
					(z as i32 + (chunk.y) * Self::CHUNK_SIZE as i32) as f64 * 0.04
				]);
				let noise2 = self.noise.get([
					(x as i32 + (chunk.x + 10000) * Self::CHUNK_SIZE as i32) as f64 * 0.06,
					(z as i32 + (chunk.y + 10000) * Self::CHUNK_SIZE as i32) as f64 * 0.06
				]);

				let height = ((noise0 + 1.0) * 20.0 + (noise1 + 1.0) * 5.0 + (noise2 + 1.0) * 3.0) as u16;
				for y in 0..height {
					if height - y > noise.round() as u16 + 4 {
						voxels.add_voxel(I16Vec3::new(x as i16, y as i16, z as i16), Voxel { color: [40, 40, 40, 255], mass: 10 });
					} else {
						voxels.add_voxel(I16Vec3::new(x as i16, y as i16, z as i16), Voxel { color: [107, 84, 40, 255], mass: 10 });
					}
				}
				voxels.add_voxel(I16Vec3::new(x as i16, height as i16, z as i16), Voxel { color: [0, 230, 0, 255], mass: 10 });
			}
		}
	}
}
