use glam::{IVec3, IVec2};
use noise::{NoiseFn, SuperSimplex};

use crate::{physics::physics_body::PhysicsBodyGrid, resource_manager::ResourceManager, voxels::Voxel};

pub struct WorldGenerator {
	noise: SuperSimplex,
}

impl WorldGenerator {
	pub fn new(seed: u32) -> Self {
		Self {
			noise: SuperSimplex::new(seed),
		}
	}

	pub fn gererate_area(&self, lower: IVec2, higher: IVec2, voxels: &mut PhysicsBodyGrid, resource_manager: &mut ResourceManager) {
		for x in lower.x..higher.x {
			for z in lower.y..higher.y {
				let noise0 = self.noise.get([
					(x as i32) as f64 * 0.0005,
					(z as i32) as f64 * 0.0005
				]);
				let noise1 = self.noise.get([
					(x as i32) as f64 * 0.003,
					(z as i32) as f64 * 0.003
				]);
				let noise = self.noise.get([
					(x as i32) as f64 * 0.004,
					(z as i32) as f64 * 0.004
				]);
				let noise2 = self.noise.get([
					(x as i32) as f64 * 0.006,
					(z as i32) as f64 * 0.006
				]);

				let height = ((noise0 + 1.0) * 50.0 + (noise1 + 1.0) * 10.0 + (noise2 + 1.0) * 10.0) as u16;
				for y in 0..height {
					if height - y > noise.round() as u16 + 4 {
						voxels.add_voxel(IVec3::new(x as i32, y as i32, z as i32), Voxel { color: [40, 40, 40, 255], mass: 10 }, resource_manager);
					} else {
						voxels.add_voxel(IVec3::new(x as i32, y as i32, z as i32), Voxel { color: [107, 84, 40, 255], mass: 10 }, resource_manager);
					}
				}
				voxels.add_voxel(IVec3::new(x as i32, height as i32, z as i32), Voxel { color: [0, 230, 0, 255], mass: 10 }, resource_manager);
			}
		}
	}
}
