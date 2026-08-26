pub mod chunk_requests;
pub mod exact;

use bevy::prelude::*;

pub use chunk_requests::PhysicsConsumer;
// pub use exact::ExactPlugin;

use crate::{GridId, PhysicsBodyId};

#[derive(Copy, Clone, Hash, PartialEq, Eq)]
pub enum CubeFeature {
	Vertex { xyz: u8 },
	Edge { vertex_vertex: u8 },
	Face { xyzs: u8 },
}

#[derive(Copy, Clone)]
pub struct HalfCollision {
	pub body_id: PhysicsBodyId,
	pub grid_id: GridId,
	pub voxel_pos: IVec3,
	pub feature: CubeFeature,
	pub collision: Vec3,
	pub local_collision: Vec3,
}

#[derive(Copy, Clone)]
pub struct Collision {
	pub part1: HalfCollision,
	pub part2: HalfCollision,
}

impl Collision {
	pub fn get_swapped(&self) -> Collision {
		Collision {
			part1: self.part2,
			part2: self.part1,
		}
	}
}

/// Contacts produced by the collision pass and consumed by the solver.
#[derive(Resource, Default)]
pub struct Collisions(pub Vec<Collision>);
