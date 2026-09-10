use voxel_math::FixedVec3;
pub mod avbd;

use bevy::prelude::*;

use crate::sparse_set::SparseSet;
use crate::PhysicsBodyId;

pub use avbd::AvbdPlugin;

pub enum Impulse {
	Impulse {
		impulse: FixedVec3,
		impulse_pos: FixedVec3,
	},
	CentralImpulse {
		central_impulse: FixedVec3,
	},
	RotationalImpulse {
		rotational_impulse: FixedVec3,
	},
}

/// Queue of impulses to apply on the next physics step.
#[derive(Resource, Default)]
pub struct Impulses {
	pub(crate) map: SparseSet<PhysicsBodyId, Vec<Impulse>>,
}

impl Impulses {
	pub fn apply_central_impulse(&mut self, body: PhysicsBodyId, impulse: FixedVec3) {
		self.map.entry(body).or_default().push(Impulse::CentralImpulse { central_impulse: impulse });
	}
	pub fn apply_rotational_impulse(&mut self, body: PhysicsBodyId, impulse: FixedVec3) {
		self.map.entry(body).or_default().push(Impulse::RotationalImpulse { rotational_impulse: impulse });
	}
	pub fn apply_impulse(&mut self, body: PhysicsBodyId, pos: FixedVec3, impulse: FixedVec3) {
		self.map.entry(body).or_default().push(Impulse::Impulse { impulse, impulse_pos: pos });
	}
}

#[derive(Resource, Default)]
pub struct Accelerations {
	pub(crate) map: SparseSet<PhysicsBodyId, Vec<FixedVec3>>,
}

impl Accelerations {
	pub fn apply_central_acceleration(&mut self, body: PhysicsBodyId, acceleration: FixedVec3) {
		self.map.entry(body).or_default().push(acceleration);
	}
}

