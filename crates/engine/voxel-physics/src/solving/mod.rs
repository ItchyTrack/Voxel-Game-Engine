pub mod avbd;

use bevy::prelude::*;

use crate::sparse_set::SparseSet;
use crate::PhysicsBodyId;

pub use avbd::AvbdPlugin;

pub enum Impulse {
	Impulse {
		impulse: Vec3,
		impulse_pos: Vec3,
	},
	CentralImpulse {
		central_impulse: Vec3,
	},
	RotationalImpulse {
		rotational_impulse: Vec3,
	},
}

/// Queue of impulses to apply on the next physics step.
#[derive(Resource, Default)]
pub struct Impulses {
	pub(crate) map: SparseSet<PhysicsBodyId, Vec<Impulse>>,
}

impl Impulses {
	pub fn apply_central_impulse(&mut self, body: PhysicsBodyId, impulse: Vec3) {
		self.map.entry(body).or_default().push(Impulse::CentralImpulse { central_impulse: impulse });
	}
	pub fn apply_rotational_impulse(&mut self, body: PhysicsBodyId, impulse: Vec3) {
		self.map.entry(body).or_default().push(Impulse::RotationalImpulse { rotational_impulse: impulse });
	}
	pub fn apply_impulse(&mut self, body: PhysicsBodyId, pos: Vec3, impulse: Vec3) {
		self.map.entry(body).or_default().push(Impulse::Impulse { impulse, impulse_pos: pos });
	}
}

#[derive(Resource, Default)]
pub struct Accelerations {
	pub(crate) map: SparseSet<PhysicsBodyId, Vec<Vec3>>,
}

impl Accelerations {
	pub fn apply_central_acceleration(&mut self, body: PhysicsBodyId, acceleration: Vec3) {
		self.map.entry(body).or_default().push(acceleration);
	}
}

