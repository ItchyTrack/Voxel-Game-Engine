mod avbd;
mod body;

use std::collections::HashMap;

use bevy::prelude::*;
use num::Zero;
use voxel_data::grid::Grid;

use crate::ball_joint_constraint::BallJointConstraint;
use crate::collision::Collisions;
use crate::components::{AngularVelocity, CenterOfMass, IsStatic, Mass, RigidBody, RotationalInertia, Velocity};
use crate::sparse_set::SparseSet;
use crate::{FreezePhysics, PhysicsBodyId, PhysicsSet};

pub(crate) use avbd::Solver;
use avbd::Impulse;
use body::SolverBody;

/// Cross-frame solver state (warm-started K/L values for collision constraints).
#[derive(Resource)]
struct PhysicsSolver(Solver);

impl Default for PhysicsSolver {
	fn default() -> Self { Self(Solver::new()) }
}

/// Ball-joint constraints between rigid body entities. Keyed by the ordered
/// pair of body entities (lower first).
#[derive(Resource, Default)]
pub struct BallJointConstraints {
	map: HashMap<(PhysicsBodyId, PhysicsBodyId), BallJointConstraint>,
}

impl BallJointConstraints {
	pub fn insert(&mut self, a: PhysicsBodyId, b: PhysicsBodyId, constraint: BallJointConstraint) {
		let key = if a < b { (a, b) } else { (b, a) };
		self.map.insert(key, constraint);
	}
	pub fn remove(&mut self, a: PhysicsBodyId, b: PhysicsBodyId) {
		let key = if a < b { (a, b) } else { (b, a) };
		self.map.remove(&key);
	}
}

/// Queue of impulses to apply on the next physics step.
#[derive(Resource, Default)]
pub struct Impulses {
	map: SparseSet<PhysicsBodyId, Vec<Impulse>>,
}

impl Impulses {
	pub fn apply_central_impulse(&mut self, body: PhysicsBodyId, impulse: Vec3) {
		self.map.entry(body).or_default().push(Impulse::CentralImpulse { central_impluse: impulse });
	}
	pub fn apply_rotational_impulse(&mut self, body: PhysicsBodyId, impulse: Vec3) {
		self.map.entry(body).or_default().push(Impulse::RotationalImpulse { rotational_impluse: impulse });
	}
	pub fn apply_impulse(&mut self, body: PhysicsBodyId, pos: Vec3, impulse: Vec3) {
		self.map.entry(body).or_default().push(Impulse::Impulse { impluse: impulse, impluse_pos: pos });
	}
}

#[derive(Default)]
pub struct SolverPlugin;

impl Plugin for SolverPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<PhysicsSolver>()
			.init_resource::<BallJointConstraints>()
			.init_resource::<Impulses>()
			.add_systems(
				FixedUpdate,
				solve_physics
					.in_set(PhysicsSet::Step)
					.run_if(|freeze: Res<FreezePhysics>| !freeze.0),
			);
	}
}

#[allow(clippy::type_complexity)]
fn solve_physics(
	time: Res<Time>,
	mut solver: ResMut<PhysicsSolver>,
	mut constraints: ResMut<BallJointConstraints>,
	mut impulses: ResMut<Impulses>,
	collisions: Res<Collisions>,
	mut bodies: Query<(
		Entity,
		&mut Transform,
		&mut Velocity,
		&mut AngularVelocity,
		&Mass,
		&RotationalInertia,
		&CenterOfMass,
		Has<IsStatic>,
	), (With<RigidBody>, Without<Grid>)>,
) {
	let dt = time.delta_secs();
	if dt <= 0.0 { return; }

	let mut solver_bodies: SparseSet<PhysicsBodyId, SolverBody> = SparseSet::with_capacity(bodies.iter().count());
	for (entity, transform, velocity, angular_velocity, mass, inertia, com, is_static) in bodies.iter() {
		let mut body = SolverBody::new(entity);
		body.transform = *transform;
		body.velocity = velocity.0;
		body.angular_velocity = angular_velocity.0;
		body.mass = mass.0;
		body.rotational_inertia = inertia.0;
		body.center_of_mass = com.0;
		body.is_static = is_static || mass.0.is_zero();
		solver_bodies.insert(entity, body);
	}

	solver.0.solve(&mut solver_bodies, &collisions.0, &mut constraints.map, &impulses.map, dt);
	impulses.map.clear();

	for (entity, mut transform, mut velocity, mut angular_velocity, _, _, _, _) in bodies.iter_mut() {
		let Some(body) = solver_bodies.get(&entity) else { continue };
		*transform = body.transform;
		velocity.0 = body.velocity;
		angular_velocity.0 = body.angular_velocity;
	}
}
