use bevy::prelude::*;
use num::Zero;
use voxel_data::grid::Grid;
use voxel_mass::{CenterOfMass, Mass, RotationalInertia};

use crate::collision::Collisions;
use crate::components::{AngularVelocity, IsStatic, RigidBody, Velocity};
use crate::constraints::BallJoint;
use crate::integration::PhysicsIntegratedCenterOfMassTransform;
use crate::sparse_set::SparseSet;
use crate::{PhysicsBodyId, VoxelPhysicsAppExt};

pub(crate) use ball_joint_constraint::AvbdBallJointConstraint;
pub(crate) use solver::Solver;
mod ball_joint_constraint;
mod body;
mod collision_constraint;
pub(crate) mod physics_constraint;
mod solver;

use self::body::SolverBody;

/// Cross-frame solver state (warm-started K/L values for collision constraints).
#[derive(Resource)]
struct PhysicsSolver(Solver);

impl Default for PhysicsSolver {
	fn default() -> Self { Self(Solver::new()) }
}

#[derive(Default)]
pub struct AvbdPlugin;

impl bevy::app::Plugin for AvbdPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<PhysicsSolver>()
			.init_resource::<super::Impulses>()
			.init_resource::<super::Accelerations>()
			.add_systems(FixedUpdate, sync_ball_joint_constraints.before(crate::PhysicsSet::Solving))
			.add_physics_solving_systems((solve_physics, clear_queued_impulses_and_accelerations).chain());
	}
}

fn sync_ball_joint_constraints(
	mut commands: Commands,
	joints: Query<(Entity, &BallJoint), Without<AvbdBallJointConstraint>>,
) {
	for (entity, joint) in joints.iter() {
		commands.entity(entity).insert(AvbdBallJointConstraint::from_ball_joint(joint));
	}
}

fn clear_queued_impulses_and_accelerations(
	mut impulses: ResMut<super::Impulses>,
	mut accelerations: ResMut<super::Accelerations>,
) {
	impulses.map.clear();
	accelerations.map.clear();
}

fn solve_physics(
	time: Res<Time>,
	mut solver: ResMut<PhysicsSolver>,
	collisions: Res<Collisions>,
	mut bodies: Query<(
		Entity,
		&mut Transform,
		&PhysicsIntegratedCenterOfMassTransform,
		&mut Velocity,
		&mut AngularVelocity,
		&Mass,
		&RotationalInertia,
		&CenterOfMass,
		Has<IsStatic>,
	), (With<RigidBody>, Without<Grid>)>,
	mut constraints: Query<(Entity, &BallJoint, &mut AvbdBallJointConstraint)>,
) {
	let dt = time.delta_secs();
	if dt <= 0.0 { return; }

	let mut solver_bodies: SparseSet<PhysicsBodyId, SolverBody> = SparseSet::with_capacity(bodies.iter().count());
	for (entity, transform, integrated_center_of_mass_transform, velocity, angular_velocity, mass, inertia, com, is_static) in bodies.iter() {
		let mut body = SolverBody::new();
		body.transform = *transform;
		body.integrated_center_of_mass_transform = integrated_center_of_mass_transform.0;
		body.velocity = velocity.0;
		body.angular_velocity = angular_velocity.0;
		body.mass = mass.0 as f32;
		body.rotational_inertia = inertia.0;
		body.center_of_mass = com.0.as_vec3();
		body.is_static = is_static || mass.0.is_zero();
		solver_bodies.insert(entity, body);
	}

	solver.0.solve(&mut solver_bodies, &collisions.0, &mut constraints, dt);

	for (entity, mut transform, _, mut velocity, mut angular_velocity, _, _, _, _) in bodies.iter_mut() {
		let Some(body) = solver_bodies.get(&entity) else { continue };
		*transform = body.transform;
		velocity.0 = body.velocity;
		angular_velocity.0 = body.angular_velocity;
	}
}
