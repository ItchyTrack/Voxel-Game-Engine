pub mod ball_joint_constraint;
pub mod collision;
pub mod collision_constraint;
pub mod components;
pub mod inertia_tensor;
pub mod math;
pub mod physics_body;
pub mod physics_constraint;
pub mod solver;
pub mod sparse_set;

use std::collections::HashMap;

use bevy::math::DVec3;
use bevy::prelude::*;

use voxel_data::bvh::bvh::BVH;
use voxel_data::grid::Grid;
use voxel_data::transform_ext;

pub use ball_joint_constraint::BallJointConstraint;
pub use components::{AngularVelocity, CenterOfMass, ComputeMassProperties, IsStatic, Mass, RigidBody, RotationalInertia, Velocity};
pub use inertia_tensor::InertiaTensor;
pub use physics_body::{GridId, PhysicsBody, PhysicsBodyId};
use crate::solver::{Impulse, Solver};
use crate::sparse_set::SparseSet;

#[derive(Resource, Debug, Clone, Copy)]
pub struct Gravity(pub Vec3);

impl Default for Gravity {
	fn default() -> Self { Self(Vec3::new(0.0, -98.0, 0.0)) }
}

/// When set to `true`, [`step_physics`] is skipped each tick.
#[derive(Resource, Debug, Default, Clone, Copy)]
pub struct FreezePhysics(pub bool);

/// Cross-frame solver state (warm-started K/L values for collision constraints).
#[derive(Resource)]
struct PhysicsSolver(Solver);

impl Default for PhysicsSolver {
	fn default() -> Self { Self(Solver::new()) }
}

/// Ball-joint constraints between rigid body entities. Keyed by the ordered
/// pair of [`PhysicsBodyId`]s (lower id first).
#[derive(Resource, Default)]
pub struct BallJointConstraints {
	map: HashMap<(PhysicsBodyId, PhysicsBodyId), BallJointConstraint>,
}

impl BallJointConstraints {
	pub fn insert(&mut self, a: PhysicsBodyId, b: PhysicsBodyId, constraint: BallJointConstraint) {
		let key = if a.0 < b.0 { (a, b) } else { (b, a) };
		self.map.insert(key, constraint);
	}
	pub fn remove(&mut self, a: PhysicsBodyId, b: PhysicsBodyId) {
		let key = if a.0 < b.0 { (a, b) } else { (b, a) };
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

/// System sets used by [`VoxelPhysicsPlugin`] inside `FixedUpdate`.
///
/// Game systems that want to push impulses, drag held bodies, etc. should run
/// in [`PhysicsSet::Apply`] so their effects are picked up by the solver in the
/// same step.
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PhysicsSet {
	/// Queue impulses, hold-targets, and constraint edits before the solver runs.
	Apply,
	/// The solver runs in this set.
	Step,
}

#[derive(Default)]
pub struct VoxelPhysicsPlugin;

impl Plugin for VoxelPhysicsPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<Gravity>()
			.init_resource::<PhysicsSolver>()
			.init_resource::<BallJointConstraints>()
			.init_resource::<Impulses>()
			.init_resource::<FreezePhysics>()
			.configure_sets(FixedUpdate, (PhysicsSet::Apply, PhysicsSet::Step).chain())
			.add_systems(FixedUpdate, compute_mass_properties.before(PhysicsSet::Step))
			.add_systems(
				FixedUpdate,
				step_physics
					.in_set(PhysicsSet::Step)
					.run_if(|freeze: Res<FreezePhysics>| !freeze.0),
			);
	}
}

/// Derives `Mass`, `CenterOfMass`, and `RotationalInertia` from the voxel masses
/// of each child `Grid`. Runs for any body tagged with [`ComputeMassProperties`]
/// and removes the marker once the body is initialized.
fn compute_mass_properties(
	mut commands: Commands,
	mut bodies: Query<(Entity, &Children, &mut Mass, &mut CenterOfMass, &mut RotationalInertia), (With<RigidBody>, With<ComputeMassProperties>)>,
	grids: Query<&Grid>,
) {
	for (entity, children, mut mass, mut com, mut inertia) in bodies.iter_mut() {
		let mut total_mass: f64 = 0.0;
		let mut com_times_mass = DVec3::ZERO;
		let mut tensor_at_origin = InertiaTensor::ZERO;

		for child in children.iter() {
			let Ok(grid) = grids.get(child) else { continue };
			let grid_pose = grid.transform();
			for (_, sub_grid) in grid.sub_grids().iter() {
				let palette = sub_grid.get_voxels().get_palette();
				let sub_pos = sub_grid.sub_grid_pos().as_dvec3();
				for (voxel_pos, count, palette_id) in sub_grid.get_voxels().get_voxels().iter() {
					let Some(voxel) = palette.get_voxel(palette_id) else { continue };
					let m = voxel.mass as f64;
					let base = sub_pos + voxel_pos.as_dvec3();
					let run = count as i32;
					for dx in 0..run { for dy in 0..run { for dz in 0..run {
						let local_pos = base + DVec3::new(dx as f64 + 0.5, dy as f64 + 0.5, dz as f64 + 0.5);
						// Map from grid-local to body-local through the grid's own transform.
						let body_pos = (*grid_pose * local_pos.as_vec3()).as_dvec3();
						total_mass += m;
						com_times_mass += body_pos * m;
						tensor_at_origin += InertiaTensor::get_inertia_tensor_for_cube_at_pos(m, 1.0, &body_pos);
					}}}
				}
			}
		}

		commands.entity(entity).remove::<ComputeMassProperties>();
		if total_mass <= 0.0 { continue; }
		let com_vec = com_times_mass / total_mass;
		mass.0 = total_mass as f32;
		com.0 = com_vec.as_vec3();
		inertia.0 = tensor_at_origin.move_to_center_of_mass(&com_vec, total_mass);
	}
}

#[allow(clippy::type_complexity)]
fn step_physics(
	time: Res<Time>,
	mut solver: ResMut<PhysicsSolver>,
	mut constraints: ResMut<BallJointConstraints>,
	mut impulses: ResMut<Impulses>,
	mut bodies: Query<(
		Entity,
		&mut Transform,
		&mut Velocity,
		&mut AngularVelocity,
		&Mass,
		&RotationalInertia,
		&CenterOfMass,
		Has<IsStatic>,
	), With<RigidBody>>,
	grid_entities: Query<(Entity, &Grid, &ChildOf)>,
) {
	let dt = time.delta_secs();
	if dt <= 0.0 { return; }

	// Collect rigid body state into the per-step SparseSet the solver expects.
	let mut physics_bodies: SparseSet<PhysicsBodyId, PhysicsBody> = SparseSet::with_capacity(bodies.iter().count());
	for (entity, transform, velocity, angular_velocity, mass, inertia, com, is_static) in bodies.iter() {
		let id = PhysicsBodyId(entity);
		let mut body = PhysicsBody::new(id);
		body.transform = *transform;
		body.velocity = velocity.0;
		body.angular_velocity = angular_velocity.0;
		body.mass = mass.0;
		body.rotational_inertia = inertia.0;
		body.center_of_mass = com.0;
		body.is_static = is_static;
		physics_bodies.insert(id, body);
	}

	// Attach grids to their parent body. A grid is any child entity carrying a
	// `Grid` component whose parent is a `RigidBody`.
	let mut grids: SparseSet<GridId, &Grid> = SparseSet::with_capacity(grid_entities.iter().count());
	for (grid_entity, grid, child_of) in grid_entities.iter() {
		let parent_id = PhysicsBodyId(child_of.parent());
		let Some(body) = physics_bodies.get_mut(&parent_id) else { continue };
		let grid_id = GridId(grid_entity);
		body.push_grid(grid_id);
		grids.insert(grid_id, grid);
	}

	// Build a broadphase BVH over every sub-grid AABB.
	let mut bounds = Vec::new();
	for (body_id, body) in physics_bodies.iter() {
		for grid_id in body.grids() {
			let grid = *grids.get(grid_id).unwrap();
			let grid_transform = body.transform * *grid.transform();
			for (sub_grid_id, sub_grid) in grid.sub_grids().iter() {
				let sub_grid_transform = grid_transform * Transform::from_translation(sub_grid.sub_grid_pos().as_vec3());
				if let Some(aabb) = sub_grid.aabb(&sub_grid_transform) {
					bounds.push(((*body_id, *grid_id, *sub_grid_id), aabb));
				}
			}
		}
	}
	let bvh = BVH::new(bounds);

	solver.0.solve(&mut physics_bodies, &grids, &mut constraints.map, &impulses.map, dt, &bvh);
	impulses.map.clear();

	// Write the solver's results back into the ECS.
	for (entity, mut transform, mut velocity, mut angular_velocity, _, _, _, _) in bodies.iter_mut() {
		let id = PhysicsBodyId(entity);
		let Some(body) = physics_bodies.get(&id) else { continue };
		*transform = body.transform;
		velocity.0 = body.velocity;
		angular_velocity.0 = body.angular_velocity;
	}
}
