pub mod ball_joint_constraint;
pub mod collision;
pub mod collision_constraint;
pub mod components;
pub mod inertia_tensor;
pub mod math;
pub mod physics_constraint;
pub mod solver;
pub mod sparse_set;

use bevy::math::DVec3;
use bevy::prelude::*;

use voxel_data::grid::Grid;

pub use ball_joint_constraint::BallJointConstraint;
pub use collision::Collisions;
pub use components::{AngularVelocity, CenterOfMass, IsStatic, Mass, RigidBody, RotationalInertia, Velocity};
pub use inertia_tensor::InertiaTensor;
pub use solver::{BallJointConstraints, Impulses};
pub use voxel_data::grid::GridId;

use crate::components::VoxelMass;

pub type PhysicsBodyId = Entity;

#[derive(Resource, Debug, Clone, Copy)]
pub struct Gravity(pub Vec3);

impl Default for Gravity {
	fn default() -> Self { Self(Vec3::new(0.0, -98.0, 0.0)) }
}

/// When set to `true`, the solver is skipped each tick.
#[derive(Resource, Debug, Default, Clone, Copy)]
pub struct FreezePhysics(pub bool);

/// System sets used by [`VoxelPhysicsPlugin`] inside `FixedUpdate`, run in order.
///
/// Game systems that push impulses, drag held bodies, etc. should run in
/// [`PhysicsSet::Apply`] so their effects are picked up the same step.
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PhysicsSet {
	/// Queue impulses, hold-targets, and constraint edits before collision/solve.
	Apply,
	/// Broadphase + narrowphase; fills the [`Collisions`] resource.
	Detect,
	/// The solver runs in this set.
	Step,
}

#[derive(Default)]
pub struct VoxelPhysicsPlugin;

impl Plugin for VoxelPhysicsPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<Gravity>()
			.init_resource::<FreezePhysics>()
			.insert_resource(Time::<Fixed>::from_hz(120.0))
			.configure_sets(FixedUpdate, (PhysicsSet::Apply, PhysicsSet::Detect, PhysicsSet::Step).chain())
			.add_plugins((collision::CollisionPlugin, solver::SolverPlugin))
			.add_systems(FixedUpdate, compute_mass_properties.before(PhysicsSet::Detect));
	}
}

/// Derives `Mass`, `CenterOfMass`, and `RotationalInertia` from the voxel masses
/// of each child `Grid`. Recomputes a body whenever one of its child grids
/// changes (voxels added/removed), which also covers the initial population.
fn compute_mass_properties(
	mut bodies: Query<(&Children, &mut Mass, &mut CenterOfMass, &mut RotationalInertia), With<RigidBody>>,
	grids: Query<(&Transform, &Grid), (With<VoxelMass>, With<Grid>)>,
	changed_grids: Query<(), (With<VoxelMass>, With<Grid>, Changed<Grid>)>,
) {
	if changed_grids.is_empty() { return; }

	for (children, mut mass, mut com, mut inertia) in bodies.iter_mut() {
		if !children.iter().any(|child| changed_grids.contains(child)) { continue; }

		let mut total_mass: f64 = 0.0;
		let mut com_times_mass = DVec3::ZERO;
		let mut tensor_at_origin = InertiaTensor::ZERO;

		for child in children.iter() {
			let Ok((grid_pose, grid)) = grids.get(child) else { continue };
			for sub_grid in grid.subgrids() {
				let palette = sub_grid.voxels().palette();
				let sub_pos = sub_grid.sub_grid_pos().as_dvec3();
				for (voxel_pos, count, palette_id) in sub_grid.voxels().grid_tree().iter() {
					let Some(voxel) = palette.voxel(palette_id) else { continue };
					let m = voxel.mass as f64;
					let base = sub_pos + voxel_pos.as_dvec3();
					let run = count as i32;
					for dx in 0..run { for dy in 0..run { for dz in 0..run {
						let local_pos = base + DVec3::new(dx as f64 + 0.5, dy as f64 + 0.5, dz as f64 + 0.5);
						let body_pos = (*grid_pose * local_pos.as_vec3()).as_dvec3();
						total_mass += m;
						com_times_mass += body_pos * m;
						tensor_at_origin += InertiaTensor::get_inertia_tensor_for_cube_at_pos(m, 1.0, &body_pos);
					}}}
				}
			}
		}

		if total_mass <= 0.0 { continue; }
		let com_vec = com_times_mass / total_mass;
		mass.0 = total_mass as f32;
		com.0 = com_vec.as_vec3();
		inertia.0 = tensor_at_origin.move_to_center_of_mass(&com_vec, total_mass);
	}
}
