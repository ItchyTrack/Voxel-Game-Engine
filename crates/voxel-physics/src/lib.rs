pub mod constraints;
pub mod collision;
pub mod components;
pub mod inertia_tensor;
pub mod integration;
pub mod math;
pub mod solving;
pub mod sparse_set;

use bevy::math::DVec3;
use bevy::prelude::*;

use voxel_data::grid::Grid;

pub use constraints::BallJoint;
pub use collision::{Collisions, PhysicsConsumer};
pub use components::{AngularVelocity, CenterOfMass, IsStatic, Mass, RigidBody, RotationalInertia, Velocity};
pub use inertia_tensor::InertiaTensor;
pub use integration::PhysicsIntegratedCenterOfMassTransform;
pub use solving::{Accelerations, Impulses};
pub use voxel_data::grid::GridId;

use crate::components::VoxelMass;

pub type PhysicsBodyId = Entity;

/// When set to `true`, the solver is skipped each tick.
#[derive(Resource, Debug, Clone, Copy)]
pub struct FreezePhysics(pub bool);

impl Default for FreezePhysics {
	fn default() -> Self {
		Self(true)
	}
}

/// System sets used by [`VoxelPhysicsPlugin`] inside `FixedUpdate`, run in order.
///
/// Game systems that push impulses, drag held bodies, etc. should run in
/// [`PhysicsSet::Apply`] so their effects are picked up the same step.
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PhysicsSet {
	/// Queue impulses, hold-targets, and constraint edits before physics stages.
	Apply,
	/// Broadphase + narrowphase; fills the [`Collisions`] resource.
	Collision,
	/// Integrates body state into [`PhysicsIntegratedCenterOfMassTransform`].
	Integration,
	/// Consumes collisions and integrated transforms to solve body motion.
	Solving,
}

pub trait VoxelPhysicsAppExt {
	fn add_physics_apply_systems<M>(
		&mut self,
		systems: impl bevy::ecs::schedule::IntoScheduleConfigs<bevy::ecs::system::ScheduleSystem, M>,
	) -> &mut Self;
	fn add_physics_collision_systems<M>(
		&mut self,
		systems: impl bevy::ecs::schedule::IntoScheduleConfigs<bevy::ecs::system::ScheduleSystem, M>,
	) -> &mut Self;
	fn add_physics_integration_systems<M>(
		&mut self,
		systems: impl bevy::ecs::schedule::IntoScheduleConfigs<bevy::ecs::system::ScheduleSystem, M>,
	) -> &mut Self;
	fn add_physics_solving_systems<M>(
		&mut self,
		systems: impl bevy::ecs::schedule::IntoScheduleConfigs<bevy::ecs::system::ScheduleSystem, M>,
	) -> &mut Self;
}

pub fn physics_not_frozen(freeze: Res<FreezePhysics>) -> bool {
	!freeze.0
}

impl VoxelPhysicsAppExt for App {
	fn add_physics_apply_systems<M>(
		&mut self,
		systems: impl bevy::ecs::schedule::IntoScheduleConfigs<bevy::ecs::system::ScheduleSystem, M>,
	) -> &mut Self {
		self.add_systems(FixedUpdate, systems.in_set(PhysicsSet::Apply));
		self
	}

	fn add_physics_collision_systems<M>(
		&mut self,
		systems: impl bevy::ecs::schedule::IntoScheduleConfigs<bevy::ecs::system::ScheduleSystem, M>,
	) -> &mut Self {
		self.add_systems(FixedUpdate, systems.in_set(PhysicsSet::Collision));
		self
	}

	fn add_physics_integration_systems<M>(
		&mut self,
		systems: impl bevy::ecs::schedule::IntoScheduleConfigs<bevy::ecs::system::ScheduleSystem, M>,
	) -> &mut Self {
		self.add_systems(FixedUpdate, systems.in_set(PhysicsSet::Integration));
		self
	}

	fn add_physics_solving_systems<M>(
		&mut self,
		systems: impl bevy::ecs::schedule::IntoScheduleConfigs<bevy::ecs::system::ScheduleSystem, M>,
	) -> &mut Self {
		self.add_systems(FixedUpdate, systems.in_set(PhysicsSet::Solving));
		self
	}
}

#[derive(Default)]
pub struct VoxelPhysicsPlugin;

impl Plugin for VoxelPhysicsPlugin {
	fn build(&self, app: &mut App) {
		use voxel_streaming::VoxelStreamingAppExt;
		app.init_resource::<FreezePhysics>()
			.insert_resource(Time::<Fixed>::from_hz(120.0))
			.configure_sets(
				FixedUpdate,
				(
					PhysicsSet::Apply,
					PhysicsSet::Collision,
					PhysicsSet::Integration,
					PhysicsSet::Solving,
				)
					.chain()
					.run_if(voxel_streaming::chunks_ready::<PhysicsConsumer>)
					.run_if(physics_not_frozen),
			)
			.add_plugins((
				collision::exact::ExactPlugin,
				integration::semi_implicit_euler::SemiImplicitEulerPlugin,
				solving::avbd::AvbdPlugin,
			))
			.register_chunk_consumer::<PhysicsConsumer>()
			.add_systems(Startup, |mut commands: Commands| { commands.spawn(PhysicsConsumer::default()); })
			.add_systems(
				FixedUpdate,
				(
					collision::chunk_requests::cache_presence_aabb,
					collision::chunk_requests::request_collision_chunks,
				)
					.chain()
					.before(PhysicsSet::Collision),
			)
			.add_systems(FixedUpdate, compute_mass_properties.before(PhysicsSet::Collision))
			.add_systems(
				FixedUpdate,
				voxel_streaming::run_streaming
					.after(collision::chunk_requests::request_collision_chunks)
					.before(PhysicsSet::Apply),
			);
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
