mod active_collision;
#[path = "collision/exact/narrowphase.rs"]
mod narrowphase;

pub mod collision;
pub mod components;
pub mod constraints;
pub mod inertia_tensor;
pub mod integration;
pub mod math;
mod mass_aggregation;
pub mod solving;
pub mod sparse_set;
pub mod transform_ext;

use bevy::prelude::*;

pub use collision::{Collision, Collisions, CubeFeature, HalfCollision, PhysicsConsumer};
pub use components::{AngularVelocity, CenterOfMass, IsStatic, Mass, RigidBody, RotationalInertia, Velocity};
pub use constraints::BallJoint;
pub use inertia_tensor::InertiaTensor;
pub use integration::PhysicsIntegratedCenterOfMassTransform;
pub use solving::{Accelerations, Impulses};
pub use voxel_data::grid::GridId;

pub type PhysicsBodyId = Entity;

/// When set to `true`, the solver is skipped each tick.
#[derive(Resource, Debug, Clone, Copy)]
pub struct FreezePhysics(pub bool);

impl Default for FreezePhysics {
	fn default() -> Self { Self(true) }
}

/// Whether every dynamic body's voxel mass is precise enough for physics.
#[derive(Resource, Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct PhysicsMassReady(pub bool);

#[derive(Resource, Debug, Clone, Copy, PartialEq, Eq)]
pub struct PhysicsSimulationEnabled(pub bool);

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

pub fn physics_not_frozen(freeze: Res<FreezePhysics>) -> bool { !freeze.0 }

pub fn physics_mass_ready(ready: Res<PhysicsMassReady>) -> bool { ready.0 }

pub fn physics_simulation_enabled(enabled: Res<PhysicsSimulationEnabled>) -> bool { enabled.0 }

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

pub struct VoxelPhysicsPlugin {
	pub simulation_enabled: bool,
}

impl Default for VoxelPhysicsPlugin {
	fn default() -> Self { Self { simulation_enabled: true } }
}

impl Plugin for VoxelPhysicsPlugin {
	fn build(&self, app: &mut App) {
		if !app.is_plugin_added::<voxel_query::VoxelQueryPlugin>() {
			app.add_plugins(voxel_query::VoxelQueryPlugin);
		}
		if !app.is_plugin_added::<voxel_mass::VoxelMassPlugin>() {
			app.add_plugins(voxel_mass::VoxelMassPlugin::default());
		}
		app.init_resource::<FreezePhysics>()
			.init_resource::<PhysicsMassReady>()
			.insert_resource(PhysicsSimulationEnabled(self.simulation_enabled))
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
					.after(voxel_mass::VoxelMassSet::BodyAggregation)
					.run_if(physics_simulation_enabled)
					.run_if(collision::chunk_requests::collision_tiles_ready)
					.run_if(physics_not_frozen)
					.run_if(physics_mass_ready),
			)
			.add_plugins((
				collision::exact::ExactPlugin,
				integration::semi_implicit_euler::SemiImplicitEulerPlugin,
				solving::avbd::AvbdPlugin,
			))
			.add_systems(Startup, |mut commands: Commands| { commands.spawn(PhysicsConsumer::default()); })
			.add_systems(
				FixedUpdate,
				mass_aggregation::aggregate_body_mass_properties
					.in_set(voxel_mass::VoxelMassSet::BodyAggregation)
					.after(voxel_mass::VoxelMassSet::ApplySourceChanges),
			)
			.add_systems(
				FixedUpdate,
				(
					collision::chunk_requests::remove_stale_collision_requests,
					collision::chunk_requests::mark_edited_collision_tiles_pending,
					collision::chunk_requests::cache_presence_aabb,
					collision::chunk_requests::request_collision_chunks,
				)
					.chain()
					.run_if(physics_simulation_enabled)
					.before(PhysicsSet::Collision),
			)
			.add_systems(
				FixedUpdate,
				voxel_streaming::run_streaming
					.run_if(physics_simulation_enabled)
					.after(collision::chunk_requests::request_collision_chunks)
					.before(collision::chunk_requests::receive_collision_tiles),
			)
			.add_systems(
				FixedUpdate,
				collision::chunk_requests::receive_collision_tiles
					.run_if(physics_simulation_enabled)
					.before(PhysicsSet::Apply),
			);
	}
}
