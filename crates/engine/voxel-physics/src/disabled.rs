use bevy::prelude::*;

pub mod components {
	use bevy::prelude::*;

	use voxel_mass::{CenterOfMass, Mass, RotationalInertia};

	#[derive(Component, Default, Debug, Clone, Copy)]
	pub struct VoxelCollider;

	#[derive(Component, Default, Debug, Clone, Copy)]
	#[require(Transform, Velocity, AngularVelocity, Mass, RotationalInertia, CenterOfMass)]
	pub struct RigidBody;

	#[derive(Component, Default, Debug, Clone, Copy)]
	pub struct Velocity(pub Vec3);

	#[derive(Component, Default, Debug, Clone, Copy)]
	pub struct AngularVelocity(pub Vec3);

	#[derive(Component, Default, Debug, Clone, Copy)]
	pub struct IsStatic;
}

pub use components::{AngularVelocity, IsStatic, RigidBody, Velocity};

pub type PhysicsBodyId = Entity;

#[derive(Component, Debug, Clone, Copy)]
pub struct BallJoint {
	pub body_1: Entity,
	pub body_2: Entity,
	pub body_1_attachment: Transform,
	pub body_2_attachment: Transform,
	pub stiffness_linear: f32,
	pub stiffness_angular: f32,
}

impl BallJoint {
	pub fn new(
		body_1: Entity,
		body_2: Entity,
		body_1_attachment: &Transform,
		body_2_attachment: &Transform,
		stiffness_linear: f32,
		stiffness_angular: f32,
	) -> Self {
		Self {
			body_1,
			body_2,
			body_1_attachment: *body_1_attachment,
			body_2_attachment: *body_2_attachment,
			stiffness_linear,
			stiffness_angular,
		}
	}
}

#[derive(Resource, Debug, Clone, Copy)]
pub struct FreezePhysics(pub bool);

impl Default for FreezePhysics {
	fn default() -> Self { Self(true) }
}

#[derive(Resource, Default)]
pub struct Impulses;

impl Impulses {
	pub fn apply_central_impulse(&mut self, _body: PhysicsBodyId, _impulse: Vec3) {}

	pub fn apply_rotational_impulse(&mut self, _body: PhysicsBodyId, _impulse: Vec3) {}

	pub fn apply_impulse(&mut self, _body: PhysicsBodyId, _pos: Vec3, _impulse: Vec3) {}
}

#[derive(Resource, Default)]
pub struct Accelerations;

impl Accelerations {
	pub fn apply_central_acceleration(&mut self, _body: PhysicsBodyId, _acceleration: Vec3) {}
}

#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PhysicsSet {
	Apply,
	Collision,
	Integration,
	Solving,
}

fn physics_not_frozen(freeze: Res<FreezePhysics>) -> bool { !freeze.0 }

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

impl VoxelPhysicsAppExt for App {
	fn add_physics_apply_systems<M>(
		&mut self,
		systems: impl bevy::ecs::schedule::IntoScheduleConfigs<bevy::ecs::system::ScheduleSystem, M>,
	) -> &mut Self {
		self.add_systems(FixedUpdate, systems.in_set(PhysicsSet::Apply))
	}

	fn add_physics_collision_systems<M>(
		&mut self,
		systems: impl bevy::ecs::schedule::IntoScheduleConfigs<bevy::ecs::system::ScheduleSystem, M>,
	) -> &mut Self {
		self.add_systems(FixedUpdate, systems.in_set(PhysicsSet::Collision))
	}

	fn add_physics_integration_systems<M>(
		&mut self,
		systems: impl bevy::ecs::schedule::IntoScheduleConfigs<bevy::ecs::system::ScheduleSystem, M>,
	) -> &mut Self {
		self.add_systems(FixedUpdate, systems.in_set(PhysicsSet::Integration))
	}

	fn add_physics_solving_systems<M>(
		&mut self,
		systems: impl bevy::ecs::schedule::IntoScheduleConfigs<bevy::ecs::system::ScheduleSystem, M>,
	) -> &mut Self {
		self.add_systems(FixedUpdate, systems.in_set(PhysicsSet::Solving))
	}
}

#[derive(Default)]
pub struct VoxelPhysicsPlugin;

impl Plugin for VoxelPhysicsPlugin {
	fn build(&self, app: &mut App) {
		if !app.is_plugin_added::<voxel_query::VoxelQueryPlugin>() {
			app.add_plugins(voxel_query::VoxelQueryPlugin);
		}
		app.init_resource::<FreezePhysics>()
			.init_resource::<Impulses>()
			.init_resource::<Accelerations>()
			.init_resource::<crate::Collisions>()
			.configure_sets(
				FixedUpdate,
				(
					PhysicsSet::Apply,
					PhysicsSet::Collision,
					PhysicsSet::Integration,
					PhysicsSet::Solving,
				)
					.chain()
					.run_if(physics_not_frozen),
			)
			.add_systems(FixedUpdate, crate::active_collision::detect_collisions.in_set(PhysicsSet::Collision));
	}
}
