use std::time::Duration;

use bevy::prelude::*;
use lightyear::prelude::*;
use serde::{Deserialize, Serialize};
use voxel_lightyear::ReplicateVoxels;
use voxel_mass::BodyMassError;
use voxel_physics::{CenterOfMass, IsStatic, Mass, RotationalInertia, components::VoxelCollider};

#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct NetworkBody;

#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct NetworkGrid;

pub const PROTOCOL_ID: u64 = 0x564F_5845_4C4D_4153;
pub const PRIVATE_KEY: [u8; 32] = [7; 32];
pub const SERVER_ADDR: &str = "127.0.0.1:5000";
pub const CLIENT_ADDR: &str = "127.0.0.1:0";
pub const REPLICATION_INTERVAL: Duration = Duration::from_millis(100);

#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct NetworkTransform {
	pub translation: Vec3,
	pub rotation: Quat,
	pub scale: Vec3,
}

impl From<Transform> for NetworkTransform {
	fn from(value: Transform) -> Self {
		Self { translation: value.translation, rotation: value.rotation, scale: value.scale }
	}
}

impl From<NetworkTransform> for Transform {
	fn from(value: NetworkTransform) -> Self {
		Self { translation: value.translation, rotation: value.rotation, scale: value.scale }
	}
}

pub struct NetworkProtocolPlugin;

impl Plugin for NetworkProtocolPlugin {
	fn build(&self, app: &mut App) {
		app.component::<NetworkBody>().replicate();
		app.component::<NetworkGrid>().replicate();
		app.component::<NetworkTransform>().replicate();
		app.component::<ReplicateVoxels>().replicate();
		app.component::<Mass>().replicate();
		app.component::<CenterOfMass>().replicate();
		app.component::<RotationalInertia>().replicate();
		app.component::<BodyMassError>().replicate();
		app.component::<IsStatic>().replicate();
		app.component::<VoxelCollider>().replicate();
	}
}

pub fn replicate_grid_bundle() -> Replicate {
	Replicate::to_clients(NetworkTarget::All)
}
