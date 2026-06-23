use std::time::Duration;

use bevy::prelude::*;
use lightyear::prelude::*;
use serde::{Deserialize, Serialize};
use voxel_lightyear::ReplicateVoxels;

#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct NetworkBody;

#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct NetworkGrid;

pub const PROTOCOL_ID: u64 = 0x564F_5845_4C47_414D;
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
		app.register_component::<NetworkBody>();
		app.register_component::<NetworkGrid>();
		app.register_component::<NetworkTransform>();
		app.register_component::<ReplicateVoxels>();
	}
}

pub fn replicate_grid_bundle() -> Replicate {
	Replicate::to_clients(NetworkTarget::All)
}
