use bevy::prelude::*;
use lightyear::prelude::*;
use lightyear::prelude::client::{ClientPlugins, Connect, NetcodeClient};
use voxel_data::grid::Grid;
use voxel_edit::GridEdits;
use voxel_physics::{IsStatic, RigidBody};
use voxel_streaming::{GridStreaming, RequestChunkPresence};

use crate::networking::network_common::{CLIENT_ADDR, NetworkBody, NetworkGrid, NetworkProtocolPlugin, NetworkTransform, PRIVATE_KEY, PROTOCOL_ID, SERVER_ADDR};
use crate::SelectedClientId;

pub struct NetworkClientPlugin;

impl Plugin for NetworkClientPlugin {
	fn build(&self, app: &mut App) {
		app.add_plugins(ClientPlugins {
			tick_duration: std::time::Duration::from_secs_f64(1.0 / 120.0),
		})
			.add_plugins(NetworkProtocolPlugin)
			.add_systems(Startup, start_client)
			.add_systems(Update, (init_replicated_bodies, init_replicated_grids, apply_network_transforms, log_client_network_entities));
	}
}

fn start_client(mut commands: Commands, client_id: Res<SelectedClientId>) {
	let client_id = client_id.0;
	let client = commands.spawn((
		Name::new("Lightyear Client"),
		UdpIo::default(),
		LocalAddr(CLIENT_ADDR.parse().expect("valid client addr")),
		NetcodeClient::new(
			Authentication::Manual {
				server_addr: SERVER_ADDR.parse().expect("valid server addr"),
				client_id,
				private_key: PRIVATE_KEY,
				protocol_id: PROTOCOL_ID,
			},
			lightyear::prelude::client::NetcodeConfig {
				client_timeout_secs: 30,
				..Default::default()
			},
		)
		.expect("create netcode client"),
		ReplicationReceiver::default(),
	)).id();
	commands.trigger(LinkStart { entity: client });
	commands.trigger(Connect { entity: client });
}

fn init_replicated_bodies(
	mut commands: Commands,
	bodies: Query<(Entity, Option<&NetworkTransform>, Has<IsStatic>), (With<NetworkBody>, With<Replicated>, Without<RigidBody>)>,
) {
	for (entity, network_transform, is_static) in &bodies {
		let mut entity_commands = commands.entity(entity);
		entity_commands.insert((RigidBody, network_transform.copied().map(Transform::from).unwrap_or_default()));
		if is_static {
			entity_commands.insert(IsStatic);
		}
	}
}

fn init_replicated_grids(
	mut commands: Commands,
	grids: Query<(Entity, Option<&NetworkTransform>), (With<NetworkGrid>, With<Replicated>, Without<Grid>)>,
) {
	for (entity, network_transform) in &grids {
		commands.entity(entity).insert((
			Grid::new(),
			GridStreaming::default(),
			GridEdits::default(),
			RequestChunkPresence,
			network_transform.copied().map(Transform::from).unwrap_or_default(),
		));
	}
}

fn apply_network_transforms(
	mut entities: Query<(&NetworkTransform, &mut Transform), (Or<(With<NetworkBody>, With<NetworkGrid>)>, With<Replicated>)>,
) {
	for (network_transform, mut transform) in &mut entities {
		*transform = Transform::from(*network_transform);
	}
}

fn log_client_network_entities(
	bodies: Query<(Entity, Option<&ChildOf>), Added<NetworkBody>>,
	grids: Query<(Entity, Option<&ChildOf>), Added<NetworkGrid>>,
) {
	for (entity, parent) in &bodies {
		info!(?entity, parent = ?parent.map(|p| p.parent()), "client added network body");
	}
	for (entity, parent) in &grids {
		info!(?entity, parent = ?parent.map(|p| p.parent()), "client added network grid");
	}
}
