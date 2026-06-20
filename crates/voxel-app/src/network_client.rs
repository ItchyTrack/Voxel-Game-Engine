use bevy::prelude::*;
use lightyear::prelude::*;
use lightyear::prelude::client::{ClientPlugins, Connect, NetcodeClient};
use voxel_data::grid::Grid;
use voxel_edit::GridEdits;
use voxel_streaming::{GridStreaming, RequestChunkPresence};

use crate::network_common::{CLIENT_ADDR, NetworkGrid, NetworkProtocolPlugin, NetworkTransform, PRIVATE_KEY, PROTOCOL_ID, SERVER_ADDR};
use crate::SelectedClientId;

pub struct NetworkClientPlugin;

impl Plugin for NetworkClientPlugin {
	fn build(&self, app: &mut App) {
		app.add_plugins(ClientPlugins::default())
			.add_plugins(NetworkProtocolPlugin)
			.add_systems(Startup, start_client)
			.add_systems(Update, (init_replicated_grids, apply_network_transforms));
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

fn init_replicated_grids(
	mut commands: Commands,
	grids: Query<(Entity, Option<&NetworkTransform>), (With<NetworkGrid>, With<Replicated>, Without<Grid>)>,
) {
	for (entity, network_transform) in &grids {
		commands.entity(entity).insert((
			Grid::new(),
			GridStreaming::default(),
			GridEdits::default(),
			RequestChunkPresence::default(),
			network_transform.copied().map(Transform::from).unwrap_or_default(),
		));
	}
}

fn apply_network_transforms(
	mut grids: Query<(&NetworkTransform, &mut Transform), (With<NetworkGrid>, With<Replicated>)>,
) {
	for (network_transform, mut transform) in &mut grids {
		*transform = Transform::from(*network_transform);
	}
}
