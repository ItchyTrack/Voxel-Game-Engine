use bevy::prelude::*;
use lightyear::prelude::*;
use lightyear::prelude::server::{NetcodeServer, ServerPlugins, ServerUdpIo, Start};

use crate::networking::network_common::{NetworkGrid, NetworkProtocolPlugin, NetworkTransform, PRIVATE_KEY, PROTOCOL_ID, REPLICATION_INTERVAL, SERVER_ADDR, replicate_grid_bundle};

pub struct NetworkServerPlugin;

impl Plugin for NetworkServerPlugin {
	fn build(&self, app: &mut App) {
		app.add_plugins(ServerPlugins::default())
			.add_plugins(NetworkProtocolPlugin)
			.add_observer(configure_connected_client)
			.add_systems(Startup, start_server)
			.add_systems(Update, (replicate_new_grids, sync_grid_transforms));
	}
}

fn start_server(mut commands: Commands) {
	let server = commands.spawn((
		Name::new("Lightyear Server"),
		ServerUdpIo::default(),
		LocalAddr(SERVER_ADDR.parse().expect("valid server addr")),
		NetcodeServer::new(
			lightyear::prelude::server::NetcodeConfig::default()
				.with_protocol_id(PROTOCOL_ID)
				.with_key(PRIVATE_KEY)
				.with_client_timeout_secs(30),
		),
	)).id();
	commands.trigger(Start { entity: server });
}

fn configure_connected_client(trigger: On<Add, LinkOf>, mut commands: Commands) {
	commands.entity(trigger.entity).insert((
		ReplicationSender::new(REPLICATION_INTERVAL, SendUpdatesMode::SinceLastAck, false),
		Name::new("Lightyear Client Connection"),
	));
}

fn replicate_new_grids(
	mut commands: Commands,
	grids: Query<(Entity, &Transform), (With<voxel_data::grid::Grid>, Without<NetworkGrid>)>,
) {
	for (entity, transform) in &grids {
		commands.entity(entity).insert((NetworkGrid, replicate_grid_bundle(), NetworkTransform::from(*transform)));
	}
}

fn sync_grid_transforms(
	mut commands: Commands,
	grids: Query<(Entity, &Transform, &NetworkTransform), (With<NetworkGrid>, With<Replicate>)>,
) {
	for (entity, transform, network_transform) in &grids {
		let latest = NetworkTransform::from(*transform);
		if latest.translation != network_transform.translation || latest.rotation != network_transform.rotation || latest.scale != network_transform.scale {
			commands.entity(entity).insert(latest);
		}
	}
}
