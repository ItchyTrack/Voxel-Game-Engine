use bevy::prelude::*;
use lightyear::prelude::{DisableReplicateHierarchy, ReplicationSystems};
use tile_data::LoadedTile;

pub struct ReplicationPlugin;

impl Plugin for ReplicationPlugin {
	fn build(&self, app: &mut App) {
		app.add_systems(PostUpdate, disable_hierarchy_replication.before(ReplicationSystems::Send));
	}
}

fn disable_hierarchy_replication(
	mut commands: Commands,
	entities: Query<Entity, (With<LoadedTile>, Without<DisableReplicateHierarchy>)>,
) {
	for entity in &entities {
		commands.entity(entity).insert(DisableReplicateHierarchy);
	}
}
