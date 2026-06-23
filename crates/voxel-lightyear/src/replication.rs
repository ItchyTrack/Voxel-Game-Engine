use bevy::prelude::*;
use lightyear::prelude::{DisableReplicateHierarchy, ReplicationBufferSystems};
use voxel_data::subgrid::SubGrid;
use voxel_gpu::lod_voxels::LodVoxels;

pub struct ReplicationPlugin;

impl Plugin for ReplicationPlugin {
	fn build(&self, app: &mut App) {
		app.add_systems(PostUpdate, disable_hierarchy_replication.before(ReplicationBufferSystems::BeforeBuffer));
	}
}

fn disable_hierarchy_replication(
	mut commands: Commands,
	entities: Query<Entity, (Or<(With<SubGrid>, With<LodVoxels>)>, Without<DisableReplicateHierarchy>)>,
) {
	for entity in &entities {
		commands.entity(entity).insert(DisableReplicateHierarchy);
	}
}
