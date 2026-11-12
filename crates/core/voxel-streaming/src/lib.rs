use bevy::prelude::*;
use bevy_trait_query::RegisterExt;
use bile_data::{BileDataSchedule, DataConsumingNode};
use bile_tiles::{BileTilesSchedule, BileTilesSystemSet, TileInstances, TileKey};
use voxel_data::grid::Grid;
use voxel_sources::VoxelPresence;

mod presence;
mod tile_node;

pub use presence::{ChunkPresenceNode, RequestChunkPresence, bounds_of_chunks};
pub use tile_node::{TileBuilderAppExt, TileComponentData, TileParams, VoxelTileBuilder, VoxelTileNode};

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum VoxelTileSourceSet {
	Ingest,
	Receive,
}

/// Runs the Bile tile and data schedules outside of the main schedule order.
pub fn run_tile_schedule(world: &mut World) {
	world.run_schedule(BileTilesSchedule);
	world.run_schedule(BileDataSchedule);
}

fn place_tile(add: On<Add<TileKey>>, keys: Query<&TileKey>, mut commands: Commands) {
	let Ok(key) = keys.get(add.entity) else { return };
	commands.entity(add.entity).insert(Transform::from_translation(key.min().as_vec3()));
}

#[derive(Default)]
pub struct VoxelStreamingPlugin;

impl Plugin for VoxelStreamingPlugin {
	fn build(&self, app: &mut App) {
		if !app.is_plugin_added::<voxel_sources::VoxelSourcesPlugin>() {
			app.add_plugins(voxel_sources::VoxelSourcesPlugin);
		}
		if !app.is_plugin_added::<bile_tiles::TileDataPlugin>() {
			app.add_plugins(bile_tiles::TileDataPlugin);
		}
		app.world_mut().spawn((Name::new("Chunk Presence Node"), ChunkPresenceNode::default()));
		app.register_component_as::<dyn DataConsumingNode<VoxelPresence>, ChunkPresenceNode>();
		app.register_required_components::<Grid, TileInstances>();
		app.add_message::<bile_components::ComponentResolved>()
			.add_message::<bile_region_loader::RegionUpdated>()
			.configure_sets(PreUpdate, VoxelTileSourceSet::Ingest.after(voxel_sources::VoxelSourcesSet))
			.configure_sets(BileTilesSchedule, VoxelTileSourceSet::Receive.before(BileTilesSystemSet::UpdateReadyClasses))
			.add_systems(PreUpdate, presence::update_presence.in_set(VoxelTileSourceSet::Ingest))
			.add_systems(BileTilesSchedule, presence::publish_bounds_updates.in_set(VoxelTileSourceSet::Receive))
			.add_observer(place_tile)
			.add_systems(
				PostUpdate,
				bevy::transform::systems::propagate_transforms_for::<Added<TileKey>>
					.after(bevy::transform::TransformSystems::Propagate),
			);
	}
}
