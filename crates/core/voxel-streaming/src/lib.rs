use bevy::prelude::*;
use bile_tiles::{BileTilesSchedule, BileTilesSystemSet, TileInstances, TileKey};
use voxel_data::grid::Grid;

mod edit_interest;
mod presence;
mod source;
mod tile_dependency_index;

pub use edit_interest::{ChunkEditInterest, ChunkEditInterestChanged};
pub use presence::{RequestChunkPresence, bounds_of_chunks};
pub use source::{TileBuilderAppExt, VoxelTileSource};

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum VoxelTileSourceSet {
	Ingest,
	Receive,
	Request,
}

pub fn run_tile_schedule(world: &mut World) {
	world.run_schedule(BileTilesSchedule);
}

fn place_tile(add: On<Add, TileKey>, keys: Query<&TileKey>, mut commands: Commands) {
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
		source::register_source(app);
		app.register_required_components::<Grid, TileInstances>();
		app.add_message::<ChunkEditInterestChanged>()
			.add_message::<bile_region_loader::RegionUpdated>()
			.configure_sets(PreUpdate, VoxelTileSourceSet::Ingest.after(voxel_sources::VoxelSourcesSet))
			.configure_sets(BileTilesSchedule, (
				VoxelTileSourceSet::Receive.before(BileTilesSystemSet::UpdateReadyClasses),
				VoxelTileSourceSet::Request.after(BileTilesSystemSet::ComponentRequest),
			))
			.add_systems(PreUpdate, (presence::request_presence, presence::receive_presence).chain().in_set(VoxelTileSourceSet::Ingest))
			.add_systems(BileTilesSchedule, (
				(
					presence::publish_bounds_updates,
					source::route_voxel_results,
					source::cancel_requests,
					source::update_edited_tiles,
					source::receive_tile_builds,
				).chain().in_set(VoxelTileSourceSet::Receive),
				(
					source::start_requested_builds,
					source::submit_voxel_requests,
					edit_interest::publish_edit_interest_changes,
				).chain().in_set(VoxelTileSourceSet::Request),
			))
			.add_observer(place_tile)
			.add_observer(source::release_removed_grid)
			.add_systems(
				PostUpdate,
				bevy::transform::systems::propagate_transforms_for::<Added<TileKey>>
					.after(bevy::transform::TransformSystems::Propagate),
			);
	}
}
