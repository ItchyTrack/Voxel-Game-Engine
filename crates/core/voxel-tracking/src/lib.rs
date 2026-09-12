mod types;

use bevy::prelude::*;
pub use types::{TrackedVoxelId, GlobalTrackedVoxels, GridTrackedVoxels};

fn update_tracking(

) {

}

#[derive(Default)]
pub struct VoxelTrackingPlugin;

impl Plugin for VoxelTrackingPlugin {
	fn build(&self, app: &mut App) {
		// app.add_systems(schedule, systems);
	}
}
