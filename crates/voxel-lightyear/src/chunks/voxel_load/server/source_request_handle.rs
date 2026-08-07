use bevy::prelude::*;
use voxel_sources::{VoxelSourcesRequestHandle, VoxelSourcesRequestHandleGetter};

#[derive(Resource)]
pub(crate) struct LightyearSourceRequestHandle(VoxelSourcesRequestHandle);

impl FromWorld for LightyearSourceRequestHandle {
	fn from_world(world: &mut World) -> Self {
		Self(world.resource::<VoxelSourcesRequestHandleGetter>().get())
	}
}

impl LightyearSourceRequestHandle {
	pub(crate) fn requests(&self) -> &VoxelSourcesRequestHandle { &self.0 }
}
