use bevy::prelude::*;
use voxel_sources::{VoxelSourcesRequestHandle, VoxelSourcesRequestHandleGetter};

#[derive(Resource)]
pub struct StreamingSourceRequestHandle(pub(crate) VoxelSourcesRequestHandle);

impl FromWorld for StreamingSourceRequestHandle {
	fn from_world(world: &mut World) -> Self {
		Self(world.resource::<VoxelSourcesRequestHandleGetter>().get())
	}
}

impl StreamingSourceRequestHandle {
	pub fn requests(&self) -> &VoxelSourcesRequestHandle { &self.0 }
}
