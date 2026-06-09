use bevy::prelude::*;
use voxel_data::grid::GridId;

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct FreezeCameraLods(pub bool);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CameraLodDebugState {
	FullRes,
	Lod(u32),
	HeldLod(u32),
	WaitingOnLod,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CameraLodDebugChunk {
	pub grid: GridId,
	pub chunk: IVec3,
	pub size: IVec3,
	pub state: CameraLodDebugState,
}

#[derive(Component, Default, Debug, Clone)]
pub struct CameraLodDebug {
	pub chunks: Vec<CameraLodDebugChunk>,
}

impl CameraLodDebug {
	pub fn clear(&mut self) {
		self.chunks.clear();
	}

	pub fn push(&mut self, grid: GridId, chunk: IVec3, state: CameraLodDebugState) {
		self.push_area(grid, chunk, IVec3::ONE, state);
	}

	pub fn push_area(&mut self, grid: GridId, chunk: IVec3, size: IVec3, state: CameraLodDebugState) {
		self.chunks.push(CameraLodDebugChunk { grid, chunk, size, state });
	}
}
