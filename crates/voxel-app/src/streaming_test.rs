//! Test harness for voxel-streaming: voxels are written straight into
//! a tmp store (never resident) and presence is marked, then physics chunk
//! requests are served from it. `voxel-streaming` writes received chunks into
//! the grids.

use std::collections::HashMap;

use bevy::math::I16Vec3;
use bevy::prelude::*;

use voxel_data::grid::{Grid, GridId};
use voxel_data::voxels::{Voxel, Voxels};
use voxel_streaming::{
	chunk_of, ChunkLoaderChannel, ChunkLoadResult, ChunkRequestChannel, GridStreaming, CHUNK_SIZE,
};

/// The "world data" the loader reads from, keyed by (grid, chunk): each chunk's
/// voxels as (chunk-local position, voxel).
#[derive(Resource, Default)]
pub struct WorldStore {
	chunks: HashMap<(GridId, IVec3), Vec<(I16Vec3, Voxel)>>,
}

pub struct StreamingTestPlugin;

impl Plugin for StreamingTestPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<WorldStore>()
			.add_systems(Update, serve_chunk_requests);
	}
}

/// Collects voxels destined for the streaming store before a grid is spawned.
#[derive(Default)]
pub struct StreamingVoxels(Vec<(IVec3, Voxel)>);

impl StreamingVoxels {
	pub fn new() -> Self {
		Self::default()
	}

	pub fn add_voxel(&mut self, pos: &IVec3, voxel: &Voxel) {
		self.0.push((*pos, *voxel));
	}
}

pub fn spawn_grid(
	commands: &mut Commands,
	store: &mut WorldStore,
	parent: Entity,
	transform: Transform,
	voxels: StreamingVoxels,
	extra: impl Bundle,
) {
	let child = commands
		.spawn((transform, Grid::new(), extra))
		.id();
	commands.entity(parent).add_child(child);

	let mut streaming = GridStreaming::default();
	for (voxel_pos, voxel) in voxels.0 {
		let chunk = chunk_of(voxel_pos);
		// Local offset is relative to the chunk so it round-trips with
		// `chunk_origin` (CHUNK_SIZE) in `receive_results`.
		let local = voxel_pos.rem_euclid(IVec3::splat(CHUNK_SIZE)).as_i16vec3();
		streaming.presence_mut().mark_present(chunk);
		store.chunks.entry((child, chunk)).or_default().push((local, voxel));
	}
	commands.entity(child).insert(streaming);
}

fn serve_chunk_requests(
	requests: Res<ChunkRequestChannel>,
	loader: Res<ChunkLoaderChannel>,
	store: Res<WorldStore>,
) {
	while let Some(request) = requests.try_recv() {
		let voxels = store.chunks.get(&(request.grid, request.chunk)).map(|list| {
			let mut voxels = Voxels::new();
			for (local, voxel) in list {
				voxels.add_voxel(*local, *voxel);
			}
			voxels
		});
		let sender = loader.sender();
		// std::thread::spawn(move || {
			// std::thread::sleep(std::time::Duration::from_millis(50));
			let _ = sender.send(ChunkLoadResult { grid: request.grid, chunk: request.chunk, voxels });
		// });
	}
}
