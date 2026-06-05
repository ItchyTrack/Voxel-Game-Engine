use std::collections::HashSet;

use bevy::camera::Camera;
use bevy::prelude::*;

use voxel_data::grid::Grid;
use voxel_streaming::{ChunkRequestChannel, GridStreaming, CHUNK_SIZE};

/// Voxel radius around the active camera within which the renderer keeps a
/// grid's chunks resident.
const REQUEST_RADIUS: f32 = 128.0;

/// Chunks the renderer currently wants resident for a grid. Diffed each frame to
/// prefetch newly wanted chunks and release ones the camera has moved away from.
#[derive(Component, Default)]
pub struct RenderWantedChunks(HashSet<IVec3>);

/// Prefetch every present chunk within [`REQUEST_RADIUS`] voxels of the camera,
/// in each grid's local space, and release chunks once they fall outside it.
pub fn request_render_chunks(
	mut commands: Commands,
	cameras: Query<(&Camera, &GlobalTransform)>,
	mut grids: Query<(Entity, &GlobalTransform, &mut GridStreaming, &mut Grid, Option<&mut RenderWantedChunks>)>,
	channel: Res<ChunkRequestChannel>,
) {
	let Some(cam_world) = cameras
		.iter()
		.find(|(c, _)| c.is_active)
		.map(|(_, tf)| tf.translation())
	else { return };

	for (entity, grid_global, mut streaming, mut grid, wanted) in grids.iter_mut() {
		// Camera position in this grid's local voxel space.
		let local = grid_global.affine().inverse().transform_point3(cam_world);
		let cmin = ((local - REQUEST_RADIUS) / CHUNK_SIZE as f32).floor().as_ivec3();
		let cmax = ((local + REQUEST_RADIUS) / CHUNK_SIZE as f32).floor().as_ivec3();

		let mut want = HashSet::new();
		streaming.presence().for_each_in_region(cmin, cmax, |chunk| {
			let lo = (chunk * CHUNK_SIZE).as_vec3();
			let hi = ((chunk + IVec3::ONE) * CHUNK_SIZE).as_vec3();
			// Nearest point of the chunk's AABB to the camera.
			if local.distance(local.clamp(lo, hi)) <= REQUEST_RADIUS {
				want.insert(chunk);
			}
		});

		let empty = HashSet::new();
		let prev = wanted.as_deref().map(|w| &w.0).unwrap_or(&empty);
		for &chunk in want.difference(prev) {
			streaming.fetch(entity, &channel, chunk);
		}
		for &chunk in prev.difference(&want) {
			streaming.release(chunk, grid.as_mut());
		}

		match wanted {
			Some(mut w) => w.0 = want,
			None => { commands.entity(entity).insert(RenderWantedChunks(want)); }
		}
	}
}
