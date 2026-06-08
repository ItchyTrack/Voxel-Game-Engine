use std::collections::HashSet;

use bevy::camera::Camera;
use bevy::prelude::*;

use gpu_voxel_data::residency::ResidencyBuffers;
use gpu_voxel_data::LodVoxels;
use voxel_streaming::{ChunkRequestChannel, GridStreaming, CHUNK_SIZE};

use crate::lod_requests::{max_lod_radius_chunks, FULL_DETAIL_CHUNKS};

/// Voxel radius around the active camera within which the renderer keeps a grid's
/// chunks resident at full resolution. Tied to the LOD full-detail radius so the
/// full-resolution region and the LOD ring meet exactly, with no overlap or gap.
const REQUEST_RADIUS: f32 = FULL_DETAIL_CHUNKS * CHUNK_SIZE as f32;

/// Chunks the renderer currently wants resident for a grid. Diffed each frame to
/// prefetch newly wanted chunks and release ones the camera has moved away from.
#[derive(Component, Default)]
pub struct RenderWantedChunks(HashSet<IVec3>);

impl RenderWantedChunks {
	pub(crate) fn wants(&self, chunk: IVec3) -> bool {
		self.0.contains(&chunk)
	}
}

/// Prefetch every present chunk within [`REQUEST_RADIUS`] voxels of the camera,
/// in each grid's local space, and release chunks once they fall outside it.
pub fn request_render_chunks(
	mut commands: Commands,
	cameras: Query<(&Camera, &GlobalTransform)>,
	mut grids: Query<(Entity, &GlobalTransform, &mut GridStreaming, Option<&mut RenderWantedChunks>)>,
	lod_tiles: Query<(Entity, &LodVoxels)>,
	residency: Res<ResidencyBuffers>,
	channel: Res<ChunkRequestChannel>,
	freeze: Res<crate::scene::FreezeRenderRequests>,
) {
	if freeze.0 { return; }
	let Some(cam_world) = cameras
		.iter()
		.find(|(c, _)| c.is_active)
		.map(|(_, tf)| tf.translation())
	else { return };

	for (entity, grid_global, mut streaming, wanted) in grids.iter_mut() {
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

		// Keep a chunk rendering full-res until a GPU-ready LOD tile covers it (or it
		// leaves view range), so the full-res -> LOD handoff has no hole.
		let leaving: Vec<IVec3> = prev.difference(&want).copied().collect();
		let mut held = want;
		if !leaving.is_empty() {
			let lod_covered = lod_covered_chunks(&lod_tiles, entity, &residency);
			let camera_chunk = local / CHUNK_SIZE as f32;
			let max_radius = max_lod_radius_chunks();
			for chunk in leaving {
				let lo = chunk.as_vec3();
				let hi = (chunk + IVec3::ONE).as_vec3();
				let distance = camera_chunk.distance(camera_chunk.clamp(lo, hi));
				if lod_covered.contains(&chunk) || distance >= max_radius {
					streaming.release(chunk);
				} else {
					held.insert(chunk);
				}
			}
		}

		match wanted {
			Some(mut w) => w.0 = held,
			None => { commands.entity(entity).insert(RenderWantedChunks(held)); }
		}
	}
}

fn lod_covered_chunks(
	lod_tiles: &Query<(Entity, &LodVoxels)>,
	grid: Entity,
	residency: &ResidencyBuffers,
) -> HashSet<IVec3> {
	let mut covered = HashSet::new();
	for (entity, tile) in lod_tiles.iter() {
		if tile.grid != grid || !residency.offsets().contains_key(&entity) { continue; }
		for z in 0..tile.size.z {
			for y in 0..tile.size.y {
				for x in 0..tile.size.x {
					covered.insert(tile.min + IVec3::new(x, y, z));
				}
			}
		}
	}
	covered
}
