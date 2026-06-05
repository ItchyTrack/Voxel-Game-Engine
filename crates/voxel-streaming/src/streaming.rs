use bevy::math::IVec3;
use bevy::prelude::*;

use voxel_data::grid::{Grid, GridId};

use crate::chunk::{chunk_origin, CHUNK_SIZE};
use crate::consumer::ChunkConsumer;
use crate::loader::{ChunkLoadRequest, ChunkLoaderChannel, ChunkRequestChannel};
use crate::presence::{ChunkPresence, ChunkState};

#[derive(Component, Default)]
pub struct GridStreaming {
	presence: ChunkPresence,
}

impl GridStreaming {
	pub fn presence(&self) -> &ChunkPresence { &self.presence }
	pub fn presence_mut(&mut self) -> &mut ChunkPresence { &mut self.presence }

	pub fn state(&self, chunk: IVec3) -> Option<ChunkState> {
		self.presence.state(chunk)
	}

	pub fn is_loaded(&self, chunk: IVec3) -> bool {
		matches!(self.presence.state(chunk), Some(ChunkState::Loaded))
	}

	/// Begin a load if the chunk is `Available`, emitting a request. Records the
	/// requesting object in the chunk's count. Absent (unknown/empty) chunks are
	/// skipped. Callers must call this exactly once when they first need a chunk,
	/// paired with one [`GridStreaming::release`] when they no longer need it.
	fn start_request(&mut self, grid: GridId, channel: &ChunkRequestChannel, chunk: IVec3) -> bool {
		match self.presence.state(chunk) {
			None => return false,
			Some(ChunkState::Available) => {
				self.presence.set_state(chunk, ChunkState::InFlight);
				channel.request(ChunkLoadRequest { grid, chunk });
			}
			Some(ChunkState::InFlight) | Some(ChunkState::Loaded) => {}
		}
		self.presence.add_request(chunk);
		true
	}

	/// Prefetch (non-gating): request a chunk without recording it in a consumer.
	pub fn fetch(&mut self, grid: GridId, channel: &ChunkRequestChannel, chunk: IVec3) {
		self.start_request(grid, channel, chunk);
	}

	/// Gating: request a chunk and record it in `consumer` until it loads.
	/// Absent (unknown/empty) chunks are skipped (nothing to gate on).
	pub fn fetch_needed<C: ChunkConsumer>(
		&mut self,
		grid: GridId,
		consumer: &mut C,
		channel: &ChunkRequestChannel,
		chunk: IVec3,
	) {
		if !self.start_request(grid, channel, chunk) { return; }
		if self.presence.state(chunk) != Some(ChunkState::Loaded) {
			consumer.needed_mut().entry(grid).or_default().insert(chunk);
		}
	}

	/// Drop one object's request for `chunk`. When the last requester releases
	/// it, the chunk's voxels are removed from `grid` and it returns to
	/// `Available`. Pairs with one earlier [`fetch`](Self::fetch) /
	/// [`fetch_needed`](Self::fetch_needed); assumes no double releases.
	pub fn release(&mut self, chunk: IVec3, grid: &mut Grid) {
		if self.presence.remove_request(chunk) > 0 { return; }
		match self.presence.state(chunk) {
			Some(ChunkState::Loaded) => {
				grid.remove_area(&chunk_origin(chunk), &IVec3::splat(CHUNK_SIZE));
				self.presence.set_state(chunk, ChunkState::Available);
			}
			Some(_) => self.presence.set_state(chunk, ChunkState::Available),
			None => {}
		}
	}

	fn mark_loaded(&mut self, chunk: IVec3) {
		self.presence.set_state(chunk, ChunkState::Loaded);
	}

	fn mark_empty(&mut self, chunk: IVec3) {
		self.presence.clear_present(chunk);
	}
}

/// Drain finished loads: write their voxels straight into the grid, update
/// chunk state, and clear them from every consumer's needed set (so
/// `needed.is_empty()` tracks readiness). Runs before `ApplyGridEdits` so the
/// queued edits land the same frame.
pub fn receive_results(
	channel: Res<ChunkLoaderChannel>,
	mut grids: Query<(&mut GridStreaming, &mut Grid)>,
	mut consumers: Query<&mut dyn ChunkConsumer>,
) {
	while let Some(result) = channel.try_recv() {
		if let Ok((mut streaming, mut grid)) = grids.get_mut(result.grid) {
			match result.voxels {
				// Released while in flight: don't make it resident.
				Some(_) if streaming.presence.request_count(result.chunk) == 0 => {
					streaming.presence.set_state(result.chunk, ChunkState::Available);
				}
				Some(voxels) => {
					streaming.mark_loaded(result.chunk);
					let base = chunk_origin(result.chunk);
					let palette = voxels.palette();
					for (pos, size, palette_id) in voxels.grid_tree().iter() {
						let Some(voxel) = palette.voxel(palette_id) else { continue };
						let origin = base + pos.as_ivec3();
						for dx in 0..size as i32 {
							for dy in 0..size as i32 {
								for dz in 0..size as i32 {
									grid.add_voxel(&(origin + IVec3::new(dx, dy, dz)), voxel);
								}
							}
						}
					}
				}
				None => streaming.mark_empty(result.chunk),
			}
		}
		for mut entity_consumers in consumers.iter_mut() {
			for mut consumer in &mut entity_consumers {
				let needed = consumer.needed_mut();
				let now_empty = match needed.get_mut(&result.grid) {
					Some(set) => {
						set.remove(&result.chunk);
						set.is_empty()
					}
					None => false,
				};
				if now_empty {
					needed.remove(&result.grid);
				}
			}
		}
	}
}
