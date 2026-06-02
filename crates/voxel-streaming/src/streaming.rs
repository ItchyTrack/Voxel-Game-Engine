use bevy::math::IVec3;
use bevy::prelude::*;

use voxel_data::grid::{Grid, GridId};

use crate::chunk::chunk_origin;
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

	/// Marks an `Available` chunk as `InFlight`. Returns true if this call began
	/// the load (so the caller should emit a request).
	fn mark_fetched(&mut self, chunk: IVec3) -> bool {
		if self.presence.state(chunk) == Some(ChunkState::Available) {
			self.presence.set_state(chunk, ChunkState::InFlight);
			true
		} else {
			false
		}
	}

	/// Prefetch (non-gating): request a chunk without recording it anywhere.
	pub fn fetch(&mut self, grid: GridId, channel: &ChunkRequestChannel, chunk: IVec3) {
		if self.mark_fetched(chunk) {
			channel.request(ChunkLoadRequest { grid, chunk });
		}
	}

	/// Gating: request a chunk and record it in `consumer` until it loads.
	/// Already-resident or absent (unknown/empty) chunks are skipped (nothing to gate on).
	pub fn fetch_needed<C: ChunkConsumer>(
		&mut self,
		grid: GridId,
		consumer: &mut C,
		channel: &ChunkRequestChannel,
		chunk: IVec3,
	) {
		match self.presence.state(chunk) {
			None | Some(ChunkState::Loaded) => return,
			Some(ChunkState::Available) => {
				self.presence.set_state(chunk, ChunkState::InFlight);
				channel.request(ChunkLoadRequest { grid, chunk });
			}
			Some(ChunkState::InFlight) => {}
		}
		consumer.needed_mut().entry(grid).or_default().insert(chunk);
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
