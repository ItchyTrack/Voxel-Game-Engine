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

	pub fn fetch(&mut self, grid: GridId, channel: &ChunkRequestChannel, chunk: IVec3) {
		self.start_request(grid, channel, chunk);
	}

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

	pub fn release(&mut self, chunk: IVec3, grid: &mut Grid) {
		if self.presence.remove_request(chunk) > 0 { return; }
		if let Some(ChunkState::Loaded) = self.presence.state(chunk) {
			grid.remove_area(&chunk_origin(chunk), &IVec3::splat(CHUNK_SIZE));
			self.presence.set_state(chunk, ChunkState::Available);
		}
	}

	// Fine to call if the chunk was requested with `fetch`
	pub fn release_needed<C: ChunkConsumer>(
		&mut self,
		grid: GridId,
		consumer: &mut C,
		chunk: IVec3,
		grid_data: &mut Grid,
	) {
		if let Some(set) = consumer.needed_mut().get_mut(&grid) {
			set.remove(&chunk);
			if set.is_empty() { consumer.needed_mut().remove(&grid); }
		}
		self.release(chunk, grid_data);
	}

	fn mark_loaded(&mut self, chunk: IVec3) {
		self.presence.set_state(chunk, ChunkState::Loaded);
	}

	fn mark_empty(&mut self, chunk: IVec3) {
		self.presence.clear_present(chunk);
	}
}

pub fn receive_results(
	channel: Res<ChunkLoaderChannel>,
	mut grids: Query<(&mut GridStreaming, &mut Grid)>,
	mut consumers: Query<&mut dyn ChunkConsumer>,
) {
	while let Some(result) = channel.try_recv() {
		if let Ok((mut streaming, mut grid)) = grids.get_mut(result.grid) {
			if streaming.presence.state(result.chunk) == Some(ChunkState::InFlight) {
				match result.voxels {
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
			} else {
				bevy::log::warn!("receive_results received a chunk without anyone requesting it!");
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
