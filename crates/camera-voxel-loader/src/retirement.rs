use std::collections::HashSet;

use bevy::prelude::*;

use crate::camera_voxel_loader::CameraVoxelLoader;
use crate::types::{ChunkKey, TileKey, TileStatus};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) enum RetireTarget {
	Tile(TileKey),
	Chunk(ChunkKey),
}

#[derive(Debug, Default, Clone)]
pub(crate) struct RetireDeps {
	waiting_for_tiles: HashSet<TileKey>,
	waiting_for_chunks: HashSet<ChunkKey>,
	waiting_for_subgrids: HashSet<Entity>,
}

impl RetireDeps {
	fn is_ready(&self) -> bool {
		self.waiting_for_tiles.is_empty() && self.waiting_for_chunks.is_empty() && self.waiting_for_subgrids.is_empty()
	}

	fn remove_tile(&mut self, tile: TileKey) {
		self.waiting_for_tiles.remove(&tile);
	}

	fn remove_subgrid(&mut self, entity: Entity) {
		self.waiting_for_subgrids.remove(&entity);
	}

	fn remove_chunk(&mut self, chunk: ChunkKey) {
		self.waiting_for_chunks.remove(&chunk);
	}
}

pub(crate) fn register_tile_retirement(loader: &mut CameraVoxelLoader, tile: TileKey) {
	let deps = loader
		.desired_tiles
		.iter()
		.copied()
		.filter(|candidate| *candidate != tile && tiles_intersect(*candidate, tile))
		.filter(|candidate| !tile_is_resolved(loader, *candidate))
		.collect::<HashSet<_>>();
	register_deps(loader, RetireTarget::Tile(tile), deps);
}

pub(crate) fn register_chunk_retirement(loader: &mut CameraVoxelLoader, chunk: ChunkKey) {
	let deps = loader
		.desired_tiles
		.iter()
		.copied()
		.filter(|tile| tile_contains_chunk(*tile, chunk))
		.filter(|tile| !tile_is_resolved(loader, *tile))
		.collect::<HashSet<_>>();
	register_deps(loader, RetireTarget::Chunk(chunk), deps);
}

pub(crate) fn cancel_retirement(loader: &mut CameraVoxelLoader, target: RetireTarget) {
	match target {
		RetireTarget::Tile(tile) => {
			loader.retiring_tiles.remove(&tile);
		}
		RetireTarget::Chunk(chunk) => {
			loader.retiring_chunks.remove(&chunk);
		}
	}
	for dependents in loader.tile_dependents.values_mut() {
		dependents.retain(|dependent| *dependent != target);
	}
	for dependents in loader.subgrid_dependents.values_mut() {
		dependents.retain(|dependent| *dependent != target);
	}
	for dependents in loader.chunk_dependents.values_mut() {
		dependents.retain(|dependent| *dependent != target);
	}
}

pub(crate) fn notify_tile_resolved(loader: &mut CameraVoxelLoader, tile: TileKey) {
	let Some(dependents) = loader.tile_dependents.remove(&tile) else { return };
	for dependent in dependents {
		match dependent {
			RetireTarget::Tile(retiring) => {
				if let Some(deps) = loader.retiring_tiles.get_mut(&retiring) {
					deps.remove_tile(tile);
				}
			}
			RetireTarget::Chunk(retiring) => {
				if let Some(deps) = loader.retiring_chunks.get_mut(&retiring) {
					deps.remove_tile(tile);
				}
			}
		}
	}
}

pub(crate) fn add_chunk_dependency(loader: &mut CameraVoxelLoader, target: RetireTarget, chunk: ChunkKey) {
	let inserted = match target {
		RetireTarget::Tile(tile) => loader
			.retiring_tiles
			.get_mut(&tile)
			.is_some_and(|deps| deps.waiting_for_chunks.insert(chunk)),
		RetireTarget::Chunk(retiring_chunk) => loader
			.retiring_chunks
			.get_mut(&retiring_chunk)
			.is_some_and(|deps| deps.waiting_for_chunks.insert(chunk)),
	};
	if inserted {
		loader.chunk_dependents.entry(chunk).or_default().push(target);
	}
}

pub(crate) fn notify_chunk_resolved(loader: &mut CameraVoxelLoader, chunk: ChunkKey) {
	let Some(dependents) = loader.chunk_dependents.remove(&chunk) else { return };
	for dependent in dependents {
		match dependent {
			RetireTarget::Tile(retiring) => {
				if let Some(deps) = loader.retiring_tiles.get_mut(&retiring) {
					deps.remove_chunk(chunk);
				}
			}
			RetireTarget::Chunk(retiring) => {
				if let Some(deps) = loader.retiring_chunks.get_mut(&retiring) {
					deps.remove_chunk(chunk);
				}
			}
		}
	}
}

pub(crate) fn add_subgrid_dependency(loader: &mut CameraVoxelLoader, target: RetireTarget, entity: Entity) {
	let inserted = match target {
		RetireTarget::Tile(tile) => loader
			.retiring_tiles
			.get_mut(&tile)
			.is_some_and(|deps| deps.waiting_for_subgrids.insert(entity)),
		RetireTarget::Chunk(chunk) => loader
			.retiring_chunks
			.get_mut(&chunk)
			.is_some_and(|deps| deps.waiting_for_subgrids.insert(entity)),
	};
	if inserted {
		loader.subgrid_dependents.entry(entity).or_default().push(target);
	}
}

pub(crate) fn notify_subgrid_ready(loader: &mut CameraVoxelLoader, entity: Entity) {
	let Some(dependents) = loader.subgrid_dependents.remove(&entity) else { return };
	for dependent in dependents {
		match dependent {
			RetireTarget::Tile(retiring) => {
				if let Some(deps) = loader.retiring_tiles.get_mut(&retiring) {
					deps.remove_subgrid(entity);
				}
			}
			RetireTarget::Chunk(retiring) => {
				if let Some(deps) = loader.retiring_chunks.get_mut(&retiring) {
					deps.remove_subgrid(entity);
				}
			}
		}
	}
}

pub(crate) fn ready_retiring_tiles(loader: &CameraVoxelLoader) -> Vec<TileKey> {
	loader
		.retiring_tiles
		.iter()
		.filter_map(|(tile, deps)| deps.is_ready().then_some(*tile))
		.collect()
}

pub(crate) fn ready_retiring_chunks(loader: &CameraVoxelLoader) -> Vec<ChunkKey> {
	loader
		.retiring_chunks
		.iter()
		.filter_map(|(chunk, deps)| deps.is_ready().then_some(*chunk))
		.collect()
}

fn register_deps(loader: &mut CameraVoxelLoader, target: RetireTarget, deps: HashSet<TileKey>) {
	cancel_retirement(loader, target);
	for tile in &deps {
		loader.tile_dependents.entry(*tile).or_default().push(target);
	}
	match target {
		RetireTarget::Tile(tile) => {
			loader.retiring_tiles.insert(tile, RetireDeps { waiting_for_tiles: deps, waiting_for_chunks: HashSet::new(), waiting_for_subgrids: HashSet::new() });
		}
		RetireTarget::Chunk(chunk) => {
			loader.retiring_chunks.insert(chunk, RetireDeps { waiting_for_tiles: deps, waiting_for_chunks: HashSet::new(), waiting_for_subgrids: HashSet::new() });
		}
	}
}

fn tile_is_resolved(loader: &CameraVoxelLoader, tile: TileKey) -> bool {
	loader
		.tiles
		.get(&tile)
		.is_some_and(|record| matches!(record.status, TileStatus::Ready | TileStatus::Empty))
}

fn tile_contains_chunk(tile: TileKey, chunk: ChunkKey) -> bool {
	tile.grid == chunk.grid && chunk.chunk.cmpge(tile.min).all() && chunk.chunk.cmplt(tile.min + tile.size()).all()
}

fn tiles_intersect(a: TileKey, b: TileKey) -> bool {
	a.grid == b.grid && a.min.cmplt(b.min + b.size()).all() && b.min.cmplt(a.min + a.size()).all()
}
