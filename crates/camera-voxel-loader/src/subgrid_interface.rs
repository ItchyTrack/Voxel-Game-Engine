use std::collections::HashSet;

use bevy::prelude::*;
use voxel_gpu::{VoxelGpuFormat, VoxelGpuState};
use voxel_data::{grid::{Grid, GridId}, subgrid::SubGrid};
use voxel_streaming::CHUNK_SIZE;

use crate::{camera_voxel_loader::CameraVoxelLoader, loading::{resolve_empty, resolve_visible}, types::TileKey};

pub(crate) fn chunks_in_bounds(grid: GridId, min: IVec3, max: IVec3) -> Vec<TileKey> {
	let min = min.div_euclid(IVec3::splat(CHUNK_SIZE));
	let max = max.div_euclid(IVec3::splat(CHUNK_SIZE));
	let mut chunks = Vec::new();
	for x in min.x..=max.x { for y in min.y..=max.y { for z in min.z..=max.z { chunks.push(TileKey::chunk(grid, IVec3::new(x, y, z))); } } }
	chunks
}

pub(crate) fn resolve_chunk_source_if_ready(
	loader: &mut CameraVoxelLoader,
	format: VoxelGpuFormat,
	grid: &Grid,
	subgrids: &Query<&SubGrid>,
	gpu_state: &Query<&VoxelGpuState>,
	chunk: TileKey,
) -> Vec<TileKey> {
	let mut ready = Vec::new();
	let mut found = false;
	let mut render_entities = HashSet::new();
	for entity in grid.subgrid_entities_in_area(chunk.min * CHUNK_SIZE, IVec3::splat(CHUNK_SIZE)) {
		if subgrids.get(entity).is_err() { continue; }
		found = true;
		if gpu_state.get(entity).is_ok_and(|state| state.matches(format)) {
			render_entities.insert(entity);
		}
	}
	if let Some(&entity) = render_entities.iter().next() {
		set_chunk_render_entities(loader, chunk, render_entities);
		ready.extend(resolve_visible(loader, chunk, entity));
	} else {
		set_chunk_render_entities(loader, chunk, HashSet::new());
		if !found { ready.extend(resolve_empty(loader, chunk)); }
	}
	ready
}

pub(crate) fn insert_chunk_render_entity(loader: &mut CameraVoxelLoader, chunk: TileKey, entity: Entity) {
	if loader.chunk_render_entities.entry(chunk).or_default().insert(entity) {
		increment_subgrid_ref(loader, entity);
	}
}

pub(crate) fn clear_chunk_render_entities(loader: &mut CameraVoxelLoader, chunk: TileKey) {
	set_chunk_render_entities(loader, chunk, HashSet::new());
}

fn set_chunk_render_entities(loader: &mut CameraVoxelLoader, chunk: TileKey, new_entities: HashSet<Entity>) {
	let old_entities = loader.chunk_render_entities.remove(&chunk).unwrap_or_default();
	for &entity in old_entities.difference(&new_entities) {
		decrement_subgrid_ref(loader, entity);
	}
	for &entity in new_entities.difference(&old_entities) {
		increment_subgrid_ref(loader, entity);
	}
	if !new_entities.is_empty() {
		loader.chunk_render_entities.insert(chunk, new_entities);
	}
}

fn increment_subgrid_ref(loader: &mut CameraVoxelLoader, entity: Entity) {
	*loader.subgrid_render_refs.entry(entity).or_default() += 1;
}

fn decrement_subgrid_ref(loader: &mut CameraVoxelLoader, entity: Entity) {
	let Some(count) = loader.subgrid_render_refs.get_mut(&entity) else { return; };
	*count = count.saturating_sub(1);
	if *count == 0 {
		loader.subgrid_render_refs.remove(&entity);
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	fn grid(bits: u64) -> Entity { Entity::from_bits(bits) }
	fn chunk(grid: Entity, min: IVec3) -> TileKey { TileKey::chunk(grid, min) }

	#[test]
	fn shared_subgrid_entity_stays_renderable_until_all_chunks_release_it() {
		let grid = grid(1);
		let entity = Entity::from_bits(10);
		let a = chunk(grid, IVec3::ZERO);
		let b = chunk(grid, IVec3::X);
		let mut loader = CameraVoxelLoader::default();

		insert_chunk_render_entity(&mut loader, a, entity);
		insert_chunk_render_entity(&mut loader, b, entity);
		assert_eq!(loader.subgrid_render_refs.get(&entity), Some(&2));
		assert_eq!(loader.subgrids_to_render().collect::<HashSet<_>>(), HashSet::from([entity]));

		clear_chunk_render_entities(&mut loader, a);
		assert_eq!(loader.subgrid_render_refs.get(&entity), Some(&1));
		assert_eq!(loader.subgrids_to_render().collect::<HashSet<_>>(), HashSet::from([entity]));

		clear_chunk_render_entities(&mut loader, b);
		assert!(!loader.subgrid_render_refs.contains_key(&entity));
		assert!(loader.subgrids_to_render().next().is_none());
	}

	#[test]
	fn one_chunk_can_render_multiple_subgrid_entities() {
		let grid = grid(1);
		let a = Entity::from_bits(10);
		let b = Entity::from_bits(11);
		let chunk = chunk(grid, IVec3::ZERO);
		let mut loader = CameraVoxelLoader::default();

		set_chunk_render_entities(&mut loader, chunk, HashSet::from([a, b]));
		assert_eq!(loader.subgrids_to_render().collect::<HashSet<_>>(), HashSet::from([a, b]));

		clear_chunk_render_entities(&mut loader, chunk);
		assert!(loader.subgrids_to_render().next().is_none());
	}
}
