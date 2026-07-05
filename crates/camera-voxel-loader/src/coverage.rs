use std::collections::HashSet;

use bevy::prelude::*;
use tracy_client::span;

use crate::camera_voxel_loader::CameraVoxelLoader;
use crate::replacement_graph::DependencyRecord;
use crate::types::TileKey;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SourceResolution { Requested, Visible(Entity), Empty }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SourceState { Desired(SourceResolution), RetiringVisible(Entity) }

pub(crate) fn request_source(loader: &mut CameraVoxelLoader, source: TileKey) {
	let _span = span!();
	match loader.coverage_sources.get(&source) {
		None => { loader.coverage_sources.insert(source, SourceState::Desired(SourceResolution::Requested)); }
		Some(SourceState::RetiringVisible(entity)) => {
			loader.replacement_graph.cancel_record(source);
			*loader.coverage_sources.get_mut(&source).unwrap() = SourceState::Desired(SourceResolution::Visible(*entity));
		}
		_ => {}
	}
}

pub(crate) fn undesire_source(loader: &mut CameraVoxelLoader, source: TileKey) -> Vec<TileKey> {
	match loader.coverage_sources.get(&source) {
		None | Some(SourceState::RetiringVisible(_)) => Vec::new(),
		Some(SourceState::Desired(SourceResolution::Requested | SourceResolution::Empty)) => {
			remove_source(loader, source);
			Vec::new()
		}
		Some(SourceState::Desired(SourceResolution::Visible(entity))) => {
			let replacements = unresolved_replacements_for(loader, source);
			*loader.coverage_sources.get_mut(&source).unwrap() = SourceState::RetiringVisible(*entity);
			if replacements.is_empty() {
				vec![source]
			} else {
				loader.replacement_graph.add_record(DependencyRecord::new(source, replacements));
				Vec::new()
			}
		}
	}
}

pub(crate) fn resolve_empty(loader: &mut CameraVoxelLoader, source: TileKey) -> Vec<TileKey> {
	resolve_source(loader, source, SourceResolution::Empty)
}

pub(crate) fn resolve_visible(loader: &mut CameraVoxelLoader, source: TileKey, entity: Entity) -> Vec<TileKey> {
	resolve_source(loader, source, SourceResolution::Visible(entity))
}

fn resolve_source(loader: &mut CameraVoxelLoader, source: TileKey, resolution: SourceResolution) -> Vec<TileKey> {
	match loader.coverage_sources.get(&source) {
		None => Vec::new(),
		Some(SourceState::Desired(_)) => {
			*loader.coverage_sources.get_mut(&source).unwrap() = SourceState::Desired(resolution);
			loader.replacement_graph.apply_satisfied(source)
		}
		Some(SourceState::RetiringVisible(_)) => {
			if let SourceResolution::Visible(entity) = resolution {
				*loader.coverage_sources.get_mut(&source).unwrap() = SourceState::RetiringVisible(entity);
			}
			let mut ready = loader.replacement_graph.apply_satisfied(source);
			if matches!(resolution, SourceResolution::Empty) { ready.push(source); }
			ready
		}
	}
}

pub(crate) fn retiring_visible_chunks(loader: &CameraVoxelLoader) -> Vec<TileKey> {
	loader.coverage_sources.iter().filter_map(|(&source, record)| matches!(record, SourceState::RetiringVisible(_)).then_some(source).filter(|key| key.lod == 0)).collect()
}

pub(crate) fn remove_source(loader: &mut CameraVoxelLoader, source: TileKey) {
	loader.coverage_sources.remove(&source);
	loader.replacement_graph.remove_source(source);
}

fn unresolved_replacements_for(loader: &CameraVoxelLoader, source: TileKey) -> HashSet<TileKey> {
	// Enumerate only the tiles that can geometrically overlap this source at each LOD.
	let mut replacements = HashSet::new();
	for lod in 0..=max_supported_lod() {
		collect_unresolved_replacements_at_lod(loader, source, lod, &mut replacements);
	}
	replacements
}

fn collect_unresolved_replacements_at_lod(
	loader: &CameraVoxelLoader,
	source: TileKey,
	lod: u8,
	replacements: &mut HashSet<TileKey>,
) {
	let size = 1i32 << lod;
	let tile = IVec3::splat(size);
	let source_max = source.min + source.size();
	let min = source.min.div_euclid(tile) * tile;

	let mut x = min.x;
	while x < source_max.x {
		let mut y = min.y;
		while y < source_max.y {
			let mut z = min.z;
			while z < source_max.z {
				let candidate = TileKey { grid: source.grid, lod, min: IVec3::new(x, y, z) };
				if candidate != source
					&& loader.desired_tiles.contains(&candidate)
					&& !matches!(
						loader.coverage_sources.get(&candidate),
						Some(SourceState::Desired(SourceResolution::Visible(_) | SourceResolution::Empty))
					) {
					replacements.insert(candidate);
				}
				z += size;
			}
			y += size;
		}
		x += size;
	}
}

const fn max_supported_lod() -> u8 {
	// Tile sizes use 1i32 << lod, so keep enumeration below the sign bit.
	i32::BITS as u8 - 2
}

#[cfg(test)]
mod tests {
	use super::*;

	fn grid(bits: u64) -> Entity { Entity::from_bits(bits) }
	fn tile(grid: Entity, lod: u8, min: IVec3) -> TileKey { TileKey { grid, lod, min } }

	#[test]
	fn same_grid_replacements_do_not_wait_on_other_grids() {
		let source_grid = grid(1);
		let other_grid = grid(2);
		let source = tile(source_grid, 1, IVec3::ZERO);
		let replacement = tile(source_grid, 0, IVec3::ZERO);
		let other_grid_replacement = tile(other_grid, 0, IVec3::ZERO);
		let mut loader = CameraVoxelLoader::default();
		loader.desired_tiles.insert(replacement);
		loader.desired_tiles.insert(other_grid_replacement);

		request_source(&mut loader, source);
		resolve_visible(&mut loader, source, Entity::from_bits(10));
		request_source(&mut loader, replacement);

		assert!(undesire_source(&mut loader, source).is_empty());
		assert_eq!(resolve_visible(&mut loader, replacement, Entity::from_bits(11)), vec![source]);
	}
}
