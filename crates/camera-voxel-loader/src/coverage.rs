use std::collections::HashSet;

use bevy::prelude::*;
use tracy_client::span;
use voxel_data::grid_tree::GridRegion;

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
		None => {
			loader.coverage_sources.insert(source, SourceState::Desired(SourceResolution::Requested));
		}
		Some(SourceState::RetiringVisible(entity)) => {
			loader.replacement_graph.cancel_record(source);
			*loader.coverage_sources.get_mut(&source).unwrap() = SourceState::Desired(SourceResolution::Visible(*entity));
		}
		_ => {}
	}
	sync_unresolved_source(loader, source);
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
	let ready = match loader.coverage_sources.get(&source) {
		None => return Vec::new(),
		Some(SourceState::Desired(_)) => {
			*loader.coverage_sources.get_mut(&source).unwrap() = SourceState::Desired(resolution);
			loader.replacement_graph.apply_satisfied(source)
		}
		Some(SourceState::RetiringVisible(_)) => {
			if let SourceResolution::Visible(entity) = resolution {
				*loader.coverage_sources.get_mut(&source).unwrap() = SourceState::RetiringVisible(entity);
			}
			let mut ready = loader.replacement_graph.apply_satisfied(source);
			if matches!(resolution, SourceResolution::Empty) {
				ready.push(source);
			}
			ready
		}
	};
	sync_unresolved_source(loader, source);
	ready
}

pub(crate) fn retiring_visible_chunks(loader: &CameraVoxelLoader) -> Vec<TileKey> {
	loader.coverage_sources.iter().filter_map(|(&source, record)| matches!(record, SourceState::RetiringVisible(_)).then_some(source).filter(|key| key.lod == 0)).collect()
}

pub(crate) fn remove_source(loader: &mut CameraVoxelLoader, source: TileKey) {
	loader.coverage_sources.remove(&source);
	loader.unresolved_tiles.remove(source);
	loader.replacement_graph.remove_source(source);
}

fn unresolved_replacements_for(loader: &CameraVoxelLoader, source: TileKey) -> HashSet<TileKey> {
	let Some(region) = GridRegion::from_min_size(source.min, source.size()) else { return HashSet::new() };
	let mut replacements = HashSet::new();
	loader.unresolved_tiles.for_each_in_region(source.grid, region, loader.settings.max_lod, Some(source.lod), |candidate| {
		replacements.insert(candidate);
	});
	replacements
}

fn sync_unresolved_source(loader: &mut CameraVoxelLoader, source: TileKey) {
	if loader.desired_tiles.contains(&source)
		&& matches!(loader.coverage_sources.get(&source), Some(SourceState::Desired(SourceResolution::Requested))) {
		loader.unresolved_tiles.insert(source);
	} else {
		loader.unresolved_tiles.remove(source);
	}
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
		loader.insert_desired_tile(replacement);
		loader.insert_desired_tile(other_grid_replacement);

		request_source(&mut loader, source);
		resolve_visible(&mut loader, source, Entity::from_bits(10));
		request_source(&mut loader, replacement);

		assert!(undesire_source(&mut loader, source).is_empty());
		assert_eq!(resolve_visible(&mut loader, replacement, Entity::from_bits(11)), vec![source]);
	}
}
