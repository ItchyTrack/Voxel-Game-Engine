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

fn unresolved_replacements_for(loader: &CameraVoxelLoader, source: TileKey) -> Vec<TileKey> {
	loader
		.desired_tiles
		.iter()
		.copied()
		.filter(|candidate| *candidate != source && sources_overlap(source, *candidate))
		.filter(|replacement| !matches!(loader.coverage_sources.get(replacement), Some(SourceState::Desired(SourceResolution::Visible(_) | SourceResolution::Empty))))
		.collect()
}

fn sources_overlap(a: TileKey, b: TileKey) -> bool {
	let (a_min, a_max) = (a.min, a.min + a.size());
	let (b_min, b_max) = (b.min, b.min + b.size());
	a_min.cmplt(b_max).all() && b_min.cmplt(a_max).all()
}
