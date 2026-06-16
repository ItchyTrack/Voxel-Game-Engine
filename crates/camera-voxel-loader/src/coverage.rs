use crate::camera_voxel_loader::CameraVoxelLoader;
use crate::replacement_graph::DependencyRecord;
use crate::types::TileKey;
use bevy::prelude::*;

pub(crate) type CoverageSource = TileKey;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SourceResolution {
	Requested,
	Visible(Entity),
	Empty,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SourceState {
	Desired(SourceResolution),
	RetiringVisible(Entity),
}

#[derive(Clone, Debug)]
pub(crate) struct CoverageRecord {
	pub(crate) state: SourceState,
}

pub(crate) fn request_source(loader: &mut CameraVoxelLoader, source: CoverageSource) {
	if !loader.coverage_sources.contains_key(&source) {
		loader.coverage_sources.insert(source, CoverageRecord { state: SourceState::Desired(SourceResolution::Requested) });
		return;
	}
	if let Some(SourceState::RetiringVisible(entity)) = loader.coverage_sources.get(&source).map(|record| record.state) {
		loader.replacement_graph.cancel_record(source);
		if let Some(record) = loader.coverage_sources.get_mut(&source) {
			record.state = SourceState::Desired(SourceResolution::Visible(entity));
		}
	}
}

pub(crate) fn renew_source_request(loader: &mut CameraVoxelLoader, source: CoverageSource) {
	loader.replacement_graph.remove_source(source);
	loader.coverage_sources.insert(source, CoverageRecord { state: SourceState::Desired(SourceResolution::Requested) });
}

pub(crate) fn undesire_source(loader: &mut CameraVoxelLoader, source: CoverageSource) -> Vec<CoverageSource> {
	let Some(record) = loader.coverage_sources.get(&source).cloned() else { return Vec::new() };
	match record.state {
		SourceState::Desired(SourceResolution::Requested | SourceResolution::Empty) => {
			remove_source(loader, source);
			Vec::new()
		}
		SourceState::Desired(SourceResolution::Visible(entity)) => {
			let replacements = unresolved_replacements_for(loader, source);
			if let Some(record) = loader.coverage_sources.get_mut(&source) {
				record.state = SourceState::RetiringVisible(entity);
			}
			if replacements.is_empty() {
				vec![source]
			} else {
				loader.replacement_graph.add_record(DependencyRecord::new(source, replacements));
				Vec::new()
			}
		}
		SourceState::RetiringVisible(_) => Vec::new(),
	}
}

pub(crate) fn resolve_empty(loader: &mut CameraVoxelLoader, source: CoverageSource) -> Vec<CoverageSource> {
	let Some(record) = loader.coverage_sources.get(&source).cloned() else { return Vec::new() };
	match record.state {
		SourceState::Desired(_) => {
			if let Some(record) = loader.coverage_sources.get_mut(&source) {
				record.state = SourceState::Desired(SourceResolution::Empty);
			}
			loader.replacement_graph.apply_satisfied(source)
		}
		SourceState::RetiringVisible(_) => {
			let mut ready = loader.replacement_graph.apply_satisfied(source);
			ready.push(source);
			ready
		}
	}
}

pub(crate) fn resolve_visible(loader: &mut CameraVoxelLoader, source: CoverageSource, entity: Entity) -> Vec<CoverageSource> {
	let Some(record) = loader.coverage_sources.get(&source).cloned() else { return Vec::new() };
	match record.state {
		SourceState::Desired(_) => {
			if let Some(record) = loader.coverage_sources.get_mut(&source) {
				record.state = SourceState::Desired(SourceResolution::Visible(entity));
			}
			loader.replacement_graph.apply_satisfied(source)
		}
		SourceState::RetiringVisible(_) => {
			if let Some(record) = loader.coverage_sources.get_mut(&source) {
				record.state = SourceState::RetiringVisible(entity);
			}
			loader.replacement_graph.apply_satisfied(source)
		}
	}
}

pub(crate) fn retiring_visible_chunks(loader: &CameraVoxelLoader) -> Vec<TileKey> {
	loader
		.coverage_sources
		.iter()
		.filter_map(|(&source, record)| match record.state {
			SourceState::RetiringVisible(_) if source.lod == 0 => Some(source),
			_ => None,
		})
		.collect()
}

pub(crate) fn remove_source(loader: &mut CameraVoxelLoader, source: CoverageSource) {
	loader.coverage_sources.remove(&source);
	loader.replacement_graph.remove_source(source);
}

fn unresolved_replacements_for(loader: &CameraVoxelLoader, source: CoverageSource) -> Vec<CoverageSource> {
	replacement_sources_for(loader, source)
		.into_iter()
		.filter(|replacement| {
			!matches!(
				loader.coverage_sources.get(replacement).map(|record| record.state),
				Some(SourceState::Desired(SourceResolution::Visible(_) | SourceResolution::Empty))
			)
		})
		.collect()
}

fn replacement_sources_for(loader: &CameraVoxelLoader, source: CoverageSource) -> Vec<CoverageSource> {
	loader.desired_tiles.iter().copied().filter(|candidate| *candidate != source && sources_overlap(source, *candidate)).collect()
}

fn sources_overlap(a: CoverageSource, b: CoverageSource) -> bool {
	let (a_min, a_max) = source_bounds(a);
	let (b_min, b_max) = source_bounds(b);
	a_min.cmplt(b_max).all() && b_min.cmplt(a_max).all()
}

fn source_bounds(source: CoverageSource) -> (IVec3, IVec3) {
	(source.min, source.min + source.size())
}
