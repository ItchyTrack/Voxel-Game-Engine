use bevy::prelude::*;

use crate::camera_voxel_loader::CameraVoxelLoader;
use crate::replacement_graph::DependencyRecord;
use crate::types::TileKey;

pub(crate) type CoverageSource = TileKey;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SourceResolution { Requested, Visible(Entity), Empty }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SourceState { Desired(SourceResolution), RetiringVisible(Entity) }

#[derive(Clone, Debug)]
pub(crate) struct CoverageRecord { pub(crate) state: SourceState }

pub(crate) fn request_source(loader: &mut CameraVoxelLoader, source: CoverageSource) {
    match loader.coverage_sources.get(&source).map(|record| record.state) {
        None => { loader.coverage_sources.insert(source, CoverageRecord { state: SourceState::Desired(SourceResolution::Requested) }); }
        Some(SourceState::RetiringVisible(entity)) => {
            loader.replacement_graph.cancel_record(source);
            loader.coverage_sources.get_mut(&source).unwrap().state = SourceState::Desired(SourceResolution::Visible(entity));
        }
        _ => {}
    }
}

pub(crate) fn renew_source_request(loader: &mut CameraVoxelLoader, source: CoverageSource) {
    loader.replacement_graph.remove_source(source);
    loader.coverage_sources.insert(source, CoverageRecord { state: SourceState::Desired(SourceResolution::Requested) });
}

pub(crate) fn undesire_source(loader: &mut CameraVoxelLoader, source: CoverageSource) -> Vec<CoverageSource> {
    match loader.coverage_sources.get(&source).map(|record| record.state) {
        None | Some(SourceState::RetiringVisible(_)) => Vec::new(),
        Some(SourceState::Desired(SourceResolution::Requested | SourceResolution::Empty)) => {
            remove_source(loader, source);
            Vec::new()
        }
        Some(SourceState::Desired(SourceResolution::Visible(entity))) => {
            let replacements = unresolved_replacements_for(loader, source);
            loader.coverage_sources.get_mut(&source).unwrap().state = SourceState::RetiringVisible(entity);
            if replacements.is_empty() {
                vec![source]
            } else {
                loader.replacement_graph.add_record(DependencyRecord::new(source, replacements));
                Vec::new()
            }
        }
    }
}

pub(crate) fn resolve_empty(loader: &mut CameraVoxelLoader, source: CoverageSource) -> Vec<CoverageSource> {
    resolve_source(loader, source, SourceResolution::Empty)
}

pub(crate) fn resolve_visible(loader: &mut CameraVoxelLoader, source: CoverageSource, entity: Entity) -> Vec<CoverageSource> {
    resolve_source(loader, source, SourceResolution::Visible(entity))
}

fn resolve_source(loader: &mut CameraVoxelLoader, source: CoverageSource, resolution: SourceResolution) -> Vec<CoverageSource> {
    match loader.coverage_sources.get(&source).map(|record| record.state) {
        None => Vec::new(),
        Some(SourceState::Desired(_)) => {
            loader.coverage_sources.get_mut(&source).unwrap().state = SourceState::Desired(resolution);
            loader.replacement_graph.apply_satisfied(source)
        }
        Some(SourceState::RetiringVisible(_)) => {
            if let SourceResolution::Visible(entity) = resolution {
                loader.coverage_sources.get_mut(&source).unwrap().state = SourceState::RetiringVisible(entity);
            }
            let mut ready = loader.replacement_graph.apply_satisfied(source);
            if matches!(resolution, SourceResolution::Empty) { ready.push(source); }
            ready
        }
    }
}

pub(crate) fn retiring_visible_chunks(loader: &CameraVoxelLoader) -> Vec<TileKey> {
    loader.coverage_sources.iter().filter_map(|(&source, record)| matches!(record.state, SourceState::RetiringVisible(_)).then_some(source).filter(|key| key.lod == 0)).collect()
}

pub(crate) fn remove_source(loader: &mut CameraVoxelLoader, source: CoverageSource) {
    loader.coverage_sources.remove(&source);
    loader.replacement_graph.remove_source(source);
}

fn unresolved_replacements_for(loader: &CameraVoxelLoader, source: CoverageSource) -> Vec<CoverageSource> {
    loader
        .desired_tiles
        .iter()
        .copied()
        .filter(|candidate| *candidate != source && sources_overlap(source, *candidate))
        .filter(|replacement| !matches!(loader.coverage_sources.get(replacement).map(|record| record.state), Some(SourceState::Desired(SourceResolution::Visible(_) | SourceResolution::Empty))))
        .collect()
}

fn sources_overlap(a: CoverageSource, b: CoverageSource) -> bool {
    let (a_min, a_max) = (a.min, a.min + a.size());
    let (b_min, b_max) = (b.min, b.min + b.size());
    a_min.cmplt(b_max).all() && b_min.cmplt(a_max).all()
}
