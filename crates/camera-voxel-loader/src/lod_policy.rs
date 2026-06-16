use std::collections::HashSet;

use bevy::prelude::*;
use voxel_data::grid::GridId;
use voxel_streaming::{GridStreaming, CHUNK_SIZE};

use crate::camera_voxel_loader::{CameraVoxelLoader, CameraVoxelLoaderSettings};
use crate::types::TileKey;

pub(crate) struct SetDelta<T> {
    pub(crate) added: Vec<T>,
    pub(crate) removed: Vec<T>,
}

pub(crate) struct DesiredSourceDelta {
    pub(crate) tiles: SetDelta<TileKey>,
}

pub(crate) fn nearest_chunk_center(local_voxels: Vec3) -> IVec3 {
    (local_voxels / CHUNK_SIZE as f32).round().as_ivec3()
}

#[allow(dead_code)]
pub(crate) fn add_near_tiles(out: &mut HashSet<TileKey>, grid: GridId, center: IVec3, loader: &CameraVoxelLoader, streaming: &GridStreaming) {
    let (min, max) = near_bounds(center, &loader.settings);
    for_each_chunk(min, max, |chunk| {
        if streaming.presence().is_present(chunk) {
            out.insert(TileKey { grid, lod: 0, min: chunk });
        }
    });
}

#[allow(dead_code)]
pub(crate) fn add_lod_tiles(out: &mut HashSet<TileKey>, grid: GridId, center: IVec3, loader: &CameraVoxelLoader, streaming: &GridStreaming) {
    add_lod_tiles_for_settings(out, grid, center, &loader.settings, streaming);
}

pub(crate) fn tile_key_covering_chunk(grid: GridId, chunk: IVec3, lod: u8) -> TileKey {
    TileKey { grid, lod, min: align_chunk_to_tile(chunk, 1i32 << lod) }
}

pub(crate) fn is_tile_wanted(settings: &CameraVoxelLoaderSettings, streaming: &GridStreaming, center: IVec3, key: TileKey) -> bool {
    if key.lod == 0 {
        let (min, max) = near_bounds(center, settings);
        key.min.cmpge(min).all() && key.min.cmplt(max).all() && streaming.presence().is_present(key.min)
    } else {
        let size = 1i32 << key.lod;
        let (inner_min, inner_max) = lod_inner_bounds(center, settings, key.lod);
        let (outer_min, outer_max) = lod_outer_bounds(center, settings, key.lod);
        let max = key.min + IVec3::splat(size);
        key.min.cmpge(outer_min).all()
            && max.cmple(outer_max).all()
            && !(key.min.cmpge(inner_min).all() && max.cmple(inner_max).all())
            && tile_has_present_chunk(streaming, key.min, size)
    }
}

pub(crate) fn update_desired_sources_delta(
    desired_tiles: &mut HashSet<TileKey>, grid: GridId, _previous_center: Option<IVec3>, center: IVec3, _settings_changed: bool,
    settings: &CameraVoxelLoaderSettings, streaming: &GridStreaming,
) -> DesiredSourceDelta {
    rebuild_desired_sources_delta(desired_tiles, grid, center, settings, streaming)
}

#[allow(dead_code)]
pub(crate) fn update_tiles_delta(
    out: &mut HashSet<TileKey>, grid: GridId, _old_center: IVec3, new_center: IVec3, settings: &CameraVoxelLoaderSettings,
    streaming: &GridStreaming,
) -> SetDelta<TileKey> {
    rebuild_desired_sources_delta(out, grid, new_center, settings, streaming).tiles
}

fn rebuild_desired_sources_delta(
    desired_tiles: &mut HashSet<TileKey>, grid: GridId, center: IVec3, settings: &CameraVoxelLoaderSettings, streaming: &GridStreaming,
) -> DesiredSourceDelta {
    let old: HashSet<_> = desired_tiles.iter().copied().filter(|key| key.grid == grid).collect();
    let mut new = HashSet::new();
    add_near_tiles_for_settings(&mut new, grid, center, settings, streaming);
    add_lod_tiles_for_settings(&mut new, grid, center, settings, streaming);

    let added: Vec<_> = new.difference(&old).copied().collect();
    let removed: Vec<_> = old.difference(&new).copied().collect();
    for key in &removed { desired_tiles.remove(key); }
    desired_tiles.extend(added.iter().copied());
    DesiredSourceDelta { tiles: SetDelta { added, removed } }
}

fn add_near_tiles_for_settings(out: &mut HashSet<TileKey>, grid: GridId, center: IVec3, settings: &CameraVoxelLoaderSettings, streaming: &GridStreaming) {
    let (min, max) = near_bounds(center, settings);
    for_each_chunk(min, max, |chunk| {
        if streaming.presence().is_present(chunk) {
            out.insert(TileKey { grid, lod: 0, min: chunk });
        }
    });
}

fn add_lod_tiles_for_settings(out: &mut HashSet<TileKey>, grid: GridId, center: IVec3, settings: &CameraVoxelLoaderSettings, streaming: &GridStreaming) {
    if streaming.presence().len() == 0 { return; }
    for lod in 1..=settings.max_lod {
        let size = 1i32 << lod;
        let (outer_min, outer_max) = lod_outer_bounds(center, settings, lod);
        for x in (outer_min.x..outer_max.x).step_by(size as usize) {
            for y in (outer_min.y..outer_max.y).step_by(size as usize) {
                for z in (outer_min.z..outer_max.z).step_by(size as usize) {
                    let key = TileKey { grid, lod, min: IVec3::new(x, y, z) };
                    if is_tile_wanted(settings, streaming, center, key) { out.insert(key); }
                }
            }
        }
    }
}

fn for_each_chunk(min: IVec3, max: IVec3, mut f: impl FnMut(IVec3)) {
    for x in min.x..max.x {
        for y in min.y..max.y {
            for z in min.z..max.z { f(IVec3::new(x, y, z)); }
        }
    }
}

fn tile_has_present_chunk(streaming: &GridStreaming, min: IVec3, size: i32) -> bool {
    streaming.presence().any_present_in_region(min, min + IVec3::splat(size) - IVec3::ONE)
}

pub(crate) fn align_chunk_to_tile(chunk: IVec3, size: i32) -> IVec3 {
    chunk.div_euclid(IVec3::splat(size)) * size
}

fn near_bounds(center: IVec3, settings: &CameraVoxelLoaderSettings) -> (IVec3, IVec3) {
    align_bounds_to_tile(center - IVec3::splat(settings.near_radius_chunks), center + IVec3::splat(settings.near_radius_chunks + 1), 2)
}

fn lod_inner_bounds(center: IVec3, settings: &CameraVoxelLoaderSettings, lod: u8) -> (IVec3, IVec3) {
    if lod == 1 { near_bounds(center, settings) } else { lod_outer_bounds(center, settings, lod - 1) }
}

fn lod_outer_bounds(center: IVec3, settings: &CameraVoxelLoaderSettings, lod: u8) -> (IVec3, IVec3) {
    let size = 1i32 << lod;
    let (min, max) = lod_inner_bounds(center, settings, lod);
    let bounds = (min - IVec3::splat(size * settings.rings_per_lod), max + IVec3::splat(size * settings.rings_per_lod));
    if lod < settings.max_lod { align_bounds_to_tile(bounds.0, bounds.1, 1i32 << (lod + 1)) } else { bounds }
}

fn align_bounds_to_tile(min: IVec3, max: IVec3, size: i32) -> (IVec3, IVec3) {
    let tile = IVec3::splat(size);
    (min.div_euclid(tile) * tile, (max + tile - IVec3::ONE).div_euclid(tile) * tile)
}
