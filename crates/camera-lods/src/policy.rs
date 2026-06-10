use std::collections::HashSet;

use bevy::prelude::*;
use lod_manager::{LodKey, LodRequestMap};
use voxel_data::grid::GridId;
use voxel_streaming::{CHUNK_SIZE, GridStreaming};

use crate::grid_control::CameraLodGridControl;

const MAX_LOD: u32 = 3;
pub const FULL_DETAIL_CHUNKS: f32 = 2.0;

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct FreezeCameraLods(pub bool);

fn ring_outer_chunks(lod: u32) -> f32 {
    FULL_DETAIL_CHUNKS * (1u32 << lod) as f32
}

fn lod_for_distance(distance: f32) -> Option<u32> {
    if distance < FULL_DETAIL_CHUNKS {
        return Some(0);
    }
    (1..=MAX_LOD).find(|&lod| distance < ring_outer_chunks(lod))
}

/// Declarative policy input for camera LODs.
#[derive(Component, Default, Debug, Clone)]
pub struct CameraLodPolicy {
    desired: Vec<CameraLodTarget>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CameraLodTarget {
    pub key: LodKey,
    pub priority: f32,
}

impl CameraLodPolicy {
    pub fn clear(&mut self) {
        self.desired.clear();
    }

    pub fn set_lod(&mut self, grid: GridId, min: IVec3, size: IVec3, level: u32, priority: f32) {
        self.desired.push(CameraLodTarget { key: LodKey::from_level(grid, min, size, level), priority });
    }

    pub fn targets(&self) -> &[CameraLodTarget] {
        &self.desired
    }
}

fn distance_to_area(camera_chunk: Vec3, min: IVec3, size: IVec3) -> f32 {
    let nearest = camera_chunk.clamp(min.as_vec3(), (min + size).as_vec3());
    camera_chunk.distance(nearest)
}

fn coalesced_areas(mut chunks: HashSet<IVec3>) -> Vec<(IVec3, IVec3)> {
    let mut areas = Vec::new();
    while let Some(min) = chunks.iter().min_by_key(|p| (p.z, p.y, p.x)).copied() {
        let mut size = IVec3::ONE;

        while chunks.contains(&(min + IVec3::new(size.x, 0, 0))) {
            size.x += 1;
        }

        'grow_y: loop {
            for x in 0..size.x {
                if !chunks.contains(&(min + IVec3::new(x, size.y, 0))) {
                    break 'grow_y;
                }
            }
            size.y += 1;
        }

        'grow_z: loop {
            for y in 0..size.y {
                for x in 0..size.x {
                    if !chunks.contains(&(min + IVec3::new(x, y, size.z))) {
                        break 'grow_z;
                    }
                }
            }
            size.z += 1;
        }

        for z in 0..size.z {
            for y in 0..size.y {
                for x in 0..size.x {
                    chunks.remove(&(min + IVec3::new(x, y, z)));
                }
            }
        }
        areas.push((min, size));
    }
    areas
}

/// Rebuild the desired camera LOD areas from the active camera, then let
/// [`CameraLodGridControl`] apply only the delta against the previous set.
///
/// The policy scans present chunks near the camera, bins them by requested LOD, and
/// coalesces adjacent chunks with the same LOD into larger areas before syncing. This
/// keeps the request map compact without inventing fixed LOD tiles in the camera policy.
pub fn update_camera_lod_policy(
    freeze: Res<FreezeCameraLods>, mut cameras: Query<(&Camera, &GlobalTransform, &mut CameraLodPolicy)>,
    grids: Query<(GridId, &GlobalTransform, &GridStreaming)>,
) {
    if freeze.0 {
        return;
    }
    let Some((_, camera_transform, mut policy)) = cameras.iter_mut().find(|(camera, _, _)| camera.is_active) else { return };
    let camera_world = camera_transform.translation();

    policy.clear();

    for (grid, grid_global, streaming) in grids.iter() {
        let camera_local = grid_global.affine().inverse().transform_point3(camera_world);
        let camera_chunk = camera_local / CHUNK_SIZE as f32;
        let center = camera_chunk.floor().as_ivec3();
        let max_radius = ring_outer_chunks(MAX_LOD).ceil() as i32;

        let min = center - IVec3::splat(max_radius);
        let max = center + IVec3::splat(max_radius);
        let mut chunks_by_lod: Vec<HashSet<IVec3>> = (0..=MAX_LOD).map(|_| HashSet::new()).collect();
        streaming.presence().for_each_in_region(min, max, |chunk| {
            let distance = distance_to_area(camera_chunk, chunk, IVec3::ONE);
            let Some(level) = lod_for_distance(distance) else { return };
            chunks_by_lod[level as usize].insert(chunk);
        });

        for (level, chunks) in chunks_by_lod.into_iter().enumerate() {
            for (area_min, area_size) in coalesced_areas(chunks) {
                let priority = -distance_to_area(camera_chunk, area_min, area_size) * CHUNK_SIZE as f32;
                policy.set_lod(grid, area_min, area_size, level as u32, priority);
            }
        }
    }
}

pub fn apply_camera_lod_policy(freeze: Res<FreezeCameraLods>, mut cameras: Query<(&CameraLodPolicy, &mut CameraLodGridControl, &mut LodRequestMap)>) {
    if freeze.0 {
        return;
    }
    for (policy, mut control, mut requests) in cameras.iter_mut() {
        control.sync(&mut requests, policy.targets().iter().map(|target| (target.key, target.priority)));
    }
}
