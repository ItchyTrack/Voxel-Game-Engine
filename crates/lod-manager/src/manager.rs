use std::collections::{HashMap, HashSet};

use bevy::prelude::*;
use gpu_voxel_data::{LodVoxels, SubGridGpuState};
use voxel_data::grid::{Grid, GridId};
use voxel_streaming::{ChunkConsumer, ChunkRequestChannel, GridStreaming, LodRequestChannel, VoxelStreamingAppExt, CHUNK_SIZE};

use crate::{LoadedLods, LodKey, LodRequestMap, LodRetainCount, LodVisibleDelta, LodVisibleKind};

voxel_streaming::chunk_consumer!(pub LodManagerConsumer);

#[derive(Resource, Default)]
struct RequestedLods(HashSet<LodKey>);

#[derive(Resource, Default)]
struct RequestedChunks(HashSet<(GridId, IVec3)>);

#[derive(Resource, Default)]
struct Owners {
    lods: HashMap<LodKey, HashMap<Entity, f32>>,
    chunks: HashMap<(GridId, IVec3), HashSet<Entity>>,
    dirty: Vec<(Entity, GridId, IVec3, IVec3)>,
}

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum LodManagerSet {
    Collect,
    Request,
    Receive,
    Resolve,
    Retire,
}

#[derive(Default)]
pub struct LodManagerPlugin;

impl Plugin for LodManagerPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<LoadedLods>()
            .init_resource::<RequestedLods>()
            .init_resource::<RequestedChunks>()
            .init_resource::<Owners>()
            .register_chunk_consumer::<LodManagerConsumer>()
            .configure_sets(
                Update,
                (LodManagerSet::Collect, LodManagerSet::Request, LodManagerSet::Receive, LodManagerSet::Resolve, LodManagerSet::Retire).chain(),
            )
            .add_systems(Startup, spawn_lod_manager_consumer)
            .add_systems(Update, collect_request_deltas.in_set(LodManagerSet::Collect))
            .add_systems(Update, request_missing.in_set(LodManagerSet::Request))
            .add_systems(Update, receive_loaded_lods.in_set(LodManagerSet::Receive))
            .add_systems(Update, resolve_dirty.in_set(LodManagerSet::Resolve))
            .add_systems(Update, retire_unused.in_set(LodManagerSet::Retire));
    }
}

fn spawn_lod_manager_consumer(mut commands: Commands) {
    commands.spawn(LodManagerConsumer::default());
}

fn collect_request_deltas(mut owners: ResMut<Owners>, mut maps: Query<(Entity, &mut LodRequestMap)>, mut removed: RemovedComponents<LodRequestMap>) {
    for owner in removed.read() {
        owners.lods.retain(|_, set| {
            set.remove(&owner);
            !set.is_empty()
        });
        owners.chunks.retain(|_, set| {
            set.remove(&owner);
            !set.is_empty()
        });
    }

    for (owner, mut map) in maps.iter_mut() {
        for delta in map.drain_area_delta() {
            if let Some(level) = delta.old_lod {
                for key in keys_for_area(delta.grid, delta.min, delta.size, level) {
                    if level == 0 {
                        let id = (key.grid, key.min);
                        let empty = owners.chunks.get_mut(&id).is_some_and(|set| {
                            set.remove(&owner);
                            set.is_empty()
                        });
                        if empty {
                            owners.chunks.remove(&id);
                        }
                    } else {
                        let empty = owners.lods.get_mut(&key).is_some_and(|set| {
                            set.remove(&owner);
                            set.is_empty()
                        });
                        if empty {
                            owners.lods.remove(&key);
                        }
                    }
                }
            }
            if let Some(level) = delta.new_lod {
                for key in keys_for_area(delta.grid, delta.min, delta.size, level) {
                    if level == 0 {
                        owners.chunks.entry((key.grid, key.min)).or_default().insert(owner);
                    } else {
                        owners.lods.entry(key).or_default().insert(owner, map.priority());
                    }
                }
            }
            owners.dirty.push((owner, delta.grid, delta.min, delta.size));
        }
    }
}

fn request_missing(
    consumer: Option<Single<Entity, With<LodManagerConsumer>>>, lod_channel: Res<LodRequestChannel>, chunk_channel: Res<ChunkRequestChannel>,
    owners: Res<Owners>, loaded: Res<LoadedLods>, mut requested_lods: ResMut<RequestedLods>, mut requested_chunks: ResMut<RequestedChunks>,
    mut grids: Query<&mut GridStreaming>,
) {
    let Some(consumer) = consumer else {
        return;
    };
    let consumer = *consumer;

    for (&key, owner_priorities) in &owners.lods {
        if loaded.contains(&key) || requested_lods.0.contains(&key) {
            continue;
        }
        let Ok(streaming) = grids.get(key.grid) else {
            continue;
        };
        let priority = owner_priorities.values().copied().fold(f32::NEG_INFINITY, f32::max);
        if streaming.fetch_lod(key.grid, consumer, &lod_channel, key.min, key.size, key.lod(), priority) {
            requested_lods.0.insert(key);
        }
    }

    for &(grid, chunk) in owners.chunks.keys() {
        if requested_chunks.0.contains(&(grid, chunk)) {
            continue;
        }
        let Ok(mut streaming) = grids.get_mut(grid) else {
            continue;
        };
        streaming.fetch(grid, &chunk_channel, chunk);
        requested_chunks.0.insert((grid, chunk));
    }
}

fn receive_loaded_lods(
    mut commands: Commands, consumer: Option<Single<&mut LodManagerConsumer>>, mut owners: ResMut<Owners>, mut loaded: ResMut<LoadedLods>,
    mut requested: ResMut<RequestedLods>,
) {
    let Some(mut consumer) = consumer else {
        return;
    };
    for result in consumer.drain_lod() {
        let key = LodKey::new(result.grid, result.min, result.size, result.lod);
        requested.0.remove(&key);
        let Some(voxels) = result.voxels else {
            continue;
        };
        if let Some(previous) = loaded.remove(&key) {
            commands.entity(previous).despawn();
        }
        let entity = commands
            .spawn((
                LodVoxels { voxels, grid: result.grid, min: result.min, size: result.size, lod: result.lod, priority: result.priority },
                LodRetainCount::default(),
            ))
            .id();
        loaded.insert(key, entity);
        if let Some(owner_set) = owners.lods.get(&key) {
            let dirty: Vec<_> = owner_set.keys().map(|&owner| (owner, key.grid, key.min, key.size)).collect();
            owners.dirty.extend(dirty);
        }
    }
}

fn resolve_dirty(
    mut owners: ResMut<Owners>, mut maps: Query<(Entity, &mut LodRequestMap)>, loaded: Res<LoadedLods>, grids: Query<&Grid>,
    gpu: Query<&SubGridGpuState>,
) {
    let dirty = std::mem::take(&mut owners.dirty);
    for (owner, grid, min, size) in dirty {
        let Ok((_, mut map)) = maps.get_mut(owner) else {
            continue;
        };
        let Some(next) = visible_for_area(&map, grid, min, size, &loaded, &grids, &gpu) else {
            owners.dirty.push((owner, grid, min, size));
            continue;
        };
        map.replace_visible_in_area(grid, min, size, next);
    }
}

fn retire_unused(
    mut commands: Commands, owners: Res<Owners>, mut loaded: ResMut<LoadedLods>, mut requested_lods: ResMut<RequestedLods>,
    mut requested_chunks: ResMut<RequestedChunks>, mut grids: Query<&mut GridStreaming>, maps: Query<&LodRequestMap>,
) {
    requested_lods.0.retain(|key| owners.lods.contains_key(key));

    let visible_chunks: HashSet<_> =
        maps.iter().flat_map(|m| m.visible()).filter(|v| v.kind == LodVisibleKind::SubGrid).map(|v| (v.grid, v.min)).collect();
    let stale_chunks: Vec<_> =
        requested_chunks.0.iter().copied().filter(|key| !owners.chunks.contains_key(key) && !visible_chunks.contains(key)).collect();
    for (grid, chunk) in stale_chunks {
        requested_chunks.0.remove(&(grid, chunk));
        if let Ok(mut streaming) = grids.get_mut(grid) {
            streaming.release(chunk);
        }
    }

    let visible_lods: HashSet<_> = maps.iter().flat_map(|m| m.visible()).filter(|v| v.kind == LodVisibleKind::Lod).map(|v| v.entity).collect();
    let stale_lods: Vec<_> = loaded.keys().copied().filter(|key| !owners.lods.contains_key(key)).collect();
    for key in stale_lods {
        let Some(entity) = loaded.get(&key) else {
            continue;
        };
        if visible_lods.contains(&entity) {
            continue;
        }
        loaded.remove(&key);
        commands.entity(entity).despawn();
    }
}

fn visible_for_area(
    map: &LodRequestMap, grid: GridId, min: IVec3, size: IVec3, loaded: &LoadedLods, grids: &Query<&Grid>, gpu: &Query<&SubGridGpuState>,
) -> Option<Vec<LodVisibleDelta>> {
    let Some(tree) = map.tree(grid) else {
        return Some(Vec::new());
    };
    let mut out = Vec::new();
    let mut missing = false;
    tree.for_each_in_region(min, min + size - IVec3::ONE, |run_min, run_size, level| {
        let level = level as u32;
        for key in keys_for_area(grid, run_min, IVec3::splat(run_size as i32), level) {
            if level == 0 {
                let Ok(grid_data) = grids.get(grid) else {
                    missing = true;
                    continue;
                };
                let entities: Vec<_> = grid_data.subgrid_entities_in_area(key.min * CHUNK_SIZE, IVec3::splat(CHUNK_SIZE)).collect();
                if entities.is_empty() || entities.iter().any(|&e| gpu.get(e).is_err()) {
                    missing = true;
                    continue;
                }
                out.extend(entities.into_iter().map(|entity| LodVisibleDelta {
                    grid,
                    min: key.min,
                    size: IVec3::ONE,
                    requested_lod: 0,
                    actual_lod: 0,
                    entity,
                    kind: LodVisibleKind::SubGrid,
                }));
            } else {
                let Some(entity) = loaded.get(&key) else {
                    missing = true;
                    continue;
                };
                if gpu.get(entity).is_err() {
                    missing = true;
                    continue;
                }
                out.push(LodVisibleDelta {
                    grid,
                    min: key.min,
                    size: key.size,
                    requested_lod: level,
                    actual_lod: level,
                    entity,
                    kind: LodVisibleKind::Lod,
                });
            }
        }
    });
    (!missing).then_some(out)
}

fn keys_for_area(grid: GridId, min: IVec3, size: IVec3, level: u32) -> Vec<LodKey> {
    let step = if level == 0 { 1 } else { 1i32 << level };
    let hi = min + size;
    let lo = min.div_euclid(IVec3::splat(step)) * step;
    let tile = IVec3::splat(step);
    let mut keys = Vec::new();
    let mut x = lo.x;
    while x < hi.x {
        let mut y = lo.y;
        while y < hi.y {
            let mut z = lo.z;
            while z < hi.z {
                let p = IVec3::new(x, y, z);
                if (p + tile).cmpgt(min).all() {
                    keys.push(LodKey::from_level(grid, p, tile, level));
                }
                z += step;
            }
            y += step;
        }
        x += step;
    }
    keys
}
