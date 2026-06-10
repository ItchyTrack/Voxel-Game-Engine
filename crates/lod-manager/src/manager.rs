use std::collections::{HashMap, HashSet};

use bevy::prelude::*;
use gpu_voxel_data::{LodVoxels, SubGridGpuState};
use voxel_data::grid::{Grid, GridId};
use voxel_streaming::{ChunkConsumer, ChunkRequestChannel, GridStreaming, LodRequestChannel, VoxelStreamingAppExt, CHUNK_SIZE};

use crate::{LoadedLods, LodKey, LodRequestMap, LodRetainCount, LodVisibleDelta, LodVisibleKind};

voxel_streaming::chunk_consumer!(pub LodManagerConsumer);

const MAX_DIRTY_AREAS_PER_FRAME: usize = 128;

#[derive(Resource, Default)]
struct RequestedLods(HashSet<LodKey>);

#[derive(Resource, Default)]
struct RequestedChunks(HashSet<(GridId, IVec3)>);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct DirtyArea {
    owner: Entity,
    grid: GridId,
    min: IVec3,
    size: IVec3,
}

#[derive(Resource, Default)]
struct Owners {
    lods: HashMap<LodKey, HashMap<Entity, f32>>,
    chunks: HashMap<(GridId, IVec3), HashSet<Entity>>,
    dirty: HashSet<DirtyArea>,
    waiting_lods: HashMap<LodKey, HashSet<DirtyArea>>,
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
            .add_systems(Update, (resolve_dirty, promote_ready_lods).in_set(LodManagerSet::Resolve))
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
        owners.dirty.retain(|area| area.owner != owner);
        owners.waiting_lods.retain(|_, areas| {
            areas.retain(|area| area.owner != owner);
            !areas.is_empty()
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
            owners.dirty.insert(DirtyArea { owner, grid: delta.grid, min: delta.min, size: delta.size });
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
    }
}

fn resolve_dirty(
    mut owners: ResMut<Owners>, mut maps: Query<(Entity, &mut LodRequestMap)>, loaded: Res<LoadedLods>, grids: Query<&Grid>,
    gpu: Query<&SubGridGpuState>,
) {
    let mut dirty = std::mem::take(&mut owners.dirty).into_iter();
    for area in dirty.by_ref().take(MAX_DIRTY_AREAS_PER_FRAME) {
        let Ok((_, mut map)) = maps.get_mut(area.owner) else {
            continue;
        };
        match visible_for_area(&map, area.grid, area.min, area.size, &loaded, &grids, &gpu) {
            VisibleAreaResult::Ready(next) => map.replace_visible_in_area(area.grid, area.min, area.size, next),
            VisibleAreaResult::WaitingForLods(keys) => {
                for key in keys {
                    owners.waiting_lods.entry(key).or_default().insert(area);
                }
            }
            VisibleAreaResult::WaitingForOther => {
                owners.dirty.insert(area);
            }
        }
    }
    owners.dirty.extend(dirty);
}

fn promote_ready_lods(
    mut owners: ResMut<Owners>, loaded: Res<LoadedLods>, ready_lods: Query<(Entity, &LodVoxels), Added<SubGridGpuState>>,
    mut maps: Query<&mut LodRequestMap>,
) {
    for (entity, lod_voxels) in ready_lods.iter() {
        let key = LodKey::new(lod_voxels.grid, lod_voxels.min, lod_voxels.size, lod_voxels.lod);
        if loaded.get(&key) != Some(entity) {
            continue;
        }

        let visible = LodVisibleDelta {
            grid: key.grid,
            min: key.min,
            size: key.size,
            requested_lod: key.level,
            actual_lod: key.level,
            entity,
            kind: LodVisibleKind::Lod,
        };

        let mut owners_to_update: Vec<Entity> = owners.lods.get(&key).map(|set| set.keys().copied().collect()).unwrap_or_default();
        if let Some(waiting) = owners.waiting_lods.remove(&key) {
            owners_to_update.extend(waiting.into_iter().map(|area| area.owner));
        }
        owners_to_update.sort();
        owners_to_update.dedup();

        for owner in owners_to_update {
            let Ok(mut map) = maps.get_mut(owner) else { continue };
            let still_requested = map.tree(key.grid).and_then(|tree| tree.get(&key.min)).map(|level| level as u32) == Some(key.level);
            if still_requested {
                map.replace_visible_in_area(key.grid, key.min, key.size, vec![visible]);
            }
        }
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

enum VisibleAreaResult {
    Ready(Vec<LodVisibleDelta>),
    WaitingForLods(Vec<LodKey>),
    WaitingForOther,
}

fn visible_for_area(
    map: &LodRequestMap, grid: GridId, min: IVec3, size: IVec3, loaded: &LoadedLods, grids: &Query<&Grid>, gpu: &Query<&SubGridGpuState>,
) -> VisibleAreaResult {
    let Some(tree) = map.tree(grid) else {
        return VisibleAreaResult::Ready(Vec::new());
    };
    let mut out = Vec::new();
    let mut missing_lods = HashSet::new();
    let mut missing_other = false;
    tree.for_each_in_region(min, min + size - IVec3::ONE, |run_min, run_size, level| {
        let level = level as u32;
        for key in keys_for_area(grid, run_min, IVec3::splat(run_size as i32), level) {
            if level == 0 {
                let Ok(grid_data) = grids.get(grid) else {
                    missing_other = true;
                    continue;
                };
                let entities: Vec<_> = grid_data.subgrid_entities_in_area(key.min * CHUNK_SIZE, IVec3::splat(CHUNK_SIZE)).collect();
                if entities.is_empty() || entities.iter().any(|&e| gpu.get(e).is_err()) {
                    missing_other = true;
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
                    missing_lods.insert(key);
                    continue;
                };
                if gpu.get(entity).is_err() {
                    missing_lods.insert(key);
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
    if !missing_lods.is_empty() {
        VisibleAreaResult::WaitingForLods(missing_lods.into_iter().collect())
    } else if missing_other {
        VisibleAreaResult::WaitingForOther
    } else {
        VisibleAreaResult::Ready(out)
    }
}

fn keys_for_area(grid: GridId, min: IVec3, size: IVec3, level: u32) -> Vec<LodKey> {
    let mut keys = Vec::new();
    for x in 0..size.x {
        for y in 0..size.y {
            for z in 0..size.z {
                keys.push(LodKey::from_level(grid, min + IVec3::new(x, y, z), IVec3::ONE, level));
            }
        }
    }
    keys
}
