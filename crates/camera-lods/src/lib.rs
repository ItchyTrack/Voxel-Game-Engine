use std::collections::{HashMap, HashSet, VecDeque};

use bevy::prelude::*;
use gpu_voxel_data::{LodVoxels, SubGridGpuState};
use voxel_data::grid::{Grid, GridId};
use voxel_data::subgrid::SubGrid;
use voxel_streaming::ChunkConsumer;
use voxel_streaming::{ChunkRequestChannel, GridStreaming, LodRequestChannel, StreamingPhase, StreamingSchedule, VoxelStreamingAppExt, CHUNK_SIZE};

voxel_streaming::chunk_consumer!(pub CameraLodConsumer);

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct FreezeCameraLods(pub bool);

#[derive(Component, Debug, Clone)]
pub struct CameraLodController {
    pub max_lod: u8,
    pub near_radius_chunks: i32,
    pub rings_per_lod: i32,
    pub requests_per_frame: usize,
    pub max_in_flight: usize,
    state: CameraLodState,
}

impl Default for CameraLodController {
    fn default() -> Self {
        Self {
            max_lod: 3,
            near_radius_chunks: 3,
            rings_per_lod: 2,
            requests_per_frame: 16,
            max_in_flight: 128,
            state: CameraLodState::default(),
        }
    }
}

#[derive(Default, Debug, Clone)]
struct CameraLodState {
    desired_chunks: HashSet<ChunkKey>,
    desired_tiles: HashSet<TileKey>,
    retiring_chunks: HashSet<ChunkKey>,
    grid_centers: HashMap<GridId, IVec3>,
    queue: VecDeque<TileKey>,
    tiles: HashMap<TileKey, TileRecord>,
}

#[derive(Component, Default, Debug, Clone)]
pub struct LodRequestMap {
    visible: Vec<LodVisible>,
}

impl LodRequestMap {
    pub fn visible(&self) -> &[LodVisible] {
        &self.visible
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct LodVisible {
    pub entity: Entity,
    pub kind: LodVisibleKind,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum LodVisibleKind {
    SubGrid,
    Lod,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct ChunkKey {
    grid: GridId,
    chunk: IVec3,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct TileKey {
    grid: GridId,
    lod: u8,
    min: IVec3,
}

impl TileKey {
    fn size(self) -> IVec3 {
        IVec3::splat(1i32 << self.lod)
    }
}

#[derive(Debug, Clone, Copy)]
struct TileRecord {
    status: TileStatus,
    entity: Option<Entity>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TileStatus {
    Queued,
    Loading,
    LoadedWaitingGpu,
    Ready,
    Empty,
    Retiring,
}

#[derive(Default)]
pub struct CameraLodsPlugin;

impl Plugin for CameraLodsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<FreezeCameraLods>()
            .register_chunk_consumer::<CameraLodConsumer>()
            .add_systems(Update, ensure_camera_lod_components)
            .add_systems(
                StreamingSchedule,
                update_camera_lod_requests.run_if(|freeze: Res<FreezeCameraLods>| !freeze.0).in_set(StreamingPhase::Request),
            )
            .add_systems(StreamingSchedule, receive_camera_lod_results.after(voxel_streaming::receive_lod_results).in_set(StreamingPhase::Receive))
            .add_systems(
                Update,
                (refresh_camera_lod_visibility, retire_replaced_tiles, retire_replaced_chunks).chain().after(gpu_voxel_data::GpuUploadSet::Upload),
            );
            // .add_systems(Update, (draw_lod_bounds_gizmos, draw_retiring_lod_gizmos));
    }
}

fn ensure_camera_lod_components(mut commands: Commands, cameras: Query<Entity, (With<Camera3d>, Without<CameraLodController>)>) {
    for entity in &cameras {
        commands.entity(entity).insert((CameraLodController::default(), CameraLodConsumer::default(), LodRequestMap::default()));
    }
}

fn update_camera_lod_requests(
    chunk_channel: Res<ChunkRequestChannel>, lod_channel: Res<LodRequestChannel>,
    mut cameras: Query<(Entity, &Camera, &GlobalTransform, &mut CameraLodController, &mut CameraLodConsumer), With<Camera3d>>,
    mut grids: Query<(GridId, &GlobalTransform, &mut GridStreaming)>, grid_transforms: Query<&GlobalTransform, With<GridStreaming>>,
) {
    for (camera_entity, camera, camera_global, mut controller, mut consumer) in &mut cameras {
        if !camera.is_active {
            continue;
        }
        let camera_world = camera_global.translation();
        let mut desired_chunks = HashSet::new();
        let mut desired_tiles = HashSet::new();

        for (grid, grid_global, streaming) in &mut grids {
            let local = grid_global.affine().inverse().transform_point3(camera_world);
            let camera_chunk = nearest_chunk_center(local);
            controller.state.grid_centers.insert(grid, camera_chunk);
            add_near_chunks(&mut desired_chunks, grid, camera_chunk, &controller);
            add_lod_tiles(&mut desired_tiles, grid, camera_chunk, &controller, streaming.as_ref());
        }

        controller.state.desired_tiles = desired_tiles.clone();

        let chunks_to_fetch = update_desired_chunks(&mut controller.state, desired_chunks);
        for chunk_key in chunks_to_fetch {
            if let Ok((_, _, mut streaming)) = grids.get_mut(chunk_key.grid) {
                streaming.fetch_needed(chunk_key.grid, consumer.as_mut(), &chunk_channel, chunk_key.chunk);
            }
        }

        let old_tiles: Vec<TileKey> = controller.state.tiles.keys().copied().collect();
        for key in old_tiles {
            if desired_tiles.contains(&key) {
                if let Some(record) = controller.state.tiles.get_mut(&key) {
                    if record.status == TileStatus::Retiring {
                        record.status = if record.entity.is_some() { TileStatus::Ready } else { TileStatus::Empty };
                    }
                }
                continue;
            }

            handle_non_desired_tile(&mut controller.state, key);
        }

        for key in desired_tiles {
            if controller.state.tiles.contains_key(&key) {
                continue;
            }
            controller.state.tiles.insert(key, TileRecord { status: TileStatus::Queued, entity: None });
            controller.state.queue.push_back(key);
        }

        let mut sent = 0usize;
        while sent < controller.requests_per_frame && in_flight_count(&controller.state) < controller.max_in_flight {
            let Some(key) = controller.state.queue.pop_front() else {
                break;
            };
            if !matches!(controller.state.tiles.get(&key).map(|r| r.status), Some(TileStatus::Queued)) {
                continue;
            }
            let priority = tile_priority(camera_world, key, &grid_transforms);
            let requested = grids
                .get(key.grid)
                .map(|(_, _, streaming)| streaming.fetch_lod(key.grid, camera_entity, &lod_channel, key.min, key.size(), key.lod as f32, priority))
                .unwrap_or(false);
            if requested {
                if let Some(record) = controller.state.tiles.get_mut(&key) {
                    record.status = TileStatus::Loading;
                }
                sent += 1;
            } else if let Some(record) = controller.state.tiles.get_mut(&key) {
                record.status = TileStatus::Empty;
            }
        }
    }
}

fn nearest_chunk_center(local_voxels: Vec3) -> IVec3 {
    (local_voxels / CHUNK_SIZE as f32).round().as_ivec3()
}

fn add_near_chunks(out: &mut HashSet<ChunkKey>, grid: GridId, center: IVec3, controller: &CameraLodController) {
    let r = controller.near_radius_chunks;
    for x in -r..=r {
        for y in -r..=r {
            for z in -r..=r {
                out.insert(ChunkKey { grid, chunk: center + IVec3::new(x, y, z) });
            }
        }
    }
}

fn add_lod_tiles(out: &mut HashSet<TileKey>, grid: GridId, center: IVec3, controller: &CameraLodController, streaming: &GridStreaming) {
    if presence_bounds(streaming).is_none() { return; }

    let mut inner = controller.near_radius_chunks + 1;
    for lod in 1..=controller.max_lod {
        let tile_size = 1i32 << lod;
        let outer = inner + controller.rings_per_lod * tile_size;
        let lod_min = center - IVec3::splat(outer);
        let lod_max = center + IVec3::splat(outer);
        let min_tile = align_chunk_to_tile(lod_min, tile_size);
        let max_tile = align_chunk_to_tile(lod_max, tile_size);
        let band_inner = if lod == 1 { 0 } else { inner };
        for x in (min_tile.x..=max_tile.x).step_by(tile_size as usize) {
            for y in (min_tile.y..=max_tile.y).step_by(tile_size as usize) {
                for z in (min_tile.z..=max_tile.z).step_by(tile_size as usize) {
                    let min = IVec3::new(x, y, z);
                    if tile_intersects_ring(center, min, tile_size, band_inner, outer)
                        && !tile_inside_near_box(center, min, tile_size, controller.near_radius_chunks)
                        && tile_has_present_chunk(streaming, min, tile_size)
                    {
                        out.insert(TileKey { grid, lod, min });
                    }
                }
            }
        }
        inner = outer + 1;
    }
}

fn presence_bounds(streaming: &GridStreaming) -> Option<(IVec3, IVec3)> {
    let mut min = IVec3::splat(i32::MAX);
    let mut max = IVec3::splat(i32::MIN);
    let mut any = false;
    for (origin, size) in streaming.presence().iter_present() {
        any = true;
        min = min.min(origin);
        max = max.max(origin + IVec3::splat(size as i32) - IVec3::ONE);
    }
    any.then_some((min, max))
}

fn tile_has_present_chunk(streaming: &GridStreaming, min: IVec3, tile_size: i32) -> bool {
    let max = min + IVec3::splat(tile_size) - IVec3::ONE;
    let mut any = false;
    streaming.presence().for_each_in_region(min, max, |_| {
        any = true;
    });
    any
}

fn align_chunk_to_tile(chunk: IVec3, tile_size: i32) -> IVec3 {
    chunk.div_euclid(IVec3::splat(tile_size)) * tile_size
}

fn axis_min_abs_delta(center: i32, lo: i32, hi: i32) -> i32 {
    if center < lo {
        lo - center
    } else if center > hi {
        center - hi
    } else {
        0
    }
}

fn tile_intersects_ring(center: IVec3, min: IVec3, tile_size: i32, inner: i32, outer: i32) -> bool {
    let max = min + IVec3::splat(tile_size - 1);
    let min_dist = axis_min_abs_delta(center.x, min.x, max.x)
        .max(axis_min_abs_delta(center.y, min.y, max.y))
        .max(axis_min_abs_delta(center.z, min.z, max.z));
    let max_dist = (min.x - center.x)
        .abs()
        .max((max.x - center.x).abs())
        .max((min.y - center.y).abs())
        .max((max.y - center.y).abs())
        .max((min.z - center.z).abs())
        .max((max.z - center.z).abs());
    max_dist >= inner && min_dist <= outer
}

fn tile_inside_near_box(center: IVec3, min: IVec3, tile_size: i32, near_radius: i32) -> bool {
    let max = min + IVec3::splat(tile_size - 1);
    (min.x - center.x).abs().max((max.x - center.x).abs()) <= near_radius
        && (min.y - center.y).abs().max((max.y - center.y).abs()) <= near_radius
        && (min.z - center.z).abs().max((max.z - center.z).abs()) <= near_radius
}

fn in_flight_count(state: &CameraLodState) -> usize {
    state.tiles.values().filter(|r| r.status == TileStatus::Loading).count()
}

fn update_desired_chunks(state: &mut CameraLodState, desired_chunks: HashSet<ChunkKey>) -> Vec<ChunkKey> {
    let old_chunks = state.desired_chunks.clone();
    for &chunk_key in old_chunks.difference(&desired_chunks) {
        state.retiring_chunks.insert(chunk_key);
    }

    let chunks_to_fetch: Vec<ChunkKey> = desired_chunks.difference(&old_chunks).copied().collect();
    for chunk_key in &chunks_to_fetch {
        state.retiring_chunks.remove(chunk_key);
    }
    state.desired_chunks = desired_chunks;
    chunks_to_fetch
}

fn handle_non_desired_tile(state: &mut CameraLodState, key: TileKey) {
    let Some(record) = state.tiles.get_mut(&key) else { return };
    match record.status {
        // Already-renderable tiles must stay visible until replacement coverage is ready.
        TileStatus::Ready | TileStatus::Retiring => {
            record.status = TileStatus::Retiring;
        }
        // In-flight work may still produce valid voxel data. Keep the record so the
        // late result can be applied, then retire it through the same no-gap path.
        TileStatus::Loading | TileStatus::LoadedWaitingGpu => {}
        // Queued work has not been sent yet, so it can be safely dropped.
        TileStatus::Queued => {
            state.tiles.remove(&key);
        }
        // Empty records render nothing. Drop them instead of treating them as coverage.
        TileStatus::Empty => {
            state.tiles.remove(&key);
        }
    }
}

fn tile_priority(camera_world: Vec3, key: TileKey, grid_transforms: &Query<&GlobalTransform, With<GridStreaming>>) -> f32 {
    let Ok(grid_global) = grid_transforms.get(key.grid) else { return 0.0 };
    let center_local = ((key.min + key.size() / 2) * CHUNK_SIZE).as_vec3();
    let center_world = grid_global.transform_point(center_local);
    -camera_world.distance(center_world)
}

fn receive_camera_lod_results(mut commands: Commands, mut cameras: Query<&mut CameraLodController>, mut consumers: Query<&mut CameraLodConsumer>) {
    for mut consumer in &mut consumers {
        let results = consumer.drain_lod();
        if results.is_empty() {
            continue;
        }
        // The consumer lives on the same entity as the controller.
        // Query iteration order is not reliable, so use the requester embedded in each result.
        for result in results {
            let Ok(mut controller) = cameras.get_mut(result.requester) else { continue };
            let key = TileKey { grid: result.grid, lod: result.lod.max(0.0).floor() as u8, min: result.min };
            let Some(record) = controller.state.tiles.get_mut(&key) else { continue };
            if record.entity.is_some() {
                continue;
            }
            match result.voxels {
                Some(voxels) if !voxels.is_empty() => {
                    let entity = commands
                        .spawn((
                            LodVoxels { voxels, grid: result.grid, min: result.min, size: result.size, lod: result.lod, priority: result.priority },
                            ChildOf(result.grid),
                        ))
                        .id();
                    record.entity = Some(entity);
                    record.status = TileStatus::LoadedWaitingGpu;
                }
                _ => {
                    record.status = TileStatus::Empty;
                }
            }
        }
    }
}

fn refresh_camera_lod_visibility(
    mut cameras: Query<(&mut CameraLodController, &mut LodRequestMap)>, lod_gpu: Query<&SubGridGpuState, With<LodVoxels>>, grids: Query<&Grid>,
    subgrids: Query<&SubGrid>,
) {
    for (mut controller, mut request_map) in &mut cameras {
        for record in controller.state.tiles.values_mut() {
            if record.status == TileStatus::LoadedWaitingGpu && record.entity.is_some_and(|e| lod_gpu.get(e).is_ok()) {
                record.status = TileStatus::Ready;
            }
        }

        let mut visible = Vec::new();
        let mut seen = HashSet::new();
        for chunk in controller.state.desired_chunks.iter().chain(controller.state.retiring_chunks.iter()) {
            let Ok(grid) = grids.get(chunk.grid) else { continue };
            let min = chunk.chunk * CHUNK_SIZE;
            for entity in grid.subgrid_entities_in_area(min, IVec3::splat(CHUNK_SIZE)) {
                if subgrids.get(entity).is_ok() && seen.insert((entity, LodVisibleKind::SubGrid)) {
                    visible.push(LodVisible { entity, kind: LodVisibleKind::SubGrid });
                }
            }
        }

        for (key, record) in &controller.state.tiles {
            let desired_or_retiring = record.status == TileStatus::Retiring || !matches!(record.status, TileStatus::Queued | TileStatus::Loading);
            if !desired_or_retiring {
                continue;
            }
            if record.status == TileStatus::Ready {
                if let Some(entity) = record.entity {
                    if seen.insert((entity, LodVisibleKind::Lod)) {
                        visible.push(LodVisible { entity, kind: LodVisibleKind::Lod });
                    }
                }
            } else if record.status == TileStatus::Retiring {
                if let Some(entity) = record.entity {
                    if seen.insert((entity, LodVisibleKind::Lod)) {
                        visible.push(LodVisible { entity, kind: LodVisibleKind::Lod });
                    }
                }
            }
            let _ = key;
        }
        request_map.visible = visible;
    }
}

fn retire_replaced_tiles(mut commands: Commands, mut cameras: Query<&mut CameraLodController>) {
    for mut controller in &mut cameras {
        let retiring: Vec<TileKey> =
            controller.state.tiles.iter().filter_map(|(key, record)| (record.status == TileStatus::Retiring).then_some(*key)).collect();
        for key in retiring {
            if !area_is_covered_by_ready_desired(&controller.state, key) {
                continue;
            }
            if let Some(record) = controller.state.tiles.remove(&key) {
                if let Some(entity) = record.entity {
                    commands.entity(entity).despawn();
                }
            }
        }
    }
}

fn retire_replaced_chunks(mut cameras: Query<(&mut CameraLodController, &mut CameraLodConsumer)>, mut grids: Query<&mut GridStreaming>) {
    for (mut controller, mut consumer) in &mut cameras {
        let max_lod = controller.max_lod;
        let retiring = std::mem::take(&mut controller.state.retiring_chunks);
        let mut still_retiring = HashSet::new();

        for chunk in retiring {
            if !chunk_is_covered_by_ready_tile(&controller.state, chunk, max_lod) {
                still_retiring.insert(chunk);
                continue;
            }
            if let Ok(mut streaming) = grids.get_mut(chunk.grid) {
                streaming.release_needed(chunk.grid, consumer.as_mut(), chunk.chunk);
            }
        }

        controller.state.retiring_chunks = still_retiring;
    }
}

fn chunk_is_covered_by_ready_tile(state: &CameraLodState, chunk: ChunkKey, max_lod: u8) -> bool {
    (1..=max_lod).any(|lod| {
        let tile_size = 1i32 << lod;
        let key = TileKey { grid: chunk.grid, lod, min: align_chunk_to_tile(chunk.chunk, tile_size) };
        state.tiles.get(&key).is_some_and(|record| record.status == TileStatus::Ready)
    })
}

fn draw_lod_bounds_gizmos(
    mut gizmos: Gizmos,
    cameras: Query<&CameraLodController, With<Camera3d>>,
    grids: Query<(GridId, &GlobalTransform), With<GridStreaming>>,
) {
    for controller in &cameras {
        for (grid, grid_transform) in &grids {
            if let Some((min_chunk, max_chunk)) = lod0_chunk_bounds(&controller.state, grid) {
                draw_lod_bound_box(&mut gizmos, grid_transform, min_chunk, max_chunk, lod_bound_color(0, controller.max_lod));
            }

            for key in controller.state.desired_tiles.iter().filter(|key| key.grid == grid) {
                draw_lod_bound_box(&mut gizmos, grid_transform, key.min, key.min + key.size(), lod_bound_color(key.lod, controller.max_lod));
            }
        }
    }
}

fn lod0_chunk_bounds(state: &CameraLodState, grid: GridId) -> Option<(IVec3, IVec3)> {
    let mut min = IVec3::splat(i32::MAX);
    let mut max = IVec3::splat(i32::MIN);
    let mut any = false;
    for chunk in state.desired_chunks.iter().filter(|chunk| chunk.grid == grid) {
        any = true;
        min = min.min(chunk.chunk);
        max = max.max(chunk.chunk + IVec3::ONE);
    }
    any.then_some((min, max))
}

fn draw_lod_bound_box(gizmos: &mut Gizmos, grid_transform: &GlobalTransform, min_chunk: IVec3, max_chunk: IVec3, color: Color) {
    draw_transformed_box(
        gizmos,
        grid_transform,
        (min_chunk * CHUNK_SIZE).as_vec3(),
        (max_chunk * CHUNK_SIZE).as_vec3(),
        color,
    );
}

fn lod_bound_color(lod: u8, max_lod: u8) -> Color {
    let t = if max_lod == 0 { 0.0 } else { lod as f32 / max_lod as f32 };
    Color::srgba(0.05 + 0.25 * t, 0.20 + 0.45 * t, 1.0, 0.45 + 0.35 * (1.0 - t))
}

#[cfg(test)]
fn lod_outer_radius_chunks(controller: &CameraLodController) -> i32 {
    let mut inner = controller.near_radius_chunks + 1;
    let mut outer = inner;
    for lod in 1..=controller.max_lod {
        outer = inner + controller.rings_per_lod * (1i32 << lod);
        inner = outer + 1;
    }
    outer
}

fn draw_retiring_lod_gizmos(
    mut gizmos: Gizmos,
    cameras: Query<&CameraLodController>,
    grid_transforms: Query<&GlobalTransform, With<GridStreaming>>,
) {
    let color = Color::srgba(1.0, 0.15, 0.0, 0.9);
    for controller in &cameras {
        for (key, record) in &controller.state.tiles {
            if record.status != TileStatus::Retiring {
                continue;
            }
            let Ok(grid_transform) = grid_transforms.get(key.grid) else { continue };
            let min = (key.min * CHUNK_SIZE).as_vec3();
            let max = ((key.min + key.size()) * CHUNK_SIZE).as_vec3();
            draw_transformed_box(&mut gizmos, grid_transform, min, max, color);
        }
    }
}

fn draw_transformed_box(gizmos: &mut Gizmos, transform: &GlobalTransform, min: Vec3, max: Vec3, color: Color) {
    let corner = |x: f32, y: f32, z: f32| transform.transform_point(Vec3::new(x, y, z));
    let c000 = corner(min.x, min.y, min.z);
    let c100 = corner(max.x, min.y, min.z);
    let c010 = corner(min.x, max.y, min.z);
    let c001 = corner(min.x, min.y, max.z);
    let c110 = corner(max.x, max.y, min.z);
    let c101 = corner(max.x, min.y, max.z);
    let c011 = corner(min.x, max.y, max.z);
    let c111 = corner(max.x, max.y, max.z);

    for (a, b) in [
        (c000, c100),
        (c010, c110),
        (c001, c101),
        (c011, c111),
        (c000, c010),
        (c100, c110),
        (c001, c011),
        (c101, c111),
        (c000, c001),
        (c100, c101),
        (c010, c011),
        (c110, c111),
    ] {
        gizmos.line(a, b, color);
    }
}

fn area_is_covered_by_ready_desired(state: &CameraLodState, old: TileKey) -> bool {
    let max_lod = state.tiles.keys().map(|key| key.lod).max().unwrap_or(old.lod);
    let old_min = old.min;
    let old_max = old.min + old.size();
    for x in old_min.x..old_max.x {
        for y in old_min.y..old_max.y {
            for z in old_min.z..old_max.z {
                let chunk = IVec3::new(x, y, z);
                if state.desired_chunks.contains(&ChunkKey { grid: old.grid, chunk }) {
                    continue;
                }
                let chunk_key = ChunkKey { grid: old.grid, chunk };
                if !chunk_is_wanted_by_non_retiring_tile_except(state, chunk_key, old) {
                    continue;
                }
                if !chunk_is_covered_by_ready_tile_except(state, chunk_key, old, max_lod) {
                    return false;
                }
            }
        }
    }
    true
}

fn chunk_is_covered_by_ready_tile_except(state: &CameraLodState, chunk: ChunkKey, except: TileKey, max_lod: u8) -> bool {
    (1..=max_lod).any(|lod| {
        let tile_size = 1i32 << lod;
        let key = TileKey { grid: chunk.grid, lod, min: align_chunk_to_tile(chunk.chunk, tile_size) };
        key != except && state.tiles.get(&key).is_some_and(|record| record.status == TileStatus::Ready)
    })
}

fn chunk_is_wanted_by_non_retiring_tile_except(state: &CameraLodState, chunk: ChunkKey, except: TileKey) -> bool {
    state.tiles.iter().any(|(key, record)| {
        *key != except
            && key.grid == chunk.grid
            && record.status != TileStatus::Retiring
            && chunk.chunk.cmpge(key.min).all()
            && chunk.chunk.cmplt(key.min + key.size()).all()
    })
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::*;

    #[test]
    fn aligns_negative_chunks_to_lod_tile() {
        assert_eq!(align_chunk_to_tile(IVec3::new(-1, -2, -3), 4), IVec3::new(-4, -4, -4));
        assert_eq!(align_chunk_to_tile(IVec3::new(4, 5, 7), 4), IVec3::new(4, 4, 4));
    }

    #[test]
    fn tile_ring_intersection_uses_tile_extent_not_center() {
        assert!(tile_intersects_ring(IVec3::ZERO, IVec3::new(-4, 0, 0), 2, 4, 8));
        assert!(!tile_intersects_ring(IVec3::ZERO, IVec3::new(-2, 0, 0), 2, 4, 8));
    }

    #[test]
    fn camera_chunk_center_switches_at_chunk_midpoint() {
        assert_eq!(nearest_chunk_center(Vec3::new(1.4 * CHUNK_SIZE as f32, 0.0, 0.0)), IVec3::new(1, 0, 0));
        assert_eq!(nearest_chunk_center(Vec3::new(1.6 * CHUNK_SIZE as f32, 0.0, 0.0)), IVec3::new(2, 0, 0));
    }

    #[test]
    fn near_chunk_leaving_desired_set_is_not_safe_to_release_without_replacement() {
        let grid = Entity::PLACEHOLDER;
        let chunk = ChunkKey { grid, chunk: IVec3::ZERO };

        let mut state = CameraLodState::default();
        state.desired_chunks = HashSet::from([chunk]);

        let to_fetch = update_desired_chunks(&mut state, HashSet::new());

        // Desired behavior: a near chunk remains tracked as retiring until a ready
        // LOD tile covers the same space. It should not be released immediately.
        assert!(to_fetch.is_empty());
        assert!(state.retiring_chunks.contains(&chunk), "dropped chunk was not kept for no-gap retirement");
    }

    #[test]
    fn empty_lod_tile_is_not_renderable_replacement_coverage() {
        let grid = Entity::PLACEHOLDER;
        let old = TileKey { grid, lod: 1, min: IVec3::ZERO };
        let mut state = CameraLodState::default();
        state.tiles.insert(old, TileRecord { status: TileStatus::Retiring, entity: Some(Entity::PLACEHOLDER) });
        for x in 0..2 {
            for y in 0..2 {
                for z in 0..2 {
                    state.tiles.insert(TileKey { grid, lod: 0, min: IVec3::new(x, y, z) }, TileRecord { status: TileStatus::Empty, entity: None });
                }
            }
        }

        // Empty tiles have no LodVoxels entity and render nothing, so they must not
        // be allowed to retire visible old coverage. If they do, the old tile is
        // removed and the area becomes a persistent hole.
        assert!(!area_is_covered_by_ready_desired(&state, old), "empty tile with no render entity was treated as replacement coverage");
    }

    #[test]
    fn in_flight_tile_that_temporarily_leaves_view_must_keep_record_for_late_result() {
        let grid = Entity::PLACEHOLDER;
        let key = TileKey { grid, lod: 2, min: IVec3::ZERO };
        let mut state = CameraLodState::default();
        state.tiles.insert(key, TileRecord { status: TileStatus::Loading, entity: None });

        handle_non_desired_tile(&mut state, key);

        // A load result can arrive after movement changes the desired set. If the
        // record was removed, receive_camera_lod_results will ignore valid voxel
        // data for this tile, leaving the area with no loaded replacement.
        assert!(state.tiles.contains_key(&key), "in-flight tile record was removed before its result could be applied");
    }

    #[test]
    fn lod_tiles_cover_present_chunks_at_near_ring_boundary() {
        let grid = Entity::PLACEHOLDER;
        let center = IVec3::ZERO;
        let controller = CameraLodController { max_lod: 1, ..Default::default() };
        let mut streaming = GridStreaming::default();
        streaming.presence_mut().mark_present_area(IVec3::splat(-8), IVec3::splat(17));

        let mut desired_chunks = HashSet::new();
        let mut desired_tiles = HashSet::new();
        add_near_chunks(&mut desired_chunks, grid, center, &controller);
        add_lod_tiles(&mut desired_tiles, grid, center, &controller, &streaming);

        let boundary_chunk = ChunkKey { grid, chunk: IVec3::new(-4, 0, 0) };
        let covered_by_near = desired_chunks.contains(&boundary_chunk);
        let covered_by_lod = desired_tiles.iter().any(|tile| tile_covers_chunk(*tile, boundary_chunk.chunk));

        assert!(
            covered_by_near || covered_by_lod,
            "present chunk {:?} is outside near chunks but not covered by any requested LOD tile; tiles={desired_tiles:?}",
            boundary_chunk.chunk,
        );
    }

    #[test]
    fn moving_camera_lod_requests_are_set_deltas_and_keep_chunk_coverage() {
        let grid = Entity::PLACEHOLDER;
        let controller = CameraLodController { max_lod: 2, ..Default::default() };
        let mut streaming = GridStreaming::default();
        streaming.presence_mut().mark_present_area(IVec3::splat(-48), IVec3::splat(97));

        let centers = [
            IVec3::ZERO,
            IVec3::new(1, 0, 0),
            IVec3::new(2, 0, 0),
            IVec3::new(4, 0, 0),
            IVec3::new(7, 0, -3),
            IVec3::new(-5, 0, 6),
        ];

        let mut previous_tiles = HashSet::new();
        for center in centers {
            let mut desired_chunks = HashSet::new();
            let mut desired_tiles = HashSet::new();
            add_near_chunks(&mut desired_chunks, grid, center, &controller);
            add_lod_tiles(&mut desired_tiles, grid, center, &controller, &streaming);

            let expected_new: HashSet<_> = desired_tiles.difference(&previous_tiles).copied().collect();
            let expected_dropped: HashSet<_> = previous_tiles.difference(&desired_tiles).copied().collect();
            let unchanged: HashSet<_> = desired_tiles.intersection(&previous_tiles).copied().collect();

            assert!(expected_new.is_disjoint(&unchanged));
            assert!(expected_dropped.is_disjoint(&desired_tiles));
            assert_all_present_chunks_in_lod_range_are_covered(grid, center, &controller, &streaming, &desired_chunks, &desired_tiles);

            // Simulate the request scheduler's stable-state decision: only tiles in
            // desired - previous should become new requests for this camera movement.
            let scheduler_new: HashSet<_> = desired_tiles.iter().filter(|tile| !previous_tiles.contains(tile)).copied().collect();
            assert_eq!(scheduler_new, expected_new, "wrong LOD request delta at center {center:?}");

            previous_tiles = desired_tiles;
        }
    }

    #[test]
    fn crossing_lod_tile_midpoint_changes_desired_delta_even_inside_same_chunk() {
        let grid = Entity::PLACEHOLDER;
        let controller = CameraLodController { max_lod: 1, near_radius_chunks: 1, rings_per_lod: 1, ..Default::default() };
        let mut streaming = GridStreaming::default();
        streaming.presence_mut().mark_present_area(IVec3::new(0, 0, 0), IVec3::new(6, 1, 1));

        // Both positions are inside chunk 1, but they cross the midpoint between
        // the previous LOD tile center and the next. Delta math should shift the
        // LOD0 window from A+B toward B+C without waiting for a whole chunk step.
        let before_center = nearest_chunk_center(Vec3::new(1.4 * CHUNK_SIZE as f32, 0.0, 0.0));
        let after_center = nearest_chunk_center(Vec3::new(1.6 * CHUNK_SIZE as f32, 0.0, 0.0));

        let mut before_chunks = HashSet::new();
        let mut before_tiles = HashSet::new();
        add_near_chunks(&mut before_chunks, grid, before_center, &controller);
        add_lod_tiles(&mut before_tiles, grid, before_center, &controller, &streaming);

        let mut after_chunks = HashSet::new();
        let mut after_tiles = HashSet::new();
        add_near_chunks(&mut after_chunks, grid, after_center, &controller);
        add_lod_tiles(&mut after_tiles, grid, after_center, &controller, &streaming);

        let added_chunks: HashSet<_> = after_chunks.difference(&before_chunks).copied().collect();
        let removed_chunks: HashSet<_> = before_chunks.difference(&after_chunks).copied().collect();
        let added_tiles: HashSet<_> = after_tiles.difference(&before_tiles).copied().collect();
        let removed_tiles: HashSet<_> = before_tiles.difference(&after_tiles).copied().collect();

        assert!(
            !added_chunks.is_empty() || !removed_chunks.is_empty() || !added_tiles.is_empty() || !removed_tiles.is_empty(),
            "crossing the tile midpoint produced no request delta: before_center={before_center:?} after_center={after_center:?}"
        );
    }

    #[test]
    fn old_lod_tiles_delete_after_camera_moves_and_new_results_are_ready() {
        let grid = Entity::PLACEHOLDER;
        let controller = CameraLodController { max_lod: 2, ..Default::default() };
        let mut streaming = GridStreaming::default();
        streaming.presence_mut().mark_present_area(IVec3::splat(-48), IVec3::splat(97));

        let centers = [IVec3::ZERO, IVec3::new(16, 0, 0), IVec3::new(-16, 0, 16), IVec3::ZERO];
        let mut state = CameraLodState::default();

        for center in centers {
            let mut desired_chunks = HashSet::new();
            let mut desired_tiles = HashSet::new();
            add_near_chunks(&mut desired_chunks, grid, center, &controller);
            add_lod_tiles(&mut desired_tiles, grid, center, &controller, &streaming);
            update_desired_chunks(&mut state, desired_chunks);

            let old_tiles: Vec<TileKey> = state.tiles.keys().copied().collect();
            for key in old_tiles {
                if !desired_tiles.contains(&key) {
                    handle_non_desired_tile(&mut state, key);
                }
            }
            for tile in desired_tiles {
                state.tiles.entry(tile).or_insert(TileRecord { status: TileStatus::Loading, entity: None });
            }

            // Provide successful load + GPU-ready results for all current requests.
            for record in state.tiles.values_mut() {
                if matches!(record.status, TileStatus::Loading | TileStatus::LoadedWaitingGpu) {
                    record.status = TileStatus::Ready;
                    record.entity = Some(Entity::PLACEHOLDER);
                }
            }
            delete_replaced_tiles_for_test(&mut state);
        }

        let stuck: Vec<_> = state
            .tiles
            .iter()
            .filter_map(|(key, record)| (record.status == TileStatus::Retiring).then_some(*key))
            .collect();
        assert!(stuck.is_empty(), "old LOD tiles stayed retiring after replacement results were ready: {stuck:?}");
    }

    fn delete_replaced_tiles_for_test(state: &mut CameraLodState) {
        let retiring: Vec<_> = state
            .tiles
            .iter()
            .filter_map(|(key, record)| (record.status == TileStatus::Retiring).then_some(*key))
            .collect();
        for key in retiring {
            if area_is_covered_by_ready_desired(state, key) {
                state.tiles.remove(&key);
            }
        }
    }

    #[test]
    fn retiring_chunks_resolve_after_ready_lods_at_multiple_camera_positions() {
        let grid = Entity::PLACEHOLDER;
        let controller = CameraLodController { max_lod: 2, ..Default::default() };
        let mut streaming = GridStreaming::default();
        streaming.presence_mut().mark_present_area(IVec3::splat(-48), IVec3::splat(97));

        let centers = [
            IVec3::ZERO,
            IVec3::new(4, 0, 0),
            IVec3::new(8, 0, 0),
            IVec3::new(8, 0, 6),
            IVec3::new(-4, 0, 6),
            IVec3::ZERO,
        ];

        let mut state = CameraLodState::default();
        for center in centers {
            let mut desired_chunks = HashSet::new();
            let mut desired_tiles = HashSet::new();
            add_near_chunks(&mut desired_chunks, grid, center, &controller);
            add_lod_tiles(&mut desired_tiles, grid, center, &controller, &streaming);

            update_desired_chunks(&mut state, desired_chunks);
            state.tiles.retain(|key, _| desired_tiles.contains(key));
            for tile in desired_tiles {
                state.tiles.insert(tile, TileRecord { status: TileStatus::Ready, entity: Some(Entity::PLACEHOLDER) });
            }

            resolve_retiring_chunks_for_test(&mut state, controller.max_lod);
            assert!(
                state.retiring_chunks.is_empty(),
                "retiring chunks did not resolve at center {center:?}; count={} sample={:?}",
                state.retiring_chunks.len(),
                state.retiring_chunks.iter().take(20).collect::<Vec<_>>(),
            );
        }
    }

    fn resolve_retiring_chunks_for_test(state: &mut CameraLodState, max_lod: u8) {
        state.retiring_chunks = state
            .retiring_chunks
            .iter()
            .copied()
            .filter(|chunk| !chunk_is_covered_by_ready_tile(state, *chunk, max_lod))
            .collect();
    }

    #[test]
    fn retiring_chunks_survive_empty_lod_results_and_resolve_when_real_lods_arrive() {
        let grid = Entity::PLACEHOLDER;
        let controller = CameraLodController { max_lod: 2, ..Default::default() };
        let mut streaming = GridStreaming::default();
        streaming.presence_mut().mark_present_area(IVec3::splat(-48), IVec3::splat(97));

        let mut state = CameraLodState::default();
        let center_a = IVec3::ZERO;
        let center_b = IVec3::new(8, 0, 0);

        let mut desired_a = HashSet::new();
        add_near_chunks(&mut desired_a, grid, center_a, &controller);
        update_desired_chunks(&mut state, desired_a);

        let mut desired_b = HashSet::new();
        let mut tiles_b = HashSet::new();
        add_near_chunks(&mut desired_b, grid, center_b, &controller);
        add_lod_tiles(&mut tiles_b, grid, center_b, &controller, &streaming);
        update_desired_chunks(&mut state, desired_b);

        let retiring_before_results = state.retiring_chunks.clone();
        assert!(!retiring_before_results.is_empty(), "movement should create retiring chunks");

        // Simulate some LOD requests returning empty first. Empty records must not
        // release valid retiring chunks, because they render no replacement voxels.
        for tile in &tiles_b {
            state.tiles.insert(*tile, TileRecord { status: TileStatus::Empty, entity: None });
        }
        resolve_retiring_chunks_for_test(&mut state, controller.max_lod);
        assert_eq!(
            state.retiring_chunks, retiring_before_results,
            "empty LOD results retired chunks even though no renderable replacement exists",
        );

        // Later, real non-empty LOD results arrive for the same requested tiles.
        for tile in &tiles_b {
            state.tiles.insert(*tile, TileRecord { status: TileStatus::Ready, entity: Some(Entity::PLACEHOLDER) });
        }
        resolve_retiring_chunks_for_test(&mut state, controller.max_lod);
        assert!(
            state.retiring_chunks.is_empty(),
            "retiring chunks did not resolve after real LOD replacements became ready; sample={:?}",
            state.retiring_chunks.iter().take(20).collect::<Vec<_>>(),
        );
    }

    #[test]
    fn church_presence_chunks_all_have_camera_lod_requests() {
        let grid = Entity::PLACEHOLDER;
        let chunks = load_church_chunks();
        assert!(!chunks.is_empty(), "test fixture did not load any church chunks");

        let mut streaming = GridStreaming::default();
        for chunk in &chunks {
            streaming.presence_mut().mark_present(*chunk);
        }

        // Matches voxel-app scene setup: church grid is under a parent translated to y=-350,
        // while the camera spawns at world (0, 0, 60).
        let camera_chunk_in_church_grid = voxel_streaming::chunk_of(IVec3::new(0, 350, 60));
        let controller = CameraLodController::default();
        let mut desired_chunks = HashSet::new();
        let mut desired_tiles = HashSet::new();
        add_near_chunks(&mut desired_chunks, grid, camera_chunk_in_church_grid, &controller);
        add_lod_tiles(&mut desired_tiles, grid, camera_chunk_in_church_grid, &controller, &streaming);

        let missing: Vec<_> = chunks
            .iter()
            .copied()
            .filter(|chunk| {
                !desired_chunks.contains(&ChunkKey { grid, chunk: *chunk })
                    && !desired_tiles.iter().any(|tile| tile.grid == grid && tile_covers_chunk(*tile, *chunk))
            })
            .collect();

        assert!(
            missing.is_empty(),
            "{} church chunks have no near or LOD request; camera_chunk={camera_chunk_in_church_grid:?}; sample={:?}",
            missing.len(),
            &missing[..missing.len().min(20)],
        );
    }

    fn tile_covers_chunk(tile: TileKey, chunk: IVec3) -> bool {
        chunk.cmpge(tile.min).all() && chunk.cmplt(tile.min + tile.size()).all()
    }

    fn lod_outer_radius(controller: &CameraLodController) -> i32 {
        let mut inner = controller.near_radius_chunks + 1;
        let mut outer = inner;
        for lod in 1..=controller.max_lod {
            let tile_size = 1i32 << lod;
            outer = inner + controller.rings_per_lod * tile_size;
            inner = outer + 1;
        }
        outer
    }

    fn assert_all_present_chunks_in_lod_range_are_covered(
        grid: Entity,
        center: IVec3,
        controller: &CameraLodController,
        streaming: &GridStreaming,
        desired_chunks: &HashSet<ChunkKey>,
        desired_tiles: &HashSet<TileKey>,
    ) {
        let outer = lod_outer_radius(controller);
        let min = center - IVec3::splat(outer);
        let max = center + IVec3::splat(outer);
        let mut missing = Vec::new();
        streaming.presence().for_each_in_region(min, max, |chunk| {
            let key = ChunkKey { grid, chunk };
            let xz_dist = (chunk.x - center.x).abs().max((chunk.z - center.z).abs()).max((chunk.y - center.y).abs());
            if xz_dist <= controller.near_radius_chunks {
                if !desired_chunks.contains(&key) {
                    missing.push(chunk);
                }
                return;
            }
            if !desired_tiles.iter().any(|tile| tile.grid == grid && tile_covers_chunk(*tile, chunk)) {
                missing.push(chunk);
            }
        });
        assert!(missing.is_empty(), "present chunks were not covered at center {center:?}: {missing:?}");
    }

    fn load_church_chunks() -> HashSet<IVec3> {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../res/Church_Of_St_Sophia.vox");
        let bytes = std::fs::read(&path).unwrap_or_else(|err| panic!("failed to read {path:?}: {err}"));
        let data = dot_vox::load_bytes(&bytes).expect("failed to parse church vox");
        let mut chunks = HashSet::new();

        #[derive(Clone, Copy)]
        struct Frame {
            translation: Vec3,
            rotation: Quat,
            flip: IVec3,
        }

        let mut stack = vec![(0u32, Frame { translation: Vec3::ZERO, rotation: Quat::IDENTITY, flip: IVec3::new(1, 1, -1) })];
        while let Some((scene_id, pose)) = stack.pop() {
            let Some(node) = data.scenes.get(scene_id as usize) else { continue };
            match node {
                dot_vox::SceneNode::Transform { frames, child, .. } => {
                    let Some(frame) = frames.first() else { continue };
                    let pos = frame.position().unwrap_or(dot_vox::Position { x: 0, y: 0, z: 0 });
                    let (rot, flip_vec) = frame
                        .orientation()
                        .map(|q| {
                            let (qarr, varr) = q.to_quat_scale();
                            let q = Quat::from_array(qarr);
                            (Quat::from_xyzw(q.x, q.z, -q.y, q.w), Vec3::from_array(varr).as_ivec3())
                        })
                        .unwrap_or((Quat::IDENTITY, IVec3::ONE));
                    stack.push((*child, Frame {
                        translation: pose.translation + pose.rotation * Vec3::new(pos.x as f32, pos.z as f32, -pos.y as f32),
                        rotation: pose.rotation * rot,
                        flip: pose.flip * IVec3::new(flip_vec.x, flip_vec.z, flip_vec.y),
                    }));
                }
                dot_vox::SceneNode::Group { children, .. } => {
                    for child in children {
                        stack.push((*child, pose));
                    }
                }
                dot_vox::SceneNode::Shape { models, .. } => {
                    for shape_model in models {
                        let Some(model) = data.models.get(shape_model.model_id as usize) else { continue };
                        let size = Vec3::new(model.size.x as f32, model.size.z as f32, model.size.y as f32);
                        let half = (size / 2.0).floor();
                        let pose_transform = Transform { translation: pose.translation, rotation: pose.rotation, scale: Vec3::ONE };
                        let half_offset = Transform::from_translation(-half * pose.flip.as_vec3());
                        for voxel in &model.voxels {
                            let local = IVec3::new(voxel.x as i32, voxel.z as i32, voxel.y as i32) * pose.flip + pose.flip.min(IVec3::ZERO);
                            let world_pos = (pose_transform * half_offset).transform_point(local.as_vec3()).as_ivec3();
                            chunks.insert(voxel_streaming::chunk_of(world_pos));
                        }
                    }
                }
            }
        }

        chunks
    }
}
