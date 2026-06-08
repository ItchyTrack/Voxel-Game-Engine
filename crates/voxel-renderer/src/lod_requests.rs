use std::collections::{HashMap, HashSet};

use bevy::camera::Camera;
use bevy::prelude::*;

use gpu_voxel_data::residency::ResidencyBuffers;
use gpu_voxel_data::LodVoxels;
use voxel_data::grid::{GridId, SUB_GRID_SIZE};
use voxel_data::subgrid::SubGrid;
use voxel_streaming::{ChunkConsumer, GridStreaming, LodRequestChannel, CHUNK_SIZE};

/// Coarsest LOD level requested. Level `L` covers `2^L` chunks per axis.
const MAX_LOD: u32 = 4;
/// Chunks within this camera distance render at full detail; LOD takes over beyond it.
/// Shared with the full-resolution chunk streamer so the two regions meet exactly.
pub const FULL_DETAIL_CHUNKS: f32 = 2.0;

/// A LOD tile: downsample level, and the chunk-space box `[min, min+size)` it covers.
type TileKey = (u32, IVec3, IVec3);

/// Outer edge of LOD level `lod`'s ring, in chunks.
fn ring_outer_chunks(lod: u32) -> f32 {
	FULL_DETAIL_CHUNKS * (1u32 << lod) as f32
}

/// Beyond this camera distance (chunks) nothing is rendered — neither full detail nor LOD.
pub fn max_lod_radius_chunks() -> f32 {
	ring_outer_chunks(MAX_LOD)
}

/// LOD level for a chunk at `distance` chunks from the camera, or `None` if it is
/// full detail (closer than the inner ring) or past the coarsest ring.
fn lod_for_distance(distance: f32) -> Option<u32> {
	if distance < FULL_DETAIL_CHUNKS { return None; }
	(1..=MAX_LOD).find(|&lod| distance < ring_outer_chunks(lod))
}

voxel_streaming::chunk_consumer!(pub RenderLodConsumer);

/// Keys of the LOD tiles wanted last frame, diffed against the new set.
#[derive(Component, Default)]
pub struct RenderWantedLod(HashSet<TileKey>);

#[derive(Resource, Default)]
pub struct LodTileEntities(HashMap<(GridId, TileKey), Entity>);

pub fn spawn_lod_consumer(mut commands: Commands) {
	commands.spawn(RenderLodConsumer::default());
}

/// Classify each present chunk near the camera by LOD, then merge same-LOD chunks into
/// tiles. Each chunk lands in exactly one tile, so the tiles cover the region with no
/// holes and no overlap; merging keeps the tile (and GPU buffer) count down.
fn collect_wanted_tiles(
	camera_chunk: Vec3,
	streaming: &GridStreaming,
	want: &mut HashMap<TileKey, f32>,
) {
	let max_radius = ring_outer_chunks(MAX_LOD);
	let region_min = (camera_chunk - max_radius).floor().as_ivec3();
	let region_max = (camera_chunk + max_radius).ceil().as_ivec3();

	// Same-LOD chunks, grouped by the `2^lod`-aligned block they fall in. Confining a
	// tile to one block bounds its size to `2^lod` per axis, which `fetch_lod` requires
	// (the downsample must land in a single chunk-sized tree).
	let mut groups: HashMap<(u32, IVec3), Vec<IVec3>> = HashMap::new();
	streaming.presence().for_each_in_region(region_min, region_max, |chunk| {
		let lo = chunk.as_vec3();
		let hi = (chunk + IVec3::ONE).as_vec3();
		let distance = camera_chunk.distance(camera_chunk.clamp(lo, hi));
		let Some(lod) = lod_for_distance(distance) else { return };
		let block = chunk.div_euclid(IVec3::splat(1i32 << lod)) * (1i32 << lod);
		groups.entry((lod, block)).or_default().push(chunk);
	});

	for ((lod, _block), chunks) in groups {
		for (min, size) in greedy_boxes(&chunks) {
			let nearest = camera_chunk.clamp(min.as_vec3(), (min + size).as_vec3());
			let priority = -camera_chunk.distance(nearest) * CHUNK_SIZE as f32;
			want.insert((lod, min, size), priority);
		}
	}
}

/// Partition a set of chunks (all within one aligned block) into disjoint boxes by
/// greedy meshing: grow each box along x, then y, then z while the next layer is fully
/// present and unclaimed.
fn greedy_boxes(chunks: &[IVec3]) -> Vec<(IVec3, IVec3)> {
	let present: HashSet<IVec3> = chunks.iter().copied().collect();
	let mut visited: HashSet<IVec3> = HashSet::new();
	let mut sorted = chunks.to_vec();
	sorted.sort_by_key(|chunk| (chunk.z, chunk.y, chunk.x));

	let free = |present: &HashSet<IVec3>, visited: &HashSet<IVec3>, chunk: IVec3| {
		present.contains(&chunk) && !visited.contains(&chunk)
	};

	let mut boxes = Vec::new();
	for &start in &sorted {
		if visited.contains(&start) { continue; }

		let mut sx = 1;
		while free(&present, &visited, start + IVec3::new(sx, 0, 0)) { sx += 1; }

		let mut sy = 1;
		while (0..sx).all(|x| free(&present, &visited, start + IVec3::new(x, sy, 0))) { sy += 1; }

		let mut sz = 1;
		while (0..sy).all(|y| (0..sx).all(|x| free(&present, &visited, start + IVec3::new(x, y, sz)))) {
			sz += 1;
		}

		for z in 0..sz {
			for y in 0..sy {
				for x in 0..sx {
					visited.insert(start + IVec3::new(x, y, z));
				}
			}
		}
		boxes.push((start, IVec3::new(sx, sy, sz)));
	}
	boxes
}

/// Chunks covered by tiles that are wanted and actually rendering (resident) — the
/// replacements a retiring tile waits on. A spawned-but-not-yet-rendering tile does not
/// count, or the old tile would unload before the new one appears (a hole).
fn covered_chunks(
	tiles: &HashMap<(GridId, TileKey), Entity>,
	grid: GridId,
	want: &HashMap<TileKey, f32>,
	residency: &ResidencyBuffers,
) -> HashSet<IVec3> {
	let mut covered = HashSet::new();
	for (&(tile_grid, key), &entity) in tiles.iter() {
		if tile_grid != grid || !want.contains_key(&key) || !residency.offsets().contains_key(&entity) {
			continue;
		}
		let (_, min, size) = key;
		for z in 0..size.z {
			for y in 0..size.y {
				for x in 0..size.x {
					covered.insert(min + IVec3::new(x, y, z));
				}
			}
		}
	}
	covered
}

/// Chunks covered by full-resolution sub-grids that are actually rendering (resident).
fn full_res_chunks(
	full_res: &Query<(Entity, &SubGrid)>,
	grid: GridId,
	residency: &ResidencyBuffers,
) -> HashSet<IVec3> {
	let mut covered = HashSet::new();
	for (entity, sub_grid) in full_res.iter() {
		if sub_grid.grid() != grid || !residency.offsets().contains_key(&entity) { continue; }
		let pos = sub_grid.sub_grid_pos();
		let lo = pos.div_euclid(IVec3::splat(CHUNK_SIZE));
		let hi = (pos + IVec3::splat(SUB_GRID_SIZE - 1)).div_euclid(IVec3::splat(CHUNK_SIZE));
		for z in lo.z..=hi.z {
			for y in lo.y..=hi.y {
				for x in lo.x..=hi.x {
					covered.insert(IVec3::new(x, y, z));
				}
			}
		}
	}
	covered
}

/// Whether every chunk of the retiring tile is now shown by a GPU-ready LOD or full-res
/// replacement, out of view range, or absent.
fn tile_fully_covered(
	min: IVec3,
	size: IVec3,
	covered: &HashSet<IVec3>,
	full_res_covered: &HashSet<IVec3>,
	camera_chunk: Vec3,
	streaming: &GridStreaming,
) -> bool {
	for z in 0..size.z {
		for y in 0..size.y {
			for x in 0..size.x {
				let chunk = min + IVec3::new(x, y, z);
				if covered.contains(&chunk) || full_res_covered.contains(&chunk) { continue; }
				if !streaming.presence().is_present(chunk) { continue; }
				let lo = chunk.as_vec3();
				let hi = (chunk + IVec3::ONE).as_vec3();
				let distance = camera_chunk.distance(camera_chunk.clamp(lo, hi));
				if distance >= max_lod_radius_chunks() { continue; }
				return false;
			}
		}
	}
	true
}

pub fn request_render_lod(
	mut commands: Commands,
	cameras: Query<(&Camera, &GlobalTransform)>,
	mut grids: Query<(Entity, &GlobalTransform, &GridStreaming, Option<&mut RenderWantedLod>)>,
	consumer: Option<Single<Entity, With<RenderLodConsumer>>>,
	channel: Res<LodRequestChannel>,
	mut tiles: ResMut<LodTileEntities>,
	full_res: Query<(Entity, &SubGrid)>,
	residency: Res<ResidencyBuffers>,
	freeze: Res<crate::scene::FreezeRenderRequests>,
) {
	if freeze.0 { return; }
	let Some(consumer) = consumer else { return };
	let consumer_entity = *consumer;
	let Some(camera_world) = cameras
		.iter()
		.find(|(camera, _)| camera.is_active)
		.map(|(_, transform)| transform.translation())
	else { return };

	for (grid_entity, grid_global, streaming, wanted) in grids.iter_mut() {
		let camera_local = grid_global.affine().inverse().transform_point3(camera_world);
		let camera_chunk = camera_local / CHUNK_SIZE as f32;

		let mut want: HashMap<TileKey, f32> = HashMap::new();
		collect_wanted_tiles(camera_chunk, streaming, &mut want);

		let empty = HashSet::new();
		let previous = wanted.as_deref().map(|wanted| &wanted.0).unwrap_or(&empty);
		for (&key, &priority) in want.iter() {
			let (lod, min, size) = key;
			if previous.contains(&key) || tiles.0.contains_key(&(grid_entity, key)) { continue; }
			streaming.fetch_lod(grid_entity, consumer_entity, &channel, min, size, lod as f32, priority);
		}

		// Retire tiles no longer wanted, but only once their region is covered by loaded
		// replacements — despawning before the new LOD loads would leave a hole.
		let retiring: Vec<TileKey> = tiles.0
			.keys()
			.filter_map(|(grid, key)| (*grid == grid_entity && !want.contains_key(key)).then_some(*key))
			.collect();
		if !retiring.is_empty() {
			let covered = covered_chunks(&tiles.0, grid_entity, &want, &residency);
			let full_res_covered = full_res_chunks(&full_res, grid_entity, &residency);
			for key in retiring {
				if tile_fully_covered(key.1, key.2, &covered, &full_res_covered, camera_chunk, streaming) {
					if let Some(entity) = tiles.0.remove(&(grid_entity, key)) {
						commands.entity(entity).despawn();
					}
				}
			}
		}

		let new_keys: HashSet<TileKey> = want.into_keys().collect();
		match wanted {
			Some(mut wanted) => wanted.0 = new_keys,
			None => { commands.entity(grid_entity).insert(RenderWantedLod(new_keys)); }
		}
	}
}

pub fn receive_render_lod(
	mut commands: Commands,
	consumer: Option<Single<&mut RenderLodConsumer>>,
	wanted: Query<&RenderWantedLod>,
	mut tiles: ResMut<LodTileEntities>,
) {
	let Some(mut consumer) = consumer else { return };
	for result in consumer.drain_lod() {
		let key: TileKey = (result.lod.max(0.0).floor() as u32, result.min, result.size);
		if let Some(previous) = tiles.0.remove(&(result.grid, key)) {
			commands.entity(previous).despawn();
		}
		// Drop deliveries for tiles the camera has already moved past.
		if !wanted.get(result.grid).is_ok_and(|wanted| wanted.0.contains(&key)) { continue; }
		let Some(voxels) = result.voxels else { continue };
		let entity = commands
			.spawn(LodVoxels {
				voxels,
				grid: result.grid,
				min: result.min,
				size: result.size,
				lod: result.lod,
				priority: result.priority,
			})
			.id();
		tiles.0.insert((result.grid, key), entity);
	}
}
