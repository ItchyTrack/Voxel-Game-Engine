use std::collections::{HashMap, HashSet};
use std::path::PathBuf;

use bevy::prelude::*;
use voxel_streaming::{chunk_of, GridStreaming};

mod types {
	use bevy::prelude::*;
	use voxel_data::grid::GridId;

	#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
	pub(crate) struct TileKey {
		pub(crate) grid: GridId,
		pub(crate) lod: u8,
		pub(crate) min: IVec3,
	}
	impl TileKey {
		pub(crate) fn size(self) -> IVec3 {
			IVec3::splat(1i32 << self.lod)
		}
	}
}

#[allow(dead_code)]
#[path = "../src/replacement_graph.rs"]
mod replacement_graph;
#[allow(dead_code)]
#[path = "../src/unresolved_tile_index.rs"]
mod unresolved_tile_index;

mod camera_voxel_loader {
	use super::*;
	use crate::coverage::SourceState;
	use crate::replacement_graph::ReplacementGraph;
	use crate::types::TileKey;
	use crate::unresolved_tile_index::UnresolvedTileIndex;

	#[derive(Debug, Clone, PartialEq, Eq)]
	pub struct CameraVoxelLoaderSettings {
		pub max_lod: u8,
		pub near_radius_chunks: i32,
		pub rings_per_lod: i32,
		pub requests_per_frame: usize,
	}
	impl Default for CameraVoxelLoaderSettings {
		fn default() -> Self {
			Self { max_lod: 2, near_radius_chunks: 3, rings_per_lod: 2, requests_per_frame: 16 }
		}
	}

	#[derive(Default)]
	pub(crate) struct CameraVoxelLoader {
		pub(crate) settings: CameraVoxelLoaderSettings,
		pub(crate) desired_tiles: HashSet<TileKey>,
		pub(crate) coverage_sources: HashMap<TileKey, SourceState>,
		pub(crate) unresolved_tiles: UnresolvedTileIndex,
		pub(crate) replacement_graph: ReplacementGraph,
	}
}

#[allow(dead_code)]
#[path = "../src/coverage.rs"]
mod coverage;
#[allow(dead_code)]
#[path = "../src/lod_policy.rs"]
mod lod_policy;

use camera_voxel_loader::CameraVoxelLoader;
use coverage::{request_source, resolve_empty, resolve_visible, undesire_source, SourceResolution, SourceState};
use lod_policy::{add_lod_tiles, add_near_tiles};
use types::TileKey;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SimTileStatus {
	Loading(u8),
	Visible,
}

#[test]
fn desired_empty_lod_tile_stays_resolved_without_persistent_tile_record() {
	let grid = Entity::PLACEHOLDER;
	let tile = TileKey { grid, lod: 2, min: IVec3::new(16, -4, 4) };
	let mut loader = CameraVoxelLoader::default();

	let desired_tiles = HashSet::from([tile]);
	sync_desired_coverage(&mut loader, &desired_tiles);
	resolve_empty(&mut loader, tile);
	sync_desired_coverage(&mut loader, &desired_tiles);

	let state = loader.coverage_sources.get(&tile).copied();
	assert!(matches!(state, Some(SourceState::Desired(SourceResolution::Empty))));

	sync_desired_coverage(&mut loader, &HashSet::new());
	sync_desired_coverage(&mut loader, &desired_tiles);
	let state = loader.coverage_sources.get(&tile).copied();
	assert!(matches!(state, Some(SourceState::Desired(SourceResolution::Requested))));
}

#[test]
fn re_desired_ready_lod_tile_resolves_visible_without_new_gpu_event() {
	let grid = Entity::PLACEHOLDER;
	let old_tile = TileKey { grid, lod: 2, min: IVec3::ZERO };
	let ready_replacement = TileKey { grid, lod: 1, min: IVec3::ZERO };
	let mut loader = CameraVoxelLoader::default();
	loader.desired_tiles.insert(ready_replacement);

	request_source(&mut loader, old_tile);
	resolve_visible(&mut loader, old_tile, Entity::from_bits(1));
	request_source(&mut loader, ready_replacement);
	undesire_source(&mut loader, old_tile);
	assert!(matches!(
		loader.coverage_sources.get(&ready_replacement).copied(),
		Some(SourceState::Desired(SourceResolution::Requested))
	));

	let existing_ready_tile_entity = Entity::from_bits(2);
	request_source(&mut loader, ready_replacement);
	let ready = resolve_visible(&mut loader, ready_replacement, existing_ready_tile_entity);

	assert!(
		matches!(loader.coverage_sources.get(&ready_replacement).copied(), Some(SourceState::Desired(SourceResolution::Visible(entity))) if entity == existing_ready_tile_entity)
	);
	assert_eq!(ready, vec![old_tile]);
}

#[test]
fn church_flight_never_waits_on_empty_lod_tile_that_will_not_request_again() {
	let grid = Entity::PLACEHOLDER;
	let church_chunks = load_church_chunks();
	assert!(!church_chunks.is_empty(), "church fixture has no chunks");

	let min = church_chunks.iter().copied().reduce(IVec3::min).unwrap() - IVec3::splat(4);
	let max = church_chunks.iter().copied().reduce(IVec3::max).unwrap() + IVec3::splat(4);
	let size = max - min + IVec3::ONE;

	let mut streaming = GridStreaming::default();
	streaming.presence_mut().mark_present_area(min, size);

	let mut loader = CameraVoxelLoader::default();
	let mut tile_records = HashMap::<TileKey, SimTileStatus>::new();
	let mut tile_data_cache = HashMap::<TileKey, bool>::new();
	let centers = smooth_orbit_path(min, max);

	for (frame, center) in centers.into_iter().enumerate() {
		let mut desired_tiles = HashSet::new();
		add_near_tiles(&mut desired_tiles, grid, center, &loader, &streaming);
		add_lod_tiles(&mut desired_tiles, grid, center, &loader, &streaming);

		sync_desired_coverage(&mut loader, &desired_tiles);
		loader.desired_tiles = desired_tiles.clone();

		for source in loader.desired_tiles.iter().copied().filter(|key| key.lod == 0).collect::<Vec<_>>() {
			if church_chunks.contains(&source.min) {
				resolve_visible(&mut loader, source, Entity::from_bits(10));
			} else {
				resolve_empty(&mut loader, source);
			}
		}

		for tile in desired_tiles.iter().copied().filter(|key| key.lod > 0) {
			match tile_records.get(&tile).copied() {
				Some(SimTileStatus::Visible) => {
					let _ = resolve_visible(&mut loader, tile, Entity::from_bits(20));
				}
				Some(SimTileStatus::Loading(_)) => {}
				None if tile_coverage_is_desired_empty(&loader, tile) => {}
				None => {
					tile_records.insert(tile, SimTileStatus::Loading(2));
				}
			}
		}

		let loading: Vec<_> =
			tile_records.iter().filter_map(|(&tile, &status)| matches!(status, SimTileStatus::Loading(_)).then_some(tile)).collect();
		for tile in loading {
			let SimTileStatus::Loading(remaining) = tile_records[&tile] else { unreachable!() };
			if remaining > 0 {
				tile_records.insert(tile, SimTileStatus::Loading(remaining - 1));
				continue;
			}
			if tile_has_real_church_data(tile, &church_chunks, &mut tile_data_cache) {
				tile_records.insert(tile, SimTileStatus::Visible);
				resolve_visible(&mut loader, tile, Entity::from_bits(20));
			} else {
				tile_records.remove(&tile);
				resolve_empty(&mut loader, tile);
			}
		}

		for tile in desired_tiles.iter().copied().filter(|key| key.lod > 0) {
			if !tile_records.contains_key(&tile) {
				let state = loader.coverage_sources.get(&tile).copied();
				assert!(
					matches!(state, Some(SourceState::Desired(SourceResolution::Empty))),
					"frame {frame}: desired LOD tile {tile:?} has no TileRecord, but coverage is {state:?}"
				);
			}
		}
	}
}

fn sync_desired_coverage(loader: &mut CameraVoxelLoader, desired_tiles: &HashSet<TileKey>) {
	loader.desired_tiles = desired_tiles.clone();
	for source in desired_tiles {
		request_source(loader, *source);
	}
	let old_desired: Vec<_> =
		loader.coverage_sources.iter().filter_map(|(&source, record)| matches!(record, SourceState::Desired(_)).then_some(source)).collect();
	for source in old_desired {
		if !desired_tiles.contains(&source) {
			undesire_source(loader, source);
		}
	}
}

fn tile_coverage_is_desired_empty(loader: &CameraVoxelLoader, tile: TileKey) -> bool {
	matches!(loader.coverage_sources.get(&tile).copied(), Some(SourceState::Desired(SourceResolution::Empty)))
}

fn tile_has_real_church_data(tile: TileKey, chunks: &HashSet<IVec3>, cache: &mut HashMap<TileKey, bool>) -> bool {
	*cache.entry(tile).or_insert_with(|| {
		let max = tile.min + tile.size();
		chunks.iter().any(|chunk| chunk.cmpge(tile.min).all() && chunk.cmplt(max).all())
	})
}

fn smooth_orbit_path(min: IVec3, max: IVec3) -> Vec<IVec3> {
	let center = (min + max) / 2;
	let radius = ((max.x - min.x).abs().max((max.z - min.z).abs()) / 2 + 8).max(12) as f32;
	let mut out = Vec::new();
	for i in 0..32 {
		let t = i as f32 * 0.065;
		let y = center.y + ((t * 0.7).sin() * 5.0).round() as i32;
		out.push(IVec3::new(center.x + (t.cos() * radius).round() as i32, y, center.z + (t.sin() * radius).round() as i32));
	}
	out
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
				stack.push((
					*child,
					Frame {
						translation: pose.translation + pose.rotation * Vec3::new(pos.x as f32, pos.z as f32, -pos.y as f32),
						rotation: pose.rotation * rot,
						flip: pose.flip * IVec3::new(flip_vec.x, flip_vec.z, flip_vec.y),
					},
				));
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
						chunks.insert(chunk_of(world_pos));
					}
				}
			}
		}
	}
	chunks
}
