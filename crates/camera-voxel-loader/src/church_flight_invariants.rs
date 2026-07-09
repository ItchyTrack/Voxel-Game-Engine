//! High-level integration test: flying a camera around the Church of St. Sophia must never drop a
//! chunk in a way that opens a hole. Once a chunk has been shown, it must keep displayable coverage
//! (a `Visible` or `RetiringVisible` source overlapping it) for as long as it stays desired. This
//! drives the real desired-set policy and the real coverage/replacement state machine, standing in
//! only for the streaming layer (which resolves requests with realistic load latency).

use std::collections::{HashMap, HashSet};
use std::path::PathBuf;

use bevy::prelude::*;
use voxel_streaming::{chunk_of, GridStreaming};

use crate::camera_voxel_loader::{CameraVoxelLoader, CameraVoxelLoaderSettings};
use crate::loading::{remove_source, request_source, resolve_empty, resolve_visible, undesire_source};
use crate::lod_policy::update_desired_sources_delta;
use crate::types::{SourceResolution, SourceState, TileKey};

// Any nonzero bits work; the invariant only cares whether a source is displayable, not which entity.
const VISIBLE_ENTITY: Entity = Entity::from_bits(0x1_0000_000A);
// Frames a desired LOD tile spends loading before the "streaming layer" resolves it. A nonzero
// latency is what creates the replacement window the no-gap logic has to bridge.
const LOAD_LATENCY: u8 = 2;

// Small radii force the church to span several LOD bands at orbit distance, so flying it exercises
// coarse-tile-to-finer-tile replacement rather than keeping everything at LOD 0.
fn flight_settings() -> CameraVoxelLoaderSettings {
	CameraVoxelLoaderSettings { max_lod: 3, near_radius_chunks: 1, rings_per_lod: 1, requests_per_frame: 4096 }
}

#[test]
fn church_flight_never_drops_a_visible_chunk_and_opens_a_hole() {
	let grid = Entity::PLACEHOLDER;
	let church_chunks = load_church_chunks();
	assert!(!church_chunks.is_empty(), "church fixture has no chunks");

	let min = church_chunks.iter().copied().reduce(IVec3::min).unwrap() - IVec3::splat(4);
	let max = church_chunks.iter().copied().reduce(IVec3::max).unwrap() + IVec3::splat(4);
	let mut streaming = GridStreaming::default();
	streaming.mark_present_area(min, max - min + IVec3::ONE);

	let settings = flight_settings();
	let mut loader = CameraVoxelLoader::with_settings(settings.clone());

	// Frames remaining until each in-flight LOD tile resolves, standing in for the streaming layer.
	let mut loading: HashMap<TileKey, u8> = HashMap::new();
	// Memoized "does this tile's region contain real church voxels" so a resolved tile becomes
	// visible only where there is something to show.
	let mut covers_church_cache: HashMap<TileKey, bool> = HashMap::new();
	// Church chunks that have been displayable at least once; only these can open a hole.
	let mut shown_chunks: HashSet<IVec3> = HashSet::new();

	for (frame, center) in smooth_orbit_path(min, max).into_iter().enumerate() {
		let delta = update_desired_sources_delta(&mut loader, grid, center, &settings, &streaming);

		// Newly desired sources are wanted first (matching the scheduler), so a coarse tile that is
		// undesired below can still see its finer replacements as unresolved and hold coverage.
		for &key in &delta.added {
			request_source(&mut loader, key);
			if key.is_chunk() {
				// Chunks resolve immediately: their voxel data is already present in the fixture.
				resolve_source_and_retire(&mut loader, &mut loading, key, &church_chunks, &mut covers_church_cache);
			} else if matches!(loader.coverage_sources.get(&key), Some(SourceState::Desired(SourceResolution::Requested))) {
				loading.insert(key, LOAD_LATENCY);
			}
		}

		for &key in &delta.removed {
			let ready = undesire_source(&mut loader, key);
			retire(&mut loader, &mut loading, ready);
		}

		for tile in loading.keys().copied().collect::<Vec<_>>() {
			let remaining = loading.get_mut(&tile).unwrap();
			if *remaining > 0 {
				*remaining -= 1;
				continue;
			}
			loading.remove(&tile);
			resolve_source_and_retire(&mut loader, &mut loading, tile, &church_chunks, &mut covers_church_cache);
		}

		for &chunk in &church_chunks {
			if chunk_has_displayable_coverage(&loader, grid, chunk) {
				shown_chunks.insert(chunk);
			}
		}

		// No-hole invariant: a chunk we have shown must stay displayable while it remains desired.
		for &chunk in &shown_chunks {
			if chunk_is_desired(&loader, grid, chunk) {
				assert!(
					chunk_has_displayable_coverage(&loader, grid, chunk),
					"frame {frame}: church chunk {chunk:?} lost visible coverage while still desired — a hole opened"
				);
			}
		}

		// No desired LOD tile may sit stuck as Requested without anything in flight to resolve it,
		// or it would wait forever and never contribute coverage.
		for &tile in loader.desired_tiles.iter().filter(|key| !key.is_chunk()) {
			if matches!(loader.coverage_sources.get(&tile), Some(SourceState::Desired(SourceResolution::Requested))) {
				assert!(loading.contains_key(&tile), "frame {frame}: desired LOD tile {tile:?} is stuck Requested with no pending load");
			}
		}
	}

	assert!(!shown_chunks.is_empty(), "the flight never displayed any church chunk — the scenario proves nothing");
}

fn resolve_source_and_retire(
	loader: &mut CameraVoxelLoader,
	loading: &mut HashMap<TileKey, u8>,
	tile: TileKey,
	church_chunks: &HashSet<IVec3>,
	cache: &mut HashMap<TileKey, bool>,
) {
	let has_data = if tile.is_chunk() { church_chunks.contains(&tile.min) } else { tile_covers_church(tile, church_chunks, cache) };
	let ready = if has_data { resolve_visible(loader, tile, VISIBLE_ENTITY) } else { resolve_empty(loader, tile) };
	retire(loader, loading, ready);
}

fn retire(loader: &mut CameraVoxelLoader, loading: &mut HashMap<TileKey, u8>, ready: Vec<TileKey>) {
	for source in ready {
		remove_source(loader, source);
		loading.remove(&source);
	}
}

fn region_contains(key: TileKey, chunk: IVec3) -> bool {
	let max = key.min + key.size();
	chunk.cmpge(key.min).all() && chunk.cmplt(max).all()
}

fn chunk_has_displayable_coverage(loader: &CameraVoxelLoader, grid: Entity, chunk: IVec3) -> bool {
	loader.coverage_sources.iter().any(|(&key, &state)| {
		key.grid == grid
			&& region_contains(key, chunk)
			&& matches!(state, SourceState::Desired(SourceResolution::Visible(_)) | SourceState::RetiringVisible(_))
	})
}

fn chunk_is_desired(loader: &CameraVoxelLoader, grid: Entity, chunk: IVec3) -> bool {
	loader.desired_tiles.iter().any(|&key| key.grid == grid && region_contains(key, chunk))
}

fn tile_covers_church(tile: TileKey, church_chunks: &HashSet<IVec3>, cache: &mut HashMap<TileKey, bool>) -> bool {
	*cache.entry(tile).or_insert_with(|| church_chunks.iter().any(|&chunk| region_contains(tile, chunk)))
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
