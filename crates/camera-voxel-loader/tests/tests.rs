
#[cfg(test)]
mod tests {
	use std::path::PathBuf;
	use bevy::math::{IVec3};

	use camera_voxel_loader::*;

	// #[test]
	// fn aligns_negative_chunks_to_lod_tile() {
	//     assert_eq!(align_chunk_to_tile(IVec3::new(-1, -2, -3), 4), IVec3::new(-4, -4, -4));
	//     assert_eq!(align_chunk_to_tile(IVec3::new(4, 5, 7), 4), IVec3::new(4, 4, 4));
	// }

	// #[test]
	// fn tile_ring_intersection_uses_tile_extent_not_center() {
	//     assert!(tile_intersects_ring(IVec3::ZERO, IVec3::new(-4, 0, 0), 2, 4, 8));
	//     assert!(!tile_intersects_ring(IVec3::ZERO, IVec3::new(-2, 0, 0), 2, 4, 8));
	// }

	// #[test]
	// fn camera_chunk_center_switches_at_chunk_midpoint() {
	//     assert_eq!(nearest_chunk_center(Vec3::new(1.4 * CHUNK_SIZE as f32, 0.0, 0.0)), IVec3::new(1, 0, 0));
	//     assert_eq!(nearest_chunk_center(Vec3::new(1.6 * CHUNK_SIZE as f32, 0.0, 0.0)), IVec3::new(2, 0, 0));
	// }

	// #[test]
	// fn near_chunk_leaving_desired_set_is_not_safe_to_release_without_replacement() {
	//     let grid = Entity::PLACEHOLDER;
	//     let chunk = ChunkKey { grid, chunk: IVec3::ZERO };

	//     let mut state = CameraLodState::default();
	//     state.desired_chunks = HashSet::from([chunk]);

	//     let to_fetch = update_desired_chunks(&mut state, HashSet::new());

	//     // Desired behavior: a near chunk remains tracked as retiring until a ready
	//     // LOD tile covers the same space. It should not be released immediately.
	//     assert!(to_fetch.is_empty());
	//     assert!(state.retiring_chunks.contains(&chunk), "dropped chunk was not kept for no-gap retirement");
	// }

	// #[test]
	// fn empty_lod_tile_is_not_renderable_replacement_coverage() {
	//     let grid = Entity::PLACEHOLDER;
	//     let old = TileKey { grid, lod: 1, min: IVec3::ZERO };
	//     let mut state = CameraLodState::default();
	//     state.tiles.insert(old, TileRecord { status: TileStatus::Retiring, entity: Some(Entity::PLACEHOLDER) });
	//     for x in 0..2 {
	//         for y in 0..2 {
	//             for z in 0..2 {
	//                 state.tiles.insert(TileKey { grid, lod: 0, min: IVec3::new(x, y, z) }, TileRecord { status: TileStatus::Empty, entity: None });
	//             }
	//         }
	//     }

	//     // Empty tiles have no LodVoxels entity and render nothing, so they must not
	//     // be allowed to retire visible old coverage. If they do, the old tile is
	//     // removed and the area becomes a persistent hole.
	//     assert!(!area_is_covered_by_ready_desired(&state, old), "empty tile with no render entity was treated as replacement coverage");
	// }

	// #[test]
	// fn in_flight_tile_that_temporarily_leaves_view_must_keep_record_for_late_result() {
	//     let grid = Entity::PLACEHOLDER;
	//     let key = TileKey { grid, lod: 2, min: IVec3::ZERO };
	//     let mut state = CameraLodState::default();
	//     state.tiles.insert(key, TileRecord { status: TileStatus::Loading, entity: None });

	//     handle_non_desired_tile(&mut state, key);

	//     // A load result can arrive after movement changes the desired set. If the
	//     // record was removed, receive_camera_lod_results will ignore valid voxel
	//     // data for this tile, leaving the area with no loaded replacement.
	//     assert!(state.tiles.contains_key(&key), "in-flight tile record was removed before its result could be applied");
	// }

	// #[test]
	// fn lod_tiles_cover_present_chunks_at_near_ring_boundary() {
	//     let grid = Entity::PLACEHOLDER;
	//     let center = IVec3::ZERO;
	//     let controller = CameraLodController { max_lod: 1, ..Default::default() };
	//     let mut streaming = GridStreaming::default();
	//     streaming.presence_mut().mark_present_area(IVec3::splat(-8), IVec3::splat(17));

	//     let mut desired_chunks = HashSet::new();
	//     let mut desired_tiles = HashSet::new();
	//     add_near_chunks(&mut desired_chunks, grid, center, &controller);
	//     add_lod_tiles(&mut desired_tiles, grid, center, &controller, &streaming);

	//     let boundary_chunk = ChunkKey { grid, chunk: IVec3::new(-4, 0, 0) };
	//     let covered_by_near = desired_chunks.contains(&boundary_chunk);
	//     let covered_by_lod = desired_tiles.iter().any(|tile| tile_covers_chunk(*tile, boundary_chunk.chunk));

	//     assert!(
	//         covered_by_near || covered_by_lod,
	//         "present chunk {:?} is outside near chunks but not covered by any requested LOD tile; tiles={desired_tiles:?}",
	//         boundary_chunk.chunk,
	//     );
	// }

	// #[test]
	// fn moving_camera_lod_requests_are_set_deltas_and_keep_chunk_coverage() {
	//     let grid = Entity::PLACEHOLDER;
	//     let controller = CameraLodController { max_lod: 2, ..Default::default() };
	//     let mut streaming = GridStreaming::default();
	//     streaming.presence_mut().mark_present_area(IVec3::splat(-48), IVec3::splat(97));

	//     let centers = [
	//         IVec3::ZERO,
	//         IVec3::new(1, 0, 0),
	//         IVec3::new(2, 0, 0),
	//         IVec3::new(4, 0, 0),
	//         IVec3::new(7, 0, -3),
	//         IVec3::new(-5, 0, 6),
	//     ];

	//     let mut previous_tiles = HashSet::new();
	//     for center in centers {
	//         let mut desired_chunks = HashSet::new();
	//         let mut desired_tiles = HashSet::new();
	//         add_near_chunks(&mut desired_chunks, grid, center, &controller);
	//         add_lod_tiles(&mut desired_tiles, grid, center, &controller, &streaming);

	//         let expected_new: HashSet<_> = desired_tiles.difference(&previous_tiles).copied().collect();
	//         let expected_dropped: HashSet<_> = previous_tiles.difference(&desired_tiles).copied().collect();
	//         let unchanged: HashSet<_> = desired_tiles.intersection(&previous_tiles).copied().collect();

	//         assert!(expected_new.is_disjoint(&unchanged));
	//         assert!(expected_dropped.is_disjoint(&desired_tiles));
	//         assert_all_present_chunks_in_lod_range_are_covered(grid, center, &controller, &streaming, &desired_chunks, &desired_tiles);

	//         // Simulate the request scheduler's stable-state decision: only tiles in
	//         // desired - previous should become new requests for this camera movement.
	//         let scheduler_new: HashSet<_> = desired_tiles.iter().filter(|tile| !previous_tiles.contains(tile)).copied().collect();
	//         assert_eq!(scheduler_new, expected_new, "wrong LOD request delta at center {center:?}");

	//         previous_tiles = desired_tiles;
	//     }
	// }

	// #[test]
	// fn crossing_lod_tile_midpoint_changes_desired_delta_even_inside_same_chunk() {
	//     let grid = Entity::PLACEHOLDER;
	//     let controller = CameraLodController { max_lod: 1, near_radius_chunks: 1, rings_per_lod: 1, ..Default::default() };
	//     let mut streaming = GridStreaming::default();
	//     streaming.presence_mut().mark_present_area(IVec3::new(0, 0, 0), IVec3::new(6, 1, 1));

	//     // Both positions are inside chunk 1, but they cross the midpoint between
	//     // the previous LOD tile center and the next. Delta math should shift the
	//     // LOD0 window from A+B toward B+C without waiting for a whole chunk step.
	//     let before_center = nearest_chunk_center(Vec3::new(1.4 * CHUNK_SIZE as f32, 0.0, 0.0));
	//     let after_center = nearest_chunk_center(Vec3::new(1.6 * CHUNK_SIZE as f32, 0.0, 0.0));

	//     let mut before_chunks = HashSet::new();
	//     let mut before_tiles = HashSet::new();
	//     add_near_chunks(&mut before_chunks, grid, before_center, &controller);
	//     add_lod_tiles(&mut before_tiles, grid, before_center, &controller, &streaming);

	//     let mut after_chunks = HashSet::new();
	//     let mut after_tiles = HashSet::new();
	//     add_near_chunks(&mut after_chunks, grid, after_center, &controller);
	//     add_lod_tiles(&mut after_tiles, grid, after_center, &controller, &streaming);

	//     let added_chunks: HashSet<_> = after_chunks.difference(&before_chunks).copied().collect();
	//     let removed_chunks: HashSet<_> = before_chunks.difference(&after_chunks).copied().collect();
	//     let added_tiles: HashSet<_> = after_tiles.difference(&before_tiles).copied().collect();
	//     let removed_tiles: HashSet<_> = before_tiles.difference(&after_tiles).copied().collect();

	//     assert!(
	//         !added_chunks.is_empty() || !removed_chunks.is_empty() || !added_tiles.is_empty() || !removed_tiles.is_empty(),
	//         "crossing the tile midpoint produced no request delta: before_center={before_center:?} after_center={after_center:?}"
	//     );
	// }

	// #[test]
	// fn old_lod_tiles_delete_after_camera_moves_and_new_results_are_ready() {
	//     let grid = Entity::PLACEHOLDER;
	//     let controller = CameraLodController { max_lod: 2, ..Default::default() };
	//     let mut streaming = GridStreaming::default();
	//     streaming.presence_mut().mark_present_area(IVec3::splat(-48), IVec3::splat(97));

	//     let centers = [IVec3::ZERO, IVec3::new(16, 0, 0), IVec3::new(-16, 0, 16), IVec3::ZERO];
	//     let mut state = CameraLodState::default();

	//     for center in centers {
	//         let mut desired_chunks = HashSet::new();
	//         let mut desired_tiles = HashSet::new();
	//         add_near_chunks(&mut desired_chunks, grid, center, &controller);
	//         add_lod_tiles(&mut desired_tiles, grid, center, &controller, &streaming);
	//         update_desired_chunks(&mut state, desired_chunks);

	//         let old_tiles: Vec<TileKey> = state.tiles.keys().copied().collect();
	//         for key in old_tiles {
	//             if !desired_tiles.contains(&key) {
	//                 handle_non_desired_tile(&mut state, key);
	//             }
	//         }
	//         for tile in desired_tiles {
	//             state.tiles.entry(tile).or_insert(TileRecord { status: TileStatus::Loading, entity: None });
	//         }

	//         // Provide successful load + GPU-ready results for all current requests.
	//         for record in state.tiles.values_mut() {
	//             if matches!(record.status, TileStatus::Loading | TileStatus::LoadedWaitingGpu) {
	//                 record.status = TileStatus::Ready;
	//                 record.entity = Some(Entity::PLACEHOLDER);
	//             }
	//         }
	//         delete_replaced_tiles_for_test(&mut state);
	//     }

	//     let stuck: Vec<_> = state
	//         .tiles
	//         .iter()
	//         .filter_map(|(key, record)| (record.status == TileStatus::Retiring).then_some(*key))
	//         .collect();
	//     assert!(stuck.is_empty(), "old LOD tiles stayed retiring after replacement results were ready: {stuck:?}");
	// }

	// fn delete_replaced_tiles_for_test(state: &mut CameraLodState) {
	//     let retiring: Vec<_> = state
	//         .tiles
	//         .iter()
	//         .filter_map(|(key, record)| (record.status == TileStatus::Retiring).then_some(*key))
	//         .collect();
	//     for key in retiring {
	//         if area_is_covered_by_ready_desired(state, key) {
	//             state.tiles.remove(&key);
	//         }
	//     }
	// }

	// #[test]
	// fn retiring_chunks_resolve_after_ready_lods_at_multiple_camera_positions() {
	//     let grid = Entity::PLACEHOLDER;
	//     let controller = CameraLodController { max_lod: 2, ..Default::default() };
	//     let mut streaming = GridStreaming::default();
	//     streaming.presence_mut().mark_present_area(IVec3::splat(-48), IVec3::splat(97));

	//     let centers = [
	//         IVec3::ZERO,
	//         IVec3::new(4, 0, 0),
	//         IVec3::new(8, 0, 0),
	//         IVec3::new(8, 0, 6),
	//         IVec3::new(-4, 0, 6),
	//         IVec3::ZERO,
	//     ];

	//     let mut state = CameraLodState::default();
	//     for center in centers {
	//         let mut desired_chunks = HashSet::new();
	//         let mut desired_tiles = HashSet::new();
	//         add_near_chunks(&mut desired_chunks, grid, center, &controller);
	//         add_lod_tiles(&mut desired_tiles, grid, center, &controller, &streaming);

	//         update_desired_chunks(&mut state, desired_chunks);
	//         state.desired_tiles = desired_tiles.clone();
	//         state.tiles.retain(|key, _| desired_tiles.contains(key));
	//         for tile in desired_tiles {
	//             state.tiles.insert(tile, TileRecord { status: TileStatus::Ready, entity: Some(Entity::PLACEHOLDER) });
	//         }

	//         resolve_retiring_chunks_for_test(&mut state, controller.max_lod);
	//         assert!(
	//             state.retiring_chunks.is_empty(),
	//             "retiring chunks did not resolve at center {center:?}; count={} sample={:?}",
	//             state.retiring_chunks.len(),
	//             state.retiring_chunks.iter().take(20).collect::<Vec<_>>(),
	//         );
	//     }
	// }

	// fn resolve_retiring_chunks_for_test(state: &mut CameraLodState, max_lod: u8) {
	//     state.retiring_chunks = state
	//         .retiring_chunks
	//         .iter()
	//         .copied()
	//         .filter(|chunk| retiring_chunk_still_needs_high_res(state, *chunk, max_lod))
	//         .collect();
	// }

	// #[test]
	// fn retiring_chunks_survive_empty_lod_results_and_resolve_when_real_lods_arrive() {
	//     let grid = Entity::PLACEHOLDER;
	//     let controller = CameraLodController { max_lod: 2, ..Default::default() };
	//     let mut streaming = GridStreaming::default();
	//     streaming.presence_mut().mark_present_area(IVec3::splat(-48), IVec3::splat(97));

	//     let mut state = CameraLodState::default();
	//     let center_a = IVec3::ZERO;
	//     let center_b = IVec3::new(8, 0, 0);

	//     let mut desired_a = HashSet::new();
	//     add_near_chunks(&mut desired_a, grid, center_a, &controller);
	//     update_desired_chunks(&mut state, desired_a);

	//     let mut desired_b = HashSet::new();
	//     let mut tiles_b = HashSet::new();
	//     add_near_chunks(&mut desired_b, grid, center_b, &controller);
	//     add_lod_tiles(&mut tiles_b, grid, center_b, &controller, &streaming);
	//     update_desired_chunks(&mut state, desired_b);
	//     state.desired_tiles = tiles_b.clone();

	//     let retiring_before_results = state.retiring_chunks.clone();
	//     assert!(!retiring_before_results.is_empty(), "movement should create retiring chunks");

	//     // Simulate some LOD requests returning empty first. Empty records must not
	//     // release valid retiring chunks, because they render no replacement voxels.
	//     for tile in &tiles_b {
	//         state.tiles.insert(*tile, TileRecord { status: TileStatus::Empty, entity: None });
	//     }
	//     resolve_retiring_chunks_for_test(&mut state, controller.max_lod);
	//     assert_eq!(
	//         state.retiring_chunks, retiring_before_results,
	//         "empty LOD results retired chunks even though no renderable replacement exists",
	//     );

	//     // Later, real non-empty LOD results arrive for the same requested tiles.
	//     for tile in &tiles_b {
	//         state.tiles.insert(*tile, TileRecord { status: TileStatus::Ready, entity: Some(Entity::PLACEHOLDER) });
	//     }
	//     resolve_retiring_chunks_for_test(&mut state, controller.max_lod);
	//     assert!(
	//         state.retiring_chunks.is_empty(),
	//         "retiring chunks did not resolve after real LOD replacements became ready; sample={:?}",
	//         state.retiring_chunks.iter().take(20).collect::<Vec<_>>(),
	//     );
	// }

	// #[test]
	// fn retiring_chunk_outside_current_lod_coverage_does_not_stick_forever() {
	//     let grid = Entity::PLACEHOLDER;
	//     let max_lod = 2;
	//     let chunk = ChunkKey { grid, chunk: IVec3::new(64, 0, 0) };
	//     let mut state = CameraLodState::default();
	//     state.retiring_chunks.insert(chunk);

	//     // If the camera has moved far enough that no currently desired LOD tile
	//     // wants this old near chunk, keeping it in retiring_chunks only leaks a
	//     // high-res chunk and permanently increases per-frame visibility work.
	//     assert!(!retiring_chunk_still_needs_high_res(&state, chunk, max_lod));
	// }

	// #[test]
	// fn retiring_lod_tile_is_valid_replacement_for_high_res_chunk() {
	//     let grid = Entity::PLACEHOLDER;
	//     let max_lod = 2;
	//     let chunk = ChunkKey { grid, chunk: IVec3::new(1, 0, 0) };
	//     let tile = TileKey { grid, lod: 1, min: IVec3::ZERO };
	//     let mut state = CameraLodState::default();
	//     state.desired_tiles.insert(tile);
	//     state.tiles.insert(tile, TileRecord { status: TileStatus::Retiring, entity: Some(Entity::PLACEHOLDER) });

	//     // Retiring LOD tiles are still rendered by refresh_camera_lod_visibility,
	//     // so they can cover the visual gap while the high-res chunk is released.
	//     assert!(!retiring_chunk_still_needs_high_res(&state, chunk, max_lod));
	// }

	// #[test]
	// fn church_presence_chunks_all_have_camera_lod_requests() {
	//     let grid = Entity::PLACEHOLDER;
	//     let chunks = load_church_chunks();
	//     assert!(!chunks.is_empty(), "test fixture did not load any church chunks");

	//     let mut streaming = GridStreaming::default();
	//     for chunk in &chunks {
	//         streaming.presence_mut().mark_present(*chunk);
	//     }

	//     // Matches voxel-app scene setup: church grid is under a parent translated to y=-350,
	//     // while the camera spawns at world (0, 0, 60).
	//     let camera_chunk_in_church_grid = voxel_streaming::chunk_of(IVec3::new(0, 350, 60));
	//     let controller = CameraLodController::default();
	//     let mut desired_chunks = HashSet::new();
	//     let mut desired_tiles = HashSet::new();
	//     add_near_chunks(&mut desired_chunks, grid, camera_chunk_in_church_grid, &controller);
	//     add_lod_tiles(&mut desired_tiles, grid, camera_chunk_in_church_grid, &controller, &streaming);

	//     let missing: Vec<_> = chunks
	//         .iter()
	//         .copied()
	//         .filter(|chunk| {
	//             !desired_chunks.contains(&ChunkKey { grid, chunk: *chunk })
	//                 && !desired_tiles.iter().any(|tile| tile.grid == grid && tile_covers_chunk(*tile, *chunk))
	//         })
	//         .collect();

	//     assert!(
	//         missing.is_empty(),
	//         "{} church chunks have no near or LOD request; camera_chunk={camera_chunk_in_church_grid:?}; sample={:?}",
	//         missing.len(),
	//         &missing[..missing.len().min(20)],
	//     );
	// }

	// fn tile_covers_chunk(tile: TileKey, chunk: IVec3) -> bool {
	//     chunk.cmpge(tile.min).all() && chunk.cmplt(tile.min + tile.size()).all()
	// }

	// fn lod_outer_radius(controller: &CameraLodController) -> i32 {
	//     let mut inner = controller.near_radius_chunks + 1;
	//     let mut outer = inner;
	//     for lod in 1..=controller.max_lod {
	//         let tile_size = 1i32 << lod;
	//         outer = inner + controller.rings_per_lod * tile_size;
	//         inner = outer + 1;
	//     }
	//     outer
	// }

	// fn assert_all_present_chunks_in_lod_range_are_covered(
	//     grid: Entity,
	//     center: IVec3,
	//     controller: &CameraLodController,
	//     streaming: &GridStreaming,
	//     desired_chunks: &HashSet<ChunkKey>,
	//     desired_tiles: &HashSet<TileKey>,
	// ) {
	//     let outer = lod_outer_radius(controller);
	//     let min = center - IVec3::splat(outer);
	//     let max = center + IVec3::splat(outer);
	//     let mut missing = Vec::new();
	//     streaming.presence().for_each_in_region(min, max, |chunk| {
	//         let key = ChunkKey { grid, chunk };
	//         let xz_dist = (chunk.x - center.x).abs().max((chunk.z - center.z).abs()).max((chunk.y - center.y).abs());
	//         if xz_dist <= controller.near_radius_chunks {
	//             if !desired_chunks.contains(&key) {
	//                 missing.push(chunk);
	//             }
	//             return;
	//         }
	//         if !desired_tiles.iter().any(|tile| tile.grid == grid && tile_covers_chunk(*tile, chunk)) {
	//             missing.push(chunk);
	//         }
	//     });
	//     assert!(missing.is_empty(), "present chunks were not covered at center {center:?}: {missing:?}");
	// }

	// fn load_church_chunks() -> HashSet<IVec3> {
	//     let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../res/Church_Of_St_Sophia.vox");
	//     let bytes = std::fs::read(&path).unwrap_or_else(|err| panic!("failed to read {path:?}: {err}"));
	//     let data = dot_vox::load_bytes(&bytes).expect("failed to parse church vox");
	//     let mut chunks = HashSet::new();

	//     #[derive(Clone, Copy)]
	//     struct Frame {
	//         translation: Vec3,
	//         rotation: Quat,
	//         flip: IVec3,
	//     }

	//     let mut stack = vec![(0u32, Frame { translation: Vec3::ZERO, rotation: Quat::IDENTITY, flip: IVec3::new(1, 1, -1) })];
	//     while let Some((scene_id, pose)) = stack.pop() {
	//         let Some(node) = data.scenes.get(scene_id as usize) else { continue };
	//         match node {
	//             dot_vox::SceneNode::Transform { frames, child, .. } => {
	//                 let Some(frame) = frames.first() else { continue };
	//                 let pos = frame.position().unwrap_or(dot_vox::Position { x: 0, y: 0, z: 0 });
	//                 let (rot, flip_vec) = frame
	//                     .orientation()
	//                     .map(|q| {
	//                         let (qarr, varr) = q.to_quat_scale();
	//                         let q = Quat::from_array(qarr);
	//                         (Quat::from_xyzw(q.x, q.z, -q.y, q.w), Vec3::from_array(varr).as_ivec3())
	//                     })
	//                     .unwrap_or((Quat::IDENTITY, IVec3::ONE));
	//                 stack.push((*child, Frame {
	//                     translation: pose.translation + pose.rotation * Vec3::new(pos.x as f32, pos.z as f32, -pos.y as f32),
	//                     rotation: pose.rotation * rot,
	//                     flip: pose.flip * IVec3::new(flip_vec.x, flip_vec.z, flip_vec.y),
	//                 }));
	//             }
	//             dot_vox::SceneNode::Group { children, .. } => {
	//                 for child in children {
	//                     stack.push((*child, pose));
	//                 }
	//             }
	//             dot_vox::SceneNode::Shape { models, .. } => {
	//                 for shape_model in models {
	//                     let Some(model) = data.models.get(shape_model.model_id as usize) else { continue };
	//                     let size = Vec3::new(model.size.x as f32, model.size.z as f32, model.size.y as f32);
	//                     let half = (size / 2.0).floor();
	//                     let pose_transform = Transform { translation: pose.translation, rotation: pose.rotation, scale: Vec3::ONE };
	//                     let half_offset = Transform::from_translation(-half * pose.flip.as_vec3());
	//                     for voxel in &model.voxels {
	//                         let local = IVec3::new(voxel.x as i32, voxel.z as i32, voxel.y as i32) * pose.flip + pose.flip.min(IVec3::ZERO);
	//                         let world_pos = (pose_transform * half_offset).transform_point(local.as_vec3()).as_ivec3();
	//                         chunks.insert(voxel_streaming::chunk_of(world_pos));
	//                     }
	//                 }
	//             }
	//         }
	//     }

	//     chunks
	// }
}
