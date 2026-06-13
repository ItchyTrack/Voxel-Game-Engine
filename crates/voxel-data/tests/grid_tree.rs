#[cfg(test)]
mod tests {
	use voxel_data::{voxel_grid_tree::{VoxelGridTree, PackedCell}, grid_tree::{GridTree, I32Coord, SIZE}};
	use bevy::math::{I16Vec3, IVec3, Vec3};
	use bevy::transform::components::Transform;
	use std::collections::HashMap;

	fn lcg(state: &mut u64) -> u64 {
		*state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
		*state >> 33
	}

	fn p(x: i16, y: i16, z: i16) -> I16Vec3 {
		I16Vec3::new(x, y, z)
	}

	/// Expand the tree's iterator into a flat voxel map, asserting cells never overlap.
	fn tree_voxels(tree: &VoxelGridTree) -> HashMap<I16Vec3, u16> {
		let mut m = HashMap::new();
		for (origin, size, value) in tree.iter() {
			for dx in 0..size as i16 {
				for dy in 0..size as i16 {
					for dz in 0..size as i16 {
						let pos = origin + I16Vec3::new(dx, dy, dz);
						assert!(m.insert(pos, value).is_none(), "iter overlapped at {pos:?}");
					}
				}
			}
		}
		m
	}

	/// Ground-truth check: get(), iter() expansion, and len() all agree with the oracle.
	fn assert_matches_oracle(tree: &VoxelGridTree, oracle: &HashMap<I16Vec3, u16>) {
		for (pos, v) in oracle {
			assert_eq!(tree.get(pos), Some(*v), "get({pos:?}) mismatch");
		}
		assert_eq!(&tree_voxels(tree), oracle, "iter expansion differs from oracle");
		assert_eq!(tree.len(), oracle.len() as u64, "len() mismatch");
	}

	// ---- basic API ----

	#[test]
	fn new_tree_is_empty() {
		let t = VoxelGridTree::new();
		assert!(t.is_empty());
		assert_eq!(t.len(), 0);
		assert_eq!(t.get(&p(0, 0, 0)), None);
		assert_eq!(t.get(&p(-5, 9, 1000)), None);
		assert!(!t.contains_key(&p(0, 0, 0)));
	}

	#[test]
	fn insert_then_get() {
		let mut t = VoxelGridTree::new();
		assert_eq!(t.insert(&p(3, 4, 5), 7), None);
		assert_eq!(t.get(&p(3, 4, 5)), Some(7));
		assert!(t.contains_key(&p(3, 4, 5)));
		assert_eq!(t.len(), 1);
		assert!(!t.is_empty());
	}

	#[test]
	fn insert_overwrite_returns_old_value() {
		let mut t = VoxelGridTree::new();
		t.insert(&p(1, 1, 1), 10);
		assert_eq!(t.insert(&p(1, 1, 1), 20), Some(10));
		assert_eq!(t.get(&p(1, 1, 1)), Some(20));
		assert_eq!(t.len(), 1, "overwrite must not change len");
	}

	#[test]
	fn insert_same_value_returns_old_and_noops() {
		let mut t = VoxelGridTree::new();
		t.insert(&p(2, 2, 2), 9);
		assert_eq!(t.insert(&p(2, 2, 2), 9), Some(9));
		assert_eq!(t.len(), 1);
	}

	#[test]
	fn remove_returns_value_absent_is_none() {
		let mut t = VoxelGridTree::new();
		t.insert(&p(5, 6, 7), 42);
		assert_eq!(t.remove(&p(0, 0, 0)), None);
		assert_eq!(t.remove(&p(5, 6, 7)), Some(42));
		assert_eq!(t.get(&p(5, 6, 7)), None);
		assert!(t.is_empty());
		assert_eq!(t.remove(&p(5, 6, 7)), None);
	}

	#[test]
	fn negative_coordinates_roundtrip() {
		let mut t = VoxelGridTree::new();
		t.insert(&p(-10, -20, -30), 3);
		t.insert(&p(-1, -1, -1), 4);
		t.insert(&p(-10, 5, -30), 5);
		assert_eq!(t.get(&p(-10, -20, -30)), Some(3));
		assert_eq!(t.get(&p(-1, -1, -1)), Some(4));
		assert_eq!(t.get(&p(-10, 5, -30)), Some(5));
		assert_eq!(t.len(), 3);
	}

	// ---- merging behavior ----

	#[test]
	fn root_node_never_merges() {
		// A uniform SIZE^3 cube that IS the whole tree stays unmerged: try_merge
		// requires a parent, and the root has none. So it yields SIZE^3 leaves.
		let mut t = VoxelGridTree::new();
		for x in 0..SIZE as i16 {
			for y in 0..SIZE as i16 {
				for z in 0..SIZE as i16 {
					t.insert(&p(x, y, z), 1);
				}
			}
		}
		assert_eq!(t.iter().count() as u64, (SIZE as u64).pow(3));
		assert!(t.iter().all(|(_, size, _)| size == 1));
		assert_eq!(t.len(), (SIZE as u64).pow(3));
	}

	#[test]
	fn full_child_node_merges_into_single_cell() {
		// With depth forced to 1 (anchor in another child cell), a uniformly
		// filled child cell collapses to one data cell of side SIZE.
		let mut t = VoxelGridTree::new();
		t.insert(&p(0, 0, 0), 1); // sets root_pos = origin (avoids negative-side growth)
		t.insert(&p(2 * SIZE as i16, 0, 0), 2); // anchor: forces root to depth 1
		for x in 0..SIZE as i16 {
			for y in 0..SIZE as i16 {
				for z in 0..SIZE as i16 {
					t.insert(&p(x, y, z), 1);
				}
			}
		}
		let cells: Vec<_> = t.iter().collect();
		assert!(cells.contains(&(p(0, 0, 0), SIZE as u16, 1)), "uniform child cell should merge to one size-{SIZE} cell, got {cells:?}");
		assert_eq!(t.len(), (SIZE as u64).pow(3) + 1);
	}

	#[test]
	fn mixed_values_do_not_merge() {
		let mut t = VoxelGridTree::new();
		for x in 0..SIZE as i16 {
			for y in 0..SIZE as i16 {
				for z in 0..SIZE as i16 {
					t.insert(&p(x, y, z), if (x + y + z) % 2 == 0 { 1 } else { 2 });
				}
			}
		}
		let cells: Vec<_> = t.iter().collect();
		assert!(cells.len() > 1, "non-uniform cube must not merge");
		assert!(cells.iter().all(|(_, size, _)| *size == 1));
	}

	#[test]
	fn remove_from_merged_region_splits_it() {
		let mut t = VoxelGridTree::new();
		t.insert(&p(0, 0, 0), 1);
		t.insert(&p(2 * SIZE as i16, 0, 0), 2); // force depth 1 so the cube merges
		for x in 0..SIZE as i16 {
			for y in 0..SIZE as i16 {
				for z in 0..SIZE as i16 {
					t.insert(&p(x, y, z), 1);
				}
			}
		}
		assert!(t.iter().any(|c| c == (p(0, 0, 0), SIZE as u16, 1)), "precondition: merged");
		assert_eq!(t.remove(&p(1, 2, 3)), Some(1));
		assert_eq!(t.get(&p(1, 2, 3)), None);
		assert_eq!(t.get(&p(0, 0, 0)), Some(1)); // neighbours survive the split
		assert_eq!(t.len(), (SIZE as u64).pow(3) - 1 + 1);
	}

	// ---- area helpers ----

	#[test]
	fn add_area_then_is_area_filled() {
		let mut t = VoxelGridTree::new();
		t.add_area(&p(0, 0, 0), IVec3::new(5, 5, 5), 8);
		assert!(t.is_area_filled(&p(0, 0, 0), IVec3::new(5, 5, 5)));
		assert!(!t.is_area_filled(&p(0, 0, 0), IVec3::new(6, 5, 5)));
		assert_eq!(t.len(), 125);
	}

	#[test]
	fn remove_area_clears_region() {
		let mut t = VoxelGridTree::new();
		t.add_area(&p(0, 0, 0), IVec3::new(8, 8, 8), 1);
		t.remove_area(&p(2, 2, 2), IVec3::new(3, 3, 3));
		assert_eq!(t.get(&p(3, 3, 3)), None);
		assert_eq!(t.get(&p(0, 0, 0)), Some(1));
		assert_eq!(t.len(), 8 * 8 * 8 - 3 * 3 * 3);
	}

	#[test]
	fn add_areas_matches_sequential_single_voxels_across_existing_merged_cells() {
		let mut t = VoxelGridTree::new();
		let mut oracle = HashMap::new();

		t.add_area(&p(-32, -32, -32), IVec3::splat(16), 1);
		for x in -32..-16 {
			for y in -32..-16 {
				for z in -32..-16 {
					oracle.insert(p(x, y, z), 1);
				}
			}
		}

		let mut batch = Vec::new();
		for i in 0..512i16 {
			let x = ((i * 17 + 11).rem_euclid(96)) - 48;
			let y = ((i * 29 + 7).rem_euclid(96)) - 48;
			let z = ((i * 43 + 3).rem_euclid(96)) - 48;
			let pos = p(x, y, z);
			let value = 10 + (i as u16 % 5);
			batch.push((pos, IVec3::ONE, value));
			oracle.insert(pos, value);
		}
		t.add_areas(&batch);

		assert_matches_oracle(&t, &oracle);
	}

	#[test]
	fn add_areas_matches_sequential_mixed_runs_and_single_voxels() {
		let mut batched = VoxelGridTree::new();
		let mut sequential = VoxelGridTree::new();
		let mut areas = Vec::new();

		for i in 0..256i16 {
			let pos = p(
				((i * 19 + 5).rem_euclid(120)) - 60,
				((i * 31 + 9).rem_euclid(120)) - 60,
				((i * 47 + 13).rem_euclid(120)) - 60,
			);
			let size = if i % 7 == 0 { IVec3::splat(4) } else if i % 5 == 0 { IVec3::new(3, 2, 5) } else { IVec3::ONE };
			let value = 1 + (i as u16 % 17);
			areas.push((pos, size, value));
			sequential.add_area(&pos, size, value);
		}

		batched.add_areas(&areas);
		assert_eq!(tree_voxels(&batched), tree_voxels(&sequential));
		assert_eq!(batched.len(), sequential.len());
	}

	// ---- iteration ----

	#[test]
	fn iter_covers_all_inserted_voxels() {
		let mut t = VoxelGridTree::new();
		let mut oracle = HashMap::new();
		for &(x, y, z, v) in &[(0i16, 0i16, 0i16, 1u16), (10, 0, 0, 2), (0, 31, 0, 3), (5, 5, 5, 4), (-7, 2, 9, 5)] {
			t.insert(&p(x, y, z), v);
			oracle.insert(p(x, y, z), v);
		}
		assert_matches_oracle(&t, &oracle);
	}

	// ---- raycast ----

	#[test]
	fn raycast_hits_voxel_along_z() {
		let mut t = VoxelGridTree::new();
		t.insert(&p(0, 0, 10), 1);
		let tf =
			Transform { translation: Vec3::new(0.5, 0.5, -1.0), rotation: bevy::math::Quat::from_rotation_arc(Vec3::Z, Vec3::Z), scale: Vec3::ONE };
		let hit = t.raycast(&tf, None);
		assert!(hit.is_some(), "ray straight at voxel should hit");
		let (pos, _normal, dist) = hit.unwrap();
		assert_eq!(pos, p(0, 0, 10));
		assert!(dist > 0.0);
	}

	#[test]
	fn raycast_miss_returns_none() {
		let mut t = VoxelGridTree::new();
		t.insert(&p(0, 0, 10), 1);
		let tf =
			Transform { translation: Vec3::new(0.5, 0.5, -1.0), rotation: bevy::math::Quat::from_rotation_arc(Vec3::Z, -Vec3::Z), scale: Vec3::ONE };
		assert_eq!(t.raycast(&tf, None), None);
	}

	#[test]
	fn raycast_empty_tree_is_none() {
		let t = VoxelGridTree::new();
		let tf = Transform::from_translation(Vec3::ZERO);
		assert_eq!(t.raycast(&tf, None), None);
	}

	#[test]
	fn merge_cascades_through_levels() {
		// A uniform 16^3 region (depth-2 tree) collapses each depth-1 subtree, so
		// every yielded cell has side > 1 and far fewer cells than voxels.
		let n = (SIZE as i16) * (SIZE as i16); // 16
		let mut t = VoxelGridTree::new();
		let mut oracle = HashMap::new();
		for x in 0..n {
			for y in 0..n {
				for z in 0..n {
					t.insert(&p(x, y, z), 7);
					oracle.insert(p(x, y, z), 7);
				}
			}
		}
		assert_matches_oracle(&t, &oracle);
		let cells: Vec<_> = t.iter().collect();
		assert!(cells.iter().all(|(_, size, _)| *size > 1), "interior should merge");
		assert!((cells.len() as u64) < oracle.len() as u64, "merging must reduce cell count");
	}

	#[test]
	fn overwrite_single_voxel_in_merged_region() {
		let mut t = VoxelGridTree::new();
		t.insert(&p(0, 0, 0), 1);
		t.insert(&p(2 * SIZE as i16, 0, 0), 9); // depth 1
		for x in 0..SIZE as i16 {
			for y in 0..SIZE as i16 {
				for z in 0..SIZE as i16 {
					t.insert(&p(x, y, z), 1);
				}
			}
		}
		// Splitting a merged cell by overwriting one voxel keeps the rest intact.
		assert_eq!(t.insert(&p(2, 2, 2), 5), Some(1));
		assert_eq!(t.get(&p(2, 2, 2)), Some(5));
		assert_eq!(t.get(&p(1, 1, 1)), Some(1));
		assert_eq!(t.len(), (SIZE as u64).pow(3) + 1);
	}

	#[test]
	fn get_outside_root_bounds_is_none() {
		let mut t = VoxelGridTree::new();
		t.insert(&p(0, 0, 0), 1);
		assert_eq!(t.get(&p(10_000, 0, 0)), None);
		assert_eq!(t.get(&p(-10_000, 0, 0)), None);
		assert_eq!(t.get(&p(0, 5, 0)), None);
	}

	#[test]
	fn extreme_coordinates_do_not_panic() {
		// i16 extremes must not panic or hang; the depth cap may skip them, which
		// is acceptable as long as it stays self-consistent.
		let mut t = VoxelGridTree::new();
		t.insert(&p(0, 0, 0), 1);
		t.insert(&p(i16::MAX, i16::MAX, i16::MAX), 2);
		t.insert(&p(i16::MIN, i16::MIN, i16::MIN), 3);
		// Whatever was actually stored must read back consistently.
		let voxels = tree_voxels(&t);
		for (pos, v) in &voxels {
			assert_eq!(t.get(pos), Some(*v));
		}
	}

	#[test]
	fn raycast_reports_entry_face_normal() {
		let mut t = VoxelGridTree::new();
		t.insert(&p(0, 0, 10), 1);
		let tf =
			Transform { translation: Vec3::new(0.5, 0.5, -1.0), rotation: bevy::math::Quat::from_rotation_arc(Vec3::Z, Vec3::Z), scale: Vec3::ONE };
		let (pos, normal, _) = t.raycast(&tf, None).unwrap();
		assert_eq!(pos, p(0, 0, 10));
		assert_eq!(normal, bevy::math::I8Vec3::new(0, 0, -1), "hit -Z face entering along +Z");
	}

	#[test]
	fn raycast_respects_max_length() {
		let mut t = VoxelGridTree::new();
		t.insert(&p(0, 0, 100), 1);
		let tf =
			Transform { translation: Vec3::new(0.5, 0.5, 0.0), rotation: bevy::math::Quat::from_rotation_arc(Vec3::Z, Vec3::Z), scale: Vec3::ONE };
		assert!(t.raycast(&tf, Some(10.0)).is_none(), "voxel beyond max_length must miss");
		assert!(t.raycast(&tf, Some(200.0)).is_some());
	}

	#[test]
	fn node_arena_bounded_under_churn() {
		// Compaction must reclaim dead nodes so the arena does not grow without
		// bound under repeated create/collapse churn (the old remove_node leak).
		let mut t = VoxelGridTree::new();
		// Spaced base voxels: each sits alone in its leaf, so toggling it actually
		// collapses (and would leak) a node.
		for i in 0..16i16 {
			for j in 0..16i16 {
				t.insert(&p(i * 4, j * 4, ((i + j) % 16) * 4), 1);
			}
		}
		let baseline = t.internals().0.len();
		for _ in 0..400 {
			for i in 0..16i16 {
				for j in 0..16i16 {
					let q = p(i * 4, j * 4, ((i + j) % 16) * 4);
					t.remove(&q);
					t.insert(&q, 2);
				}
			}
		}
		let after = t.internals().0.len();
		assert!(after <= baseline * 2, "arena must stay bounded by compaction ({baseline} -> {after})");
	}

	#[test]
	fn node_cap_is_handled_gracefully() {
		// Past the 15-bit node-offset limit, edits are skipped (warned), never
		// corrupting: everything the tree reports as stored must read back.
		let mut t = VoxelGridTree::new();
		let mut s = 0xC0FFEEu64;
		for _ in 0..400_000 {
			let q = p((lcg(&mut s) % 8000) as i16, (lcg(&mut s) % 8000) as i16, (lcg(&mut s) % 8000) as i16);
			t.insert(&q, 1);
		}
		// Whatever actually made it in must be self-consistent.
		for (pos, v) in &tree_voxels(&t) {
			assert_eq!(t.get(pos), Some(*v));
		}
	}

	// ---- region query ----

	#[test]
	fn for_each_in_region_matches_filtered_iter() {
		let mut t = VoxelGridTree::new();
		let mut s = 0xBEEFu64;
		for _ in 0..2_000 {
			let q = p((lcg(&mut s) % 80) as i16, (lcg(&mut s) % 80) as i16, (lcg(&mut s) % 80) as i16);
			t.insert(&q, 1);
		}
		let (lo, hi) = (p(10, 10, 10), p(40, 40, 40));
		let mut region: HashMap<I16Vec3, u16> = HashMap::new();
		t.for_each_in_region(lo, hi, |origin, size, v| {
			for dx in 0..size as i16 {
				for dy in 0..size as i16 {
					for dz in 0..size as i16 {
						let pos = origin + I16Vec3::new(dx, dy, dz);
						if pos.cmpge(lo).all() && pos.cmple(hi).all() {
							region.insert(pos, v);
						}
					}
				}
			}
		});
		let mut expected: HashMap<I16Vec3, u16> = HashMap::new();
		for (pos, v) in tree_voxels(&t) {
			if pos.cmpge(lo).all() && pos.cmple(hi).all() {
				expected.insert(pos, v);
			}
		}
		assert_eq!(region, expected);
	}

	// ---- differential model tests vs HashMap oracle ----

	fn run_model(lo: i16, hi: i16, n_ops: usize, seed: u64) {
		let mut t = VoxelGridTree::new();
		let mut oracle: HashMap<I16Vec3, u16> = HashMap::new();
		let mut s = seed;
		let span = (hi - lo) as u64;
		for op in 0..n_ops {
			let pos = p(lo + (lcg(&mut s) % span) as i16, lo + (lcg(&mut s) % span) as i16, lo + (lcg(&mut s) % span) as i16);
			if lcg(&mut s) % 3 == 0 {
				assert_eq!(t.remove(&pos), oracle.remove(&pos), "remove ret @ {pos:?} op {op}");
			} else {
				let v = 1 + (lcg(&mut s) % 5) as u16;
				assert_eq!(t.insert(&pos, v), oracle.insert(pos, v), "insert ret @ {pos:?} op {op}");
			}
			if op % 97 == 0 {
				assert_matches_oracle(&t, &oracle);
			}
		}
		assert_matches_oracle(&t, &oracle);
	}

	#[test]
	fn model_tiny_range() {
		run_model(0, 4, 20_000, 1);
	}

	#[test]
	fn model_small_range() {
		run_model(0, 16, 50_000, 2);
	}

	#[test]
	fn model_with_negatives() {
		run_model(-16, 16, 50_000, 3);
	}

	#[test]
	fn model_subgrid_range() {
		run_model(0, 64, 200_000, 4);
	}

	#[test]
	fn model_depth3() {
		run_model(0, 200, 40_000, 11);
	}

	// Depth-4 trees, kept under the node-arena cap so no inserts are skipped.
	#[test]
	fn model_depth4() {
		run_model(0, 300, 6_000, 12);
	}

	#[test]
	fn model_sparse_wide() {
		run_model(-500, 500, 6_000, 5);
	}

	// Root depth must stay minimal for in-range (0..63) coords (no drift).
	#[test]
	fn root_depth_stays_bounded_for_local_coords() {
		let mut tree = VoxelGridTree::new();
		let mut s: u64 = 0x9e3779b9;
		for i in 0..200_000u64 {
			let pos = p((lcg(&mut s) % 64) as i16, (lcg(&mut s) % 64) as i16, (lcg(&mut s) % 64) as i16);
			if lcg(&mut s) % 3 == 0 {
				tree.remove(&pos);
			} else {
				tree.insert(&pos, 1);
			}
			let (_, _, root_depth) = tree.internals();
			assert!(root_depth <= 3, "root_depth climbed to {} after {} ops (last pos {pos:?})", root_depth, i);
		}
	}

	// ---- i32 coordinate system (chunk-space) ----

	#[test]
	fn i32_coord_far_coordinates() {
		let mut t: GridTree<PackedCell, I32Coord> = GridTree::new();
		let far = IVec3::new(100_000, -50_000, 70_000);
		assert_eq!(t.insert(&far, 3), None);
		assert_eq!(t.get(&far), Some(3));
		t.insert(&(far + IVec3::new(1, 0, 0)), 4);
		assert_eq!(t.get(&far), Some(3));
		assert_eq!(t.get(&(far + IVec3::new(1, 0, 0))), Some(4));
		assert_eq!(t.len(), 2);
	}
}
