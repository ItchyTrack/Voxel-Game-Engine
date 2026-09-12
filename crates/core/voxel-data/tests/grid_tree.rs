#[cfg(test)]
mod tests {
	use bevy::math::{IVec2, IVec3, UVec3, Vec3};
	use bevy::transform::components::Transform;
	use std::{collections::{HashMap, HashSet}, io::Cursor};
	use voxel_trees::{
		grid_tree::{CellKind, GridTree64, NonZeroVoxelRegion, U16Cell, SIZE},
		views::{GridTreeView, GridView},
	};
	use bincode;

	fn tree_voxels(tree: &GridTree64<U16Cell>) -> HashMap<UVec3, u16> {
		let mut m = HashMap::new();
		for (origin, size, value) in tree.iter() {
			for dx in 0..size {
				for dy in 0..size {
					for dz in 0..size {
						let pos = origin + UVec3::new(dx, dy, dz);
						assert!(m.insert(pos, value).is_none(), "iter overlapped at {pos:?}");
					}
				}
			}
		}
		m
	}

	fn root_only_stream(root_depth: u8, item_count: u64) -> Vec<u8> {
		let mut bytes = Vec::new();
		bytes.extend_from_slice(&0i32.to_le_bytes());
		bytes.extend_from_slice(&0i32.to_le_bytes());
		bytes.extend_from_slice(&0i32.to_le_bytes());
		bytes.push(root_depth);
		bytes.extend_from_slice(&item_count.to_le_bytes());
		bytes.extend_from_slice(&0u64.to_le_bytes());
		bytes.extend_from_slice(&0u64.to_le_bytes());
		bytes
	}

	fn bounds_from_positions(positions: impl Iterator<Item = UVec3>) -> Option<NonZeroVoxelRegion> {
		positions.fold(None, |bounds, pos| {
			let pos = pos.as_ivec3();
			Some(match bounds {
				Some(existing) => NonZeroVoxelRegion::from_min_max(existing.min().min(pos), existing.max().max(pos)).unwrap(),
				None => NonZeroVoxelRegion::from_min_max(pos, pos).unwrap(),
			})
		})
	}

	#[test]
	fn basic_insert_get_remove() {
		let mut t = GridTree64::<U16Cell>::new();
		assert!(!t.insert(&UVec3::new(3, 4, 5), 7));
		assert_eq!(t.get(UVec3::new(3, 4, 5)), Some(7));
		assert!(t.remove(&UVec3::new(3, 4, 5)));
		assert_eq!(t.get(UVec3::new(3, 4, 5)), None);
	}

	#[test]
	fn negative_space_is_not_representable() {
		let t = GridTree64::<U16Cell>::new();
		assert_eq!(t.get(UVec3::new(0, 0, 0)), None);
	}

	#[test]
	fn view_exposes_root_metadata_and_children() {
		let mut t = GridTree64::<U16Cell>::new();
		t.add_area(&UVec3::new(0, 0, 0), UVec3::splat(16), 7);
		let view = &t;
		let root = view.root();
		assert_eq!(root.handle, 0);
		assert_eq!(root.depth, 1);
		assert_eq!(root.origin, UVec3::ZERO);
		assert_eq!(view.root_pos(), UVec3::new(0, 0, 0));
		let children: Vec<_> = view.occupied_children(root).collect();
		assert_eq!(children.len(), SIZE as usize * SIZE as usize * SIZE as usize);
		assert!(children.iter().all(|child| child.kind == CellKind::Data));
	}

	#[test]
	fn serialization_roundtrips() {
		let mut tree = GridTree64::<U16Cell>::new();
		tree.add_area(&UVec3::new(2, 4, 6), UVec3::new(6, 5, 4), 3);
		tree.add_area(&UVec3::new(12, 10, 3), UVec3::new(5, 7, 2), 9);
		tree.remove_area(&UVec3::new(3, 5, 7), UVec3::new(2, 2, 2));
		tree.insert(&UVec3::new(31, 1, 1), 12);

		let encoded = bincode::serialize(&tree).expect("serialize grid tree");
		let decoded: GridTree64<U16Cell> = bincode::deserialize(&encoded).expect("deserialize grid tree");
		assert_eq!(decoded.iter().collect::<Vec<_>>(), tree.iter().collect::<Vec<_>>());
	}

	#[test]
	fn region_queries_match_iterator() {
		let mut t = GridTree64::<U16Cell>::new();
		for x in 0..16 {
			for y in 0..16 {
				for z in 0..16 {
					if (x + y + z) % 5 == 0 {
						t.insert(&UVec3::new(x, y, z), 1);
					}
				}
			}
		}
		let region = NonZeroVoxelRegion::from_min_max(IVec3::new(4, 4, 4), IVec3::new(10, 10, 10)).unwrap();
		let mut actual = HashMap::new();
		t.for_each_leaf_in_region(region, |origin, size, value| {
			for dx in 0..size {
				for dy in 0..size {
					for dz in 0..size {
						actual.insert(origin + UVec3::new(dx, dy, dz), value);
					}
				}
			}
		});
		let expected: HashMap<_, _> = tree_voxels(&t)
			.into_iter()
			.filter(|(pos, _)| pos.cmpge(UVec3::new(4, 4, 4)).all() && pos.cmple(UVec3::new(10, 10, 10)).all())
			.collect();
		assert_eq!(actual, expected);
	}

	#[test]
	fn voxel_region_query_clips_compressed_leaves() {
		let mut tree = GridTree64::<U16Cell>::new();
		tree.add_area(&UVec3::ZERO, UVec3::splat(16), 7);
		let region = NonZeroVoxelRegion::from_min_max(IVec3::ONE, IVec3::splat(2)).unwrap();
		let mut actual = HashSet::new();

		tree.for_each_in_region(region, |pos, value| {
			assert_eq!(value, 7);
			actual.insert(pos);
		});

		let expected = (1..=2)
			.flat_map(|x| (1..=2).flat_map(move |y| (1..=2).map(move |z| UVec3::new(x, y, z))))
			.collect();
		assert_eq!(actual, expected);
	}

	#[test]
	fn occupied_bounds_queries_match_oracle() {
		let mut t = GridTree64::<U16Cell>::new();
		t.add_area(&UVec3::new(2, 3, 4), UVec3::new(5, 6, 7), 3);
		t.remove_area(&UVec3::new(4, 5, 6), UVec3::new(2, 2, 2));
		t.insert(&UVec3::new(20, 1, 9), 8);

		let voxels = tree_voxels(&t);
		assert_eq!(t.occupied_bounds(), bounds_from_positions(voxels.keys().copied()));

		let region = NonZeroVoxelRegion::from_min_max(IVec3::new(3, 4, 5), IVec3::new(8, 9, 10)).unwrap();
		let expected = bounds_from_positions(voxels.keys().copied().filter(|pos| region.contains(pos.as_ivec3())));
		assert_eq!(t.occupied_bounds_in_region(region), expected);

		let empty_region = NonZeroVoxelRegion::from_min_max(IVec3::new(30, 30, 30), IVec3::new(35, 35, 35)).unwrap();
		assert_eq!(t.occupied_bounds_in_region(empty_region), None);
	}

	#[test]
	fn occupied_tile_cover_matches_oracle() {
		let mut t = GridTree64::<U16Cell>::new();
		for x in 0..24 {
			for y in 0..20 {
				for z in 0..18 {
					if (x + 2 * y + 3 * z) % 7 == 0 {
						t.insert(&UVec3::new(x, y, z), 1);
					}
				}
			}
		}
		let region = NonZeroVoxelRegion::from_min_max(IVec3::new(3, 2, 1), IVec3::new(21, 17, 15)).unwrap();
		let tile_size = 5;

		let mut actual = HashSet::new();
		t.for_each_occupied_tile_cover(region.min().as_uvec3(), region.max().as_uvec3(), tile_size, |tile| {
			actual.insert(tile);
		});

		let expected: HashSet<_> = tree_voxels(&t)
			.into_keys()
			.filter(|pos| region.contains(pos.as_ivec3()))
			.map(|pos| pos / UVec3::splat(tile_size) * tile_size)
			.collect();
		assert_eq!(actual, expected);
	}

	#[test]
	fn occupied_tile_cover_does_not_report_occupancy_from_an_adjacent_tile() {
		let mut t = GridTree64::<U16Cell>::new();
		t.add_area(&UVec3::new(2, 0, 0), UVec3::splat(2), 1);
		let region = NonZeroVoxelRegion::from_single(IVec3::ZERO);
		let mut actual = Vec::new();

		t.for_each_occupied_tile_cover(region.min().as_uvec3(), region.max().as_uvec3(), 2, |tile| actual.push(tile));

		assert!(actual.is_empty(), "query returned phantom tiles: {actual:?}");
	}

	#[test]
	fn raycast_hits() {
		let mut t = GridTree64::<U16Cell>::new();
		t.insert(&UVec3::new(0, 0, 10), 1);
		let tf = Transform {
			translation: Vec3::new(0.5, 0.5, -1.0),
			rotation: bevy::math::Quat::from_rotation_arc(Vec3::Z, Vec3::Z),
			scale: Vec3::ONE,
		};
		let (pos, _, _) = t.raycast(&tf, Some(100.0)).expect("raycast hit");
		assert_eq!(pos, UVec3::new(0, 0, 10));
	}

	#[test]
	fn root_promotion_preserves_existing_and_new_data() {
		let mut t = GridTree64::<U16Cell>::new();
		assert!(!t.insert(&UVec3::new(0, 0, 0), 1));
		t.add_area(&UVec3::new(16, 0, 0), UVec3::ONE, 2);
		assert_eq!(t.get(UVec3::new(0, 0, 0)), Some(1));
		assert_eq!(t.get(UVec3::new(16, 0, 0)), Some(2));
	}

	#[test]
	fn single_voxel_pair_build_uses_canonical_root_origin() {
		let mut t = GridTree64::<U16Cell>::new();
		t.add_single_voxels(&[(UVec3::new(5, 0, 0), 7), (UVec3::new(8, 0, 0), 9)]);
		assert_eq!(t.get(UVec3::new(5, 0, 0)), Some(7));
		assert_eq!(t.get(UVec3::new(8, 0, 0)), Some(9));
	}

	#[test]
	fn single_voxel_pair_depth_two_builds_directly_and_keeps_last_duplicate() {
		let mut t = GridTree64::<U16Cell>::new();
		t.add_single_voxels(&[(UVec3::new(0, 0, 0), 1), (UVec3::new(63, 63, 63), 2), (UVec3::new(0, 0, 0), 3)]);
		assert_eq!(t.len(), 2);
		assert_eq!(t.get(UVec3::new(0, 0, 0)), Some(3));
		assert_eq!(t.get(UVec3::new(63, 63, 63)), Some(2));
	}

	#[test]
	fn single_voxel_pair_build_collapses_uniform_leaf_nodes() {
		let voxels: Vec<_> = (0..16)
			.flat_map(|z| (0..16).flat_map(move |y| (0..16).map(move |x| (UVec3::new(x, y, z), 7))))
			.collect();
		let mut t = GridTree64::<U16Cell>::new();
		t.add_single_voxels(&voxels);
		assert_eq!(t.len(), 16 * 16 * 16);
		assert_eq!(t.iter().count(), 64);
	}

	#[test]
	fn single_voxel_insert_uses_canonical_root_origin() {
		let mut t = GridTree64::<U16Cell>::new();
		assert!(!t.insert(&UVec3::new(5, 0, 0), 7));
		assert!(!t.insert(&UVec3::new(8, 0, 0), 9));
		assert_eq!(t.get(UVec3::new(5, 0, 0)), Some(7));
		assert_eq!(t.get(UVec3::new(8, 0, 0)), Some(9));
	}

	#[test]
	fn apply_sdf_fills_single_voxel_from_exact_bounds_on_empty_tree() {
		let mut t = GridTree64::<U16Cell>::new();
		let sdf = |q: Vec3| if (q - Vec3::new(16.5, 0.5, 0.5)).length() < 0.1 { -1.0 } else { 1.0 };
		t.apply_sdf(Vec3::new(16.0, 0.0, 0.0), Vec3::new(17.0, 1.0, 1.0), &sdf, IVec2::splat(3), 2, 2);
		assert_eq!(t.get(UVec3::new(16, 0, 0)), Some(2));
	}

	#[test]
	fn apply_sdf_fills_single_voxel_from_exact_bounds_after_root_growth() {
		let mut t = GridTree64::<U16Cell>::new();
		t.insert(&UVec3::new(0, 0, 0), 1);
		let sdf = |q: Vec3| if (q - Vec3::new(16.5, 0.5, 0.5)).length() < 0.1 { -1.0 } else { 1.0 };
		t.apply_sdf(Vec3::new(16.0, 0.0, 0.0), Vec3::new(17.0, 1.0, 1.0), &sdf, IVec2::splat(3), 2, 2);
		assert_eq!(t.get(UVec3::new(0, 0, 0)), Some(1));
		assert_eq!(t.get(UVec3::new(16, 0, 0)), Some(2));
	}



	#[test]
	fn read_from_rejects_trailing_bytes() {
		let mut bytes = root_only_stream(0, 0);
		bytes.push(123);
		let result = GridTree64::<U16Cell>::read_from(&mut Cursor::new(bytes));
		assert!(result.is_err(), "read_from accepted trailing bytes");
	}

	#[test]
	fn read_from_rejects_invalid_root_depth() {
		let result = GridTree64::<U16Cell>::read_from(&mut Cursor::new(root_only_stream(255, 0)));
		assert!(result.is_err(), "read_from accepted an impossible root depth");
	}

	#[test]
	fn read_from_recomputes_item_count_from_contents() {
		let tree = GridTree64::<U16Cell>::read_from(&mut Cursor::new(root_only_stream(0, 99))).expect("deserializing minimal tree");
		assert_eq!(tree.len(), 0, "header item_count should not outrank actual node contents");
	}
}
