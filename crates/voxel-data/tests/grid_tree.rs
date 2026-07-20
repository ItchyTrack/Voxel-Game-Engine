#[cfg(test)]
mod tests {
	use bevy::math::{IVec2, IVec3, U16Vec3, Vec3};
	use bevy::transform::components::Transform;
	use std::{collections::{HashMap, HashSet}, io::Cursor};
	use voxel_data::{
		grid_tree::{CellKind, GridRegion, SIZE},
		voxel_grid_tree::PackedGridTree,
	};
	use bincode;

	fn p(x: u16, y: u16, z: u16) -> U16Vec3 { U16Vec3::new(x, y, z) }

	fn tree_voxels(tree: &PackedGridTree) -> HashMap<U16Vec3, u16> {
		let mut m = HashMap::new();
		for (origin, size, value) in tree.iter() {
			for dx in 0..size {
				for dy in 0..size {
					for dz in 0..size {
						let pos = origin + U16Vec3::new(dx, dy, dz);
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

	fn bounds_from_positions(positions: impl Iterator<Item = U16Vec3>) -> Option<GridRegion> {
		positions.fold(None, |bounds, pos| {
			let pos = pos.as_ivec3();
			Some(match bounds {
				Some(existing) => GridRegion::from_min_max_inclusive(existing.min.min(pos), existing.max_inclusive().max(pos)).unwrap(),
				None => GridRegion::from_min_max_inclusive(pos, pos).unwrap(),
			})
		})
	}

	#[test]
	fn basic_insert_get_remove() {
		let mut t = PackedGridTree::new();
		assert!(!t.insert(&p(3, 4, 5), 7));
		assert_eq!(t.get(&p(3, 4, 5)), Some(7));
		assert!(t.remove(&p(3, 4, 5)));
		assert_eq!(t.get(&p(3, 4, 5)), None);
	}

	#[test]
	fn negative_space_is_not_representable() {
		let t = PackedGridTree::new();
		assert_eq!(t.get(&p(0, 0, 0)), None);
	}

	#[test]
	fn view_exposes_root_metadata_and_children() {
		let mut t = PackedGridTree::new();
		t.add_area(&p(0, 0, 0), IVec3::splat(16), 7);
		let view = t.view();
		let root = view.root();
		assert_eq!(root.index, 0);
		assert_eq!(root.depth, 1);
		assert_eq!(root.origin, IVec3::ZERO);
		assert_eq!(view.root_pos(), p(0, 0, 0));
		let children: Vec<_> = view.occupied_children(root).collect();
		assert_eq!(children.len(), SIZE as usize * SIZE as usize * SIZE as usize);
		assert!(children.iter().all(|child| child.kind() == CellKind::Data));
	}

	#[test]
	fn serialization_roundtrips() {
		let mut tree = PackedGridTree::new();
		tree.add_area(&p(2, 4, 6), IVec3::new(6, 5, 4), 3);
		tree.add_area(&p(12, 10, 3), IVec3::new(5, 7, 2), 9);
		tree.remove_area(&p(3, 5, 7), IVec3::new(2, 2, 2));
		tree.insert(&p(31, 1, 1), 12);

		let encoded = bincode::serialize(&tree).expect("serialize grid tree");
		let decoded: PackedGridTree = bincode::deserialize(&encoded).expect("deserialize grid tree");
		assert_eq!(decoded.iter().collect::<Vec<_>>(), tree.iter().collect::<Vec<_>>());
	}

	#[test]
	fn region_queries_match_iterator() {
		let mut t = PackedGridTree::new();
		for x in 0..16 {
			for y in 0..16 {
				for z in 0..16 {
					if (x + y + z) % 5 == 0 {
						t.insert(&p(x, y, z), 1);
					}
				}
			}
		}
		let region = GridRegion::from_min_max_inclusive(IVec3::new(4, 4, 4), IVec3::new(10, 10, 10)).unwrap();
		let mut actual = HashMap::new();
		t.for_each_in_region(region, |origin, size, value| {
			for dx in 0..size {
				for dy in 0..size {
					for dz in 0..size {
						actual.insert(origin + U16Vec3::new(dx, dy, dz), value);
					}
				}
			}
		});
		let expected: HashMap<_, _> = tree_voxels(&t)
			.into_iter()
			.filter(|(pos, _)| pos.cmpge(p(4, 4, 4)).all() && pos.cmple(p(10, 10, 10)).all())
			.collect();
		assert_eq!(actual, expected);
	}

	#[test]
	fn occupied_bounds_queries_match_oracle() {
		let mut t = PackedGridTree::new();
		t.add_area(&p(2, 3, 4), IVec3::new(5, 6, 7), 3);
		t.remove_area(&p(4, 5, 6), IVec3::new(2, 2, 2));
		t.insert(&p(20, 1, 9), 8);

		let voxels = tree_voxels(&t);
		assert_eq!(t.occupied_bounds(), bounds_from_positions(voxels.keys().copied()));

		let region = GridRegion::from_min_max_inclusive(IVec3::new(3, 4, 5), IVec3::new(8, 9, 10)).unwrap();
		let expected = bounds_from_positions(voxels.keys().copied().filter(|pos| region.contains(pos.as_ivec3())));
		assert_eq!(t.occupied_bounds_in_region(region), expected);

		let empty_region = GridRegion::from_min_max_inclusive(IVec3::new(30, 30, 30), IVec3::new(35, 35, 35)).unwrap();
		assert_eq!(t.occupied_bounds_in_region(empty_region), None);
	}

	#[test]
	fn occupied_tile_cover_matches_oracle() {
		let mut t = PackedGridTree::new();
		for x in 0..24 {
			for y in 0..20 {
				for z in 0..18 {
					if (x + 2 * y + 3 * z) % 7 == 0 {
						t.insert(&p(x, y, z), 1);
					}
				}
			}
		}
		let region = GridRegion::from_min_max_inclusive(IVec3::new(3, 2, 1), IVec3::new(21, 17, 15)).unwrap();
		let tile_size = 5;

		let mut actual = HashSet::new();
		t.for_each_occupied_tile_cover(region, tile_size, |tile| {
			actual.insert(tile);
		});

		let expected: HashSet<_> = tree_voxels(&t)
			.into_keys()
			.filter(|pos| region.contains(pos.as_ivec3()))
			.map(|pos| pos.as_ivec3().div_euclid(IVec3::splat(tile_size)) * tile_size)
			.collect();
		assert_eq!(actual, expected);
	}

	#[test]
	fn occupied_tile_cover_does_not_report_occupancy_from_an_adjacent_tile() {
		let mut t = PackedGridTree::new();
		t.add_area(&p(2, 0, 0), IVec3::splat(2), 1);
		let region = GridRegion::from_min_size(IVec3::ZERO, IVec3::ONE).unwrap();
		let mut actual = Vec::new();

		t.for_each_occupied_tile_cover(region, 2, |tile| actual.push(tile));

		assert!(actual.is_empty(), "query returned phantom tiles: {actual:?}");
	}

	#[test]
	fn raycast_hits() {
		let mut t = PackedGridTree::new();
		t.insert(&p(0, 0, 10), 1);
		let tf = Transform {
			translation: Vec3::new(0.5, 0.5, -1.0),
			rotation: bevy::math::Quat::from_rotation_arc(Vec3::Z, Vec3::Z),
			scale: Vec3::ONE,
		};
		let (pos, _, _) = t.raycast(&tf, Some(100.0)).expect("raycast hit");
		assert_eq!(pos, p(0, 0, 10));
	}

	#[test]
	fn root_promotion_preserves_existing_and_new_data() {
		let mut t = PackedGridTree::new();
		assert!(!t.insert(&p(0, 0, 0), 1));
		t.add_area(&p(16, 0, 0), IVec3::ONE, 2);
		assert_eq!(t.get(&p(0, 0, 0)), Some(1));
		assert_eq!(t.get(&p(16, 0, 0)), Some(2));
	}

	#[test]
	fn single_voxel_batch_build_uses_canonical_root_origin() {
		let mut t = PackedGridTree::new();
		t.add_single_voxels(&[(p(5, 0, 0), 7), (p(8, 0, 0), 9)]);
		assert_eq!(t.get(&p(5, 0, 0)), Some(7));
		assert_eq!(t.get(&p(8, 0, 0)), Some(9));
	}

	#[test]
	fn single_voxel_pair_build_uses_canonical_root_origin() {
		let mut t = PackedGridTree::new();
		assert!(!t.insert(&p(5, 0, 0), 7));
		assert!(!t.insert(&p(8, 0, 0), 9));
		assert_eq!(t.get(&p(5, 0, 0)), Some(7));
		assert_eq!(t.get(&p(8, 0, 0)), Some(9));
	}

	#[test]
	fn apply_sdf_fills_single_voxel_from_exact_bounds_on_empty_tree() {
		let mut t = PackedGridTree::new();
		let sdf = |q: Vec3| if (q - Vec3::new(16.5, 0.5, 0.5)).length() < 0.1 { -1.0 } else { 1.0 };
		t.apply_sdf(Vec3::new(16.0, 0.0, 0.0), Vec3::new(17.0, 1.0, 1.0), &sdf, IVec2::splat(3), 2, 2);
		assert_eq!(t.get(&p(16, 0, 0)), Some(2));
	}

	#[test]
	fn apply_sdf_fills_single_voxel_from_exact_bounds_after_root_growth() {
		let mut t = PackedGridTree::new();
		t.insert(&p(0, 0, 0), 1);
		let sdf = |q: Vec3| if (q - Vec3::new(16.5, 0.5, 0.5)).length() < 0.1 { -1.0 } else { 1.0 };
		t.apply_sdf(Vec3::new(16.0, 0.0, 0.0), Vec3::new(17.0, 1.0, 1.0), &sdf, IVec2::splat(3), 2, 2);
		assert_eq!(t.get(&p(0, 0, 0)), Some(1));
		assert_eq!(t.get(&p(16, 0, 0)), Some(2));
	}



	#[test]
	fn read_from_rejects_trailing_bytes() {
		let mut bytes = root_only_stream(0, 0);
		bytes.push(123);
		let result = PackedGridTree::read_from(&mut Cursor::new(bytes));
		assert!(result.is_err(), "read_from accepted trailing bytes");
	}

	#[test]
	fn read_from_rejects_invalid_root_depth() {
		let result = PackedGridTree::read_from(&mut Cursor::new(root_only_stream(255, 0)));
		assert!(result.is_err(), "read_from accepted an impossible root depth");
	}

	#[test]
	fn read_from_recomputes_item_count_from_contents() {
		let tree = PackedGridTree::read_from(&mut Cursor::new(root_only_stream(0, 99))).expect("deserializing minimal tree");
		assert_eq!(tree.len(), 0, "header item_count should not outrank actual node contents");
	}
}
