use super::{assert_matches_oracle, lcg, p, tree_voxels};
use bevy::math::{I16Vec3, IVec3};
use std::collections::HashMap;
use voxel_data::{grid_tree::GridRegion, voxel_grid_tree::VoxelGridTree};

fn in_half_open_region(pos: I16Vec3, min: I16Vec3, size: IVec3) -> bool {
	let pos = pos.as_ivec3();
	let min = min.as_ivec3();
	let end = min + size;
	pos.cmpge(min).all() && pos.cmplt(end).all()
}

#[test]
fn split_region_empty_size_is_noop() {
	let mut tree = VoxelGridTree::new();
	tree.add_area(&p(0, 0, 0), IVec3::splat(8), 5);
	let before = tree_voxels(&tree);

	assert!(GridRegion::from_min_size(p(2, 2, 2).as_ivec3(), IVec3::ZERO).is_none());
	assert_matches_oracle(&tree, &before);
}

#[test]
fn split_region_disjoint_returns_empty_and_preserves_source() {
	let mut tree = VoxelGridTree::new();
	tree.add_area(&p(0, 0, 0), IVec3::splat(8), 5);
	let before = tree_voxels(&tree);

	let moved = tree.split_region(GridRegion::from_min_size(p(64, 64, 64).as_ivec3(), IVec3::splat(8)).unwrap());

	assert!(moved.is_empty());
	assert_matches_oracle(&tree, &before);
}

#[test]
fn split_region_partial_uniform_cell_matches_oracle() {
	let mut tree = VoxelGridTree::new();
	tree.add_area(&p(-8, -8, -8), IVec3::splat(24), 2);
	tree.add_area(&p(0, 0, 0), IVec3::splat(6), 9);
	let before = tree_voxels(&tree);
	let region_min = p(-3, -2, -1);
	let region_size = IVec3::new(13, 11, 9);

	let moved = tree.split_region(GridRegion::from_min_size(region_min.as_ivec3(), region_size).unwrap());

	let mut source_oracle = before.clone();
	let mut moved_oracle = HashMap::new();
	for (pos, value) in before {
		if in_half_open_region(pos, region_min, region_size) {
			moved_oracle.insert(pos, value);
			source_oracle.remove(&pos);
		}
	}
	assert_matches_oracle(&tree, &source_oracle);
	assert_matches_oracle(&moved, &moved_oracle);
}

#[test]
fn merge_tree_empty_source_is_noop() {
	let mut dest = VoxelGridTree::new();
	dest.add_area(&p(0, 0, 0), IVec3::splat(8), 4);
	let before = tree_voxels(&dest);
	let src = VoxelGridTree::new();

	dest.merge_tree(&src, IVec3::new(10, 10, 10));

	assert_matches_oracle(&dest, &before);
}

#[test]
fn merge_tree_negative_offset_matches_oracle() {
	let mut src = VoxelGridTree::new();
	src.add_area(&p(10, 10, 10), IVec3::splat(5), 7);
	src.add_area(&p(12, 12, 12), IVec3::splat(2), 8);
	let mut dest = VoxelGridTree::new();
	dest.add_area(&p(-2, -2, -2), IVec3::splat(8), 1);
	let mut oracle = tree_voxels(&dest);
	let src_voxels = tree_voxels(&src);
	let offset = IVec3::new(-14, -13, -12);
	for (pos, value) in src_voxels {
		oracle.insert((pos.as_ivec3() + offset).as_i16vec3(), value);
	}

	dest.merge_tree(&src, offset);

	assert_matches_oracle(&dest, &oracle);
}

#[test]
fn merge_uses_child_alignment_when_parent_origin_is_offset() {
	let mut src = VoxelGridTree::new();
	src.add_area(&p(0, 0, 0), IVec3::splat(64), 7);

	let mut dest = VoxelGridTree::new();
	dest.add_area(&p(-64, -64, -64), IVec3::splat(4), 1);
	let offset = IVec3::new(16, 0, 0);
	let mut oracle = tree_voxels(&dest);
	for (pos, value) in tree_voxels(&src) {
		oracle.insert((pos.as_ivec3() + offset).as_i16vec3(), value);
	}

	dest.merge_tree(&src, offset);

	assert_eq!(dest.get(&p(16, 0, 0)), Some(7));
	assert_eq!(dest.get(&p(79, 63, 63)), Some(7));
	assert_eq!(dest.get(&p(-64, -64, -64)), Some(1));
	assert_matches_oracle(&dest, &oracle);
}

#[test]
fn merge_large_source_over_small_destination_preserves_destination_holes() {
	let mut src = VoxelGridTree::new();
	src.add_area(&p(0, 0, 0), IVec3::splat(32), 7);
	src.remove_area(&p(10, 10, 10), IVec3::splat(4));

	let mut dest = VoxelGridTree::new();
	dest.add_area(&p(9, 9, 9), IVec3::splat(6), 1);
	let mut oracle = tree_voxels(&dest);
	let src_voxels = tree_voxels(&src);
	for (pos, value) in src_voxels {
		oracle.insert(pos, value);
	}

	dest.merge_tree(&src, IVec3::ZERO);

	assert_eq!(dest.get(&p(9, 9, 9)), Some(7));
	assert_eq!(dest.get(&p(10, 10, 10)), Some(1));
	assert_eq!(dest.get(&p(13, 13, 13)), Some(1));
	assert_eq!(dest.get(&p(14, 14, 14)), Some(7));
	assert_matches_oracle(&dest, &oracle);
}

#[test]
fn split_then_merge_with_offset_matches_oracle() {
	let mut tree = VoxelGridTree::new();
	tree.add_area(&p(0, 0, 0), IVec3::splat(16), 3);
	tree.add_area(&p(4, 4, 4), IVec3::splat(5), 6);
	let before = tree_voxels(&tree);
	let region_min = p(2, 3, 4);
	let region_size = IVec3::new(9, 8, 7);
	let offset = IVec3::new(20, -1, 5);

	let moved = tree.split_region(GridRegion::from_min_size(region_min.as_ivec3(), region_size).unwrap());
	tree.merge_tree(&moved, offset);

	let mut oracle = before.clone();
	let mut moved_oracle = Vec::new();
	for pos in before.keys().copied().collect::<Vec<_>>() {
		if in_half_open_region(pos, region_min, region_size) {
			let value = oracle.remove(&pos).unwrap();
			moved_oracle.push(((pos.as_ivec3() + offset).as_i16vec3(), value));
		}
	}
	for (pos, value) in moved_oracle {
		oracle.insert(pos, value);
	}
	assert_matches_oracle(&tree, &oracle);
}

#[test]
fn randomized_split_and_merge_match_oracle() {
	let mut tree = VoxelGridTree::new();
	let mut oracle = HashMap::new();
	let mut seed = 0x1234_5678_abcd_ef01;
	for _ in 0..400 {
		let min = p((lcg(&mut seed) % 24) as i16 - 12, (lcg(&mut seed) % 24) as i16 - 12, (lcg(&mut seed) % 24) as i16 - 12);
		let size = IVec3::new((lcg(&mut seed) % 5 + 1) as i32, (lcg(&mut seed) % 5 + 1) as i32, (lcg(&mut seed) % 5 + 1) as i32);
		let value = (lcg(&mut seed) % 11 + 1) as u16;
		tree.add_area(&min, size, value);
		for x in 0..size.x as i16 {
			for y in 0..size.y as i16 {
				for z in 0..size.z as i16 {
					oracle.insert(min + p(x, y, z), value);
				}
			}
		}
	}

	for _ in 0..80 {
		let region_min = p((lcg(&mut seed) % 24) as i16 - 12, (lcg(&mut seed) % 24) as i16 - 12, (lcg(&mut seed) % 24) as i16 - 12);
		let region_size = IVec3::new((lcg(&mut seed) % 10 + 1) as i32, (lcg(&mut seed) % 10 + 1) as i32, (lcg(&mut seed) % 10 + 1) as i32);
		let offset = IVec3::new((lcg(&mut seed) % 17) as i32 - 8, (lcg(&mut seed) % 17) as i32 - 8, (lcg(&mut seed) % 17) as i32 - 8);
		let before = oracle.clone();

		let moved = tree.split_region(GridRegion::from_min_size(region_min.as_ivec3(), region_size).unwrap());
		tree.merge_tree(&moved, offset);

		let mut moved_oracle = Vec::new();
		for (pos, value) in before {
			if in_half_open_region(pos, region_min, region_size) {
				oracle.remove(&pos);
				moved_oracle.push(((pos.as_ivec3() + offset).as_i16vec3(), value));
			}
		}
		for (pos, value) in moved_oracle {
			oracle.insert(pos, value);
		}
		assert_matches_oracle(&tree, &oracle);
	}
}
