use bevy::math::{I8Vec3, IVec3, UVec3, Vec3};
use bevy::transform::components::Transform;

use crate::grid_tree::{NonZeroVoxelRegion, GridTree, GridType, U32Coord};

#[derive(Clone, Debug)]
pub struct SignedGridTree<G: GridType> {
	trees: [GridTree<G, U32Coord>; 8],
}

impl<G: GridType + Default> Default for SignedGridTree<G> {
	fn default() -> Self {
		Self { trees: std::array::from_fn(|_| GridTree::new()) }
	}
}

impl<G: GridType + Default> SignedGridTree<G> {
	pub fn new() -> Self { Self::default() }

	pub fn len(&self) -> u64 {
		self.trees.iter().map(GridTree::len).sum()
	}

	pub fn is_empty(&self) -> bool {
		self.trees.iter().all(GridTree::is_empty)
	}

	pub fn insert(&mut self, pos: IVec3, data: G::Data<'_>) -> bool {
		let (oct, local) = split_pos(pos);
		self.trees[oct].insert(&local, data)
	}

	pub fn get(&self, pos: IVec3) -> Option<G::Data<'_>> {
		let (oct, local) = split_pos(pos);
		self.trees[oct].get(&local)
	}

	pub fn remove(&mut self, pos: IVec3) -> bool {
		let (oct, local) = split_pos(pos);
		self.trees[oct].remove(&local)
	}

	pub fn add_area(&mut self, region: NonZeroVoxelRegion, data: G::Data<'_>) {
		for (oct, local) in split_region(region) {
			self.trees[oct].fill_region(local, data);
		}
	}

	pub fn remove_area(&mut self, region: NonZeroVoxelRegion) {
		for (oct, local) in split_region(region) {
			self.trees[oct].clear_region(local);
		}
	}

	pub fn any_in_region(&self, region: NonZeroVoxelRegion) -> bool {
		split_region(region).into_iter().any(|(oct, local)| self.trees[oct].any_in_region(local))
	}

	pub fn is_area_filled(&self, region: NonZeroVoxelRegion) -> bool {
		let parts = split_region(region);
		!parts.is_empty() && parts.into_iter().all(|(oct, local)| self.trees[oct].is_region_filled(local))
	}

	pub fn for_each_in_region(&self, region: NonZeroVoxelRegion, mut f: impl FnMut(IVec3, u32, G::Data<'_>)) {
		for (oct, local) in split_region(region) {
			self.trees[oct].for_each_in_region(local, |origin, size, value| f(join_region_origin(oct, origin, size), size, value));
		}
	}

	pub fn for_each_node_in_region(&self, region: NonZeroVoxelRegion, mut f: impl FnMut(IVec3, u32, bool)) {
		for (oct, local) in split_region(region) {
			self.trees[oct].for_each_node_in_region(local, |origin, size, is_leaf| f(join_region_origin(oct, origin, size), size, is_leaf));
		}
	}

	pub fn for_each_occupied_tile_cover(&self, region: NonZeroVoxelRegion, tile_size: i32, mut f: impl FnMut(IVec3)) {
		for (oct, local) in split_region(region) {
			self.trees[oct].for_each_occupied_tile_cover(local, tile_size, |tile| {
				f(join_region_origin(oct, tile.as_uvec3(), tile_size as u32));
			});
		}
	}

	pub fn raycast(&self, transform: &Transform, max_length: Option<f32>) -> Option<(IVec3, I8Vec3, f32)> {
		let world_dir = transform.rotation * Vec3::Z;
		self.trees.iter().enumerate().filter_map(|(oct, tree)| {
			let local_origin = world_to_local_point(oct, transform.translation);
			let local_dir = world_to_local_vector(oct, world_dir);
			let local_transform = Transform::from_translation(local_origin).looking_to(local_dir, Vec3::Y);
			tree.raycast(&local_transform, max_length).map(|(pos, normal, dist)| (join_pos(oct, pos), local_to_world_normal(oct, normal), dist))
		}).min_by(|a, b| a.2.total_cmp(&b.2))
	}

	pub fn iter(&self) -> std::vec::IntoIter<(IVec3, u32, G::Data<'_>)> {
		let mut out = Vec::new();
		for (oct, tree) in self.trees.iter().enumerate() {
			out.extend(tree.iter().map(|(origin, size, value)| (join_region_origin(oct, origin, size), size, value)));
		}
		out.into_iter()
	}
}

fn split_pos(pos: IVec3) -> (usize, UVec3) {
	let sx = (pos.x >= 0) as usize;
	let sy = (pos.y >= 0) as usize;
	let sz = (pos.z >= 0) as usize;
	let oct = sx | (sy << 1) | (sz << 2);
	let local = UVec3::new(axis_to_local(pos.x), axis_to_local(pos.y), axis_to_local(pos.z));
	(oct, local)
}

fn join_pos(oct: usize, pos: UVec3) -> IVec3 {
	IVec3::new(axis_from_local((oct & 1) != 0, pos.x), axis_from_local((oct & 2) != 0, pos.y), axis_from_local((oct & 4) != 0, pos.z))
}

fn join_region_origin(oct: usize, pos: UVec3, size: u32) -> IVec3 {
	let max = size.saturating_sub(1);
	IVec3::new(
		axis_from_local((oct & 1) != 0, if (oct & 1) != 0 { pos.x } else { pos.x + max }),
		axis_from_local((oct & 2) != 0, if (oct & 2) != 0 { pos.y } else { pos.y + max }),
		axis_from_local((oct & 4) != 0, if (oct & 4) != 0 { pos.z } else { pos.z + max }),
	)
}

fn world_to_local_point(oct: usize, point: Vec3) -> Vec3 {
	Vec3::new(
		if (oct & 1) != 0 { point.x } else { -point.x },
		if (oct & 2) != 0 { point.y } else { -point.y },
		if (oct & 4) != 0 { point.z } else { -point.z },
	)
}

fn world_to_local_vector(oct: usize, vector: Vec3) -> Vec3 {
	Vec3::new(
		if (oct & 1) != 0 { vector.x } else { -vector.x },
		if (oct & 2) != 0 { vector.y } else { -vector.y },
		if (oct & 4) != 0 { vector.z } else { -vector.z },
	)
}

fn local_to_world_normal(oct: usize, normal: I8Vec3) -> I8Vec3 {
	I8Vec3::new(
		if (oct & 1) != 0 { normal.x } else { -normal.x },
		if (oct & 2) != 0 { normal.y } else { -normal.y },
		if (oct & 4) != 0 { normal.z } else { -normal.z },
	)
}

#[inline]
fn axis_to_local(v: i32) -> u32 {
	if v >= 0 { v as u32 } else { (-v - 1) as u32 }
}

#[inline]
fn axis_from_local(non_negative: bool, v: u32) -> i32 {
	if non_negative { v as i32 } else { -(v as i32) - 1 }
}

fn split_axis(min: i32, end: i32) -> [(bool, u32, u32); 2] {
	let mut out = [(false, 0, 0), (false, 0, 0)];
	let mut n = 0;
	if min < 0 {
		let neg_end = end.min(0);
		if min < neg_end {
			out[n] = (false, (-neg_end) as u32, (-min) as u32);
			n += 1;
		}
	}
	if end > 0 {
		let pos_min = min.max(0);
		if pos_min < end {
			out[n] = (true, pos_min as u32, end as u32);
		}
	}
	out
}

fn split_region(region: NonZeroVoxelRegion) -> Vec<(usize, NonZeroVoxelRegion)> {
	let xs = split_axis(region.min().x, region.end().x);
	let ys = split_axis(region.min().y, region.end().y);
	let zs = split_axis(region.min().z, region.end().z);
	let mut out = Vec::new();
	for (x_sign, x0, x1) in xs.into_iter().filter(|(_, a, b)| a < b) {
		for (y_sign, y0, y1) in ys.into_iter().filter(|(_, a, b)| a < b) {
			for (z_sign, z0, z1) in zs.into_iter().filter(|(_, a, b)| a < b) {
				let oct = (x_sign as usize) | ((y_sign as usize) << 1) | ((z_sign as usize) << 2);
				out.push((oct, NonZeroVoxelRegion::from_min_end(IVec3::new(x0 as i32, y0 as i32, z0 as i32), IVec3::new(x1 as i32, y1 as i32, z1 as i32)).unwrap()));
			}
		}
	}
	out
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::voxel_grid_tree::PackedCell;
	use std::collections::HashMap;

	#[test]
	fn raycast_hits_negative_space() {
		let mut t = SignedGridTree::<PackedCell>::new();
		t.insert(IVec3::new(-1, 0, 0), 1);
		let tf = Transform::from_translation(Vec3::new(1.5, 0.5, 0.5)).looking_to(Vec3::NEG_X, Vec3::Y);
		let hit = t.raycast(&tf, Some(10.0)).map(|(pos, _, _)| pos);
		assert_eq!(hit, Some(IVec3::new(-1, 0, 0)));
	}

	#[test]
	fn iter_preserves_negative_voxel_coverage() {
		let mut t = SignedGridTree::<PackedCell>::new();
		t.add_area(NonZeroVoxelRegion::new(IVec3::new(-4, -4, -4), UVec3::splat(4)).unwrap(), 1);
		let mut seen = HashMap::new();
		for (origin, size, value) in t.iter() {
			for dx in 0..size as i32 {
				for dy in 0..size as i32 {
					for dz in 0..size as i32 {
						let pos = origin + IVec3::new(dx, dy, dz);
						assert!(seen.insert(pos, value).is_none(), "iter overlapped at {pos:?}");
					}
				}
			}
		}
		assert_eq!(seen.len(), 64);
		assert!(seen.keys().all(|pos| pos.cmpge(IVec3::new(-4, -4, -4)).all() && pos.cmplt(IVec3::ZERO).all()));
	}
}
