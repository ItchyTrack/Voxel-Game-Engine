use super::{GridTree64, GridType, LOG_SIZE};
use crate::views::{GridTreeView, NodeRef, CellKind};
use bevy::math::{I8Vec3, UVec3};
use voxel_math::{Fixed, FixedVec3, Transform, ray_aabb_intersection};

impl<G: GridType> GridTreeView for GridTree64<G> {
	type NodeHandle = u32;
	type Data<'d> = G::Data<'d> where Self: 'd;
	const BRANCH_LOG2: u8 = LOG_SIZE;

	fn is_empty(&self) -> bool { self.raw.is_empty() }
	fn root_depth(&self) -> u8 { self.raw.root_depth() }
	fn root_pos(&self) -> UVec3 { self.raw.root_pos() }
	fn root(&self) -> NodeRef<u32> {
		NodeRef { handle: 0, depth: self.raw.root_depth(), origin: self.raw.root_pos() }
	}
	fn cell_kind(&self, node: u32, i: u8) -> CellKind { self.raw.cell_kind(node, i) }
	fn child_handle(&self, node: u32, i: u8) -> u32 { self.raw.child_index(node, i) }
	fn cell_data<'tree>(&'tree self, node: u32, i: u8) -> G::Data<'tree> {
		self.grid_type.read_data(self.raw.cell_bytes(node, i))
	}
	fn occupancy_mask(&self, node: u32) -> u64 { self.raw.data_mask(node) | self.raw.node_mask(node) }

	fn raycast(&self, transform: &(impl Transform + ?Sized), max_length: Option<Fixed>) -> Option<(UVec3, I8Vec3, Fixed)> {
		if self.is_empty() { return None; }
		let max_length = max_length.unwrap_or(Fixed::MAX);
		let origin = transform.translation();
		let dir = transform.direction();
		if dir == FixedVec3::ZERO || max_length < Fixed::ZERO { return None; }
		let root_pos = self.root_pos();
		let root_depth = self.root_depth();
		let root_size = Self::size(root_depth);
		let root_min = FixedVec3::from(root_pos);
		let root_max = root_min + FixedVec3::splat(Fixed::from_num(root_size));
		let mut last_distance = ray_aabb_intersection(&origin, &dir, &(root_min, root_max))?;
		if last_distance > max_length { return None; }
		let step = dir.signum().as_i8vec3();
		let mut entry = None;
		for axis in 0..3 {
			if step[axis] == 0 { continue; }
			let boundary = if step[axis] > 0 { root_min[axis] } else { root_max[axis] };
			let ratio = plane_ratio(boundary, origin[axis], dir[axis]);
			if ratio.0 >= 0 && entry.is_none_or(|(_, previous)| ratio_cmp(ratio, previous).is_gt()) {
				entry = Some((axis, ratio));
			}
		}
		let mut last_axis = entry.map_or(dir.abs().max_position(), |(axis, _)| axis);
		let mut grid = [0i64; 3];
		for axis in 0..3 {
			let (n, d) = if let Some((_, ratio)) = entry {
				coordinate_ratio(origin[axis], dir[axis], ratio)
			} else { (origin[axis].to_bits() as i128, 1) };
			// At an integer boundary, start in the voxel ahead of the ray.
			let denominator = d.checked_mul(Fixed::ONE.to_bits() as i128).expect("ray coordinate overflow");
			let coordinate = if step[axis] < 0 { div_ceil(n, denominator) - 1 } else { n.div_euclid(denominator) };
			grid[axis] = i64::try_from(coordinate - root_pos[axis] as i128).ok()?;
			if grid[axis] < 0 || grid[axis] >= root_size as i64 { return None; }
		}
		let mut current_node_index = 0u32;
		let mut current_depth = root_depth;
		loop {
			let pos = UVec3::new(grid[0] as u32, grid[1] as u32, grid[2] as u32);
			let node_relative = pos % Self::size(current_depth);
			let cell_size = Self::child_size(current_depth);
			let contents_index = Self::child_index_of((node_relative / cell_size).as_u8vec3());
			match self.raw.cell_kind(current_node_index, contents_index) {
				CellKind::Data => return Some((pos + root_pos, -step[last_axis] * I8Vec3::AXES[last_axis], last_distance)),
				CellKind::Node => {
					current_depth -= 1;
					current_node_index = self.raw.child_index(current_node_index, contents_index);
				}
				CellKind::Empty => {
					let mut exit = None;
					for axis in 0..3 {
						if step[axis] == 0 { continue; }
						let cell_min = grid[axis] / cell_size as i64 * cell_size as i64;
						let boundary = cell_min + if step[axis] > 0 { cell_size as i64 } else { 0 };
						let ratio = plane_ratio(Fixed::from_num(boundary + root_pos[axis] as i64), origin[axis], dir[axis]);
						if exit.is_none_or(|(_, previous, _)| ratio_cmp(ratio, previous).is_lt()) {
							exit = Some((axis, ratio, boundary));
						}
					}
					let (axis, ratio, boundary) = exit?;
					last_distance = ratio_distance(ratio)?;
					if last_distance > max_length { return None; }
					let old_pos = pos;
					for other in 0..3 {
						if other == axis {
							grid[other] = boundary - i64::from(step[other] < 0);
						} else if step[other] != 0 {
							let (n, d) = coordinate_ratio(origin[other], dir[other], ratio);
							let denominator = d.checked_mul(Fixed::ONE.to_bits() as i128).expect("ray coordinate overflow");
							// Tied crossings visit X, then Y, then Z, without skipping touched cells.
							let coordinate = if step[other] > 0 { div_ceil(n, denominator) - 1 } else { n.div_euclid(denominator) };
							let next = i64::try_from(coordinate - root_pos[other] as i128).ok()?;
							grid[other] = if step[other] > 0 { grid[other].max(next) } else { grid[other].min(next) };
						}
						if grid[other] < 0 || grid[other] >= root_size as i64 { return None; }
					}
					let next = UVec3::new(grid[0] as u32, grid[1] as u32, grid[2] as u32);
					while old_pos / Self::size(current_depth) != next / Self::size(current_depth) {
						let parent_offset = self.raw.parent_offset(current_node_index);
						if parent_offset == 0 { return None; }
						current_depth += 1;
						current_node_index -= parent_offset;
					}
					last_axis = axis;
				}
			}
		}
	}
}

fn plane_ratio(plane: Fixed, origin: Fixed, direction: Fixed) -> (i128, i128) {
	let numerator = plane.to_bits() as i128 - origin.to_bits() as i128;
	let denominator = direction.to_bits() as i128;
	if denominator < 0 { (-numerator, -denominator) } else { (numerator, denominator) }
}

fn ratio_cmp(a: (i128, i128), b: (i128, i128)) -> std::cmp::Ordering {
	a.0.checked_mul(b.1).expect("ray comparison overflow").cmp(&b.0.checked_mul(a.1).expect("ray comparison overflow"))
}

fn coordinate_ratio(origin: Fixed, direction: Fixed, time: (i128, i128)) -> (i128, i128) {
	let numerator = (origin.to_bits() as i128).checked_mul(time.1)
		.and_then(|n| (direction.to_bits() as i128).checked_mul(time.0).and_then(|offset| n.checked_add(offset)))
		.expect("ray coordinate overflow");
	(numerator, time.1)
}

fn div_ceil(n: i128, d: i128) -> i128 { n.div_euclid(d) + i128::from(n.rem_euclid(d) != 0) }

fn ratio_distance((n, d): (i128, i128)) -> Option<Fixed> {
	let numerator = n.checked_mul(Fixed::ONE.to_bits() as i128)?;
	let q = numerator / d;
	let r = numerator % d;
	let rounded = q + i128::from(r * 2 > d || (r * 2 == d && q & 1 != 0));
	Some(Fixed::from_bits(i64::try_from(rounded).ok()?))
}
