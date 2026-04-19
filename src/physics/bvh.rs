use std::{cmp::Ordering, collections::BinaryHeap, fmt::Debug};

use glam::{Vec3, Vec4};

use tracy_client::span;

use crate::{debug_draw, pose::Pose};

#[derive(Debug)]
pub enum BVHInternal {
	SubNodes {
		sub1: u16,
		sub2: u16,
	},
	Leaf {
		start: u16,
		count: u16,
	},
}

#[derive(Debug)]
pub struct BVHNode {
	pub min_corner: Vec3,
	pub max_corner: Vec3,
	pub sub_nodes: BVHInternal,
}

// SAH binning constants
//
// SAH (Surface Area Heuristic) produces significantly better trees than a
// plain spatial median split. The heuristic estimates the expected cost of
// a split by weighting each side's AABB surface area against the number of
// primitives it contains.
//
// BIN_COUNT=16 gives noticeably better split planes than 8 at modest extra
// build cost. The improvement matters most when primitives are non-uniform
// in size or distribution.
const BIN_COUNT: usize = 16;

// Cost of traversing one BVH node relative to testing one primitive.
const TRAVERSAL_COST: f32 = 1.0;
const INTERSECT_COST: f32 = 1.0;

fn surface_area(min_corner: Vec3, max_corner: Vec3) -> f32 {
	let extent = (max_corner - min_corner).max(Vec3::ZERO);
	2.0 * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x)
}

struct Bin {
	min_corner:      Vec3,
	max_corner:      Vec3,
	primitive_count: u32,
}

impl Bin {
	fn empty() -> Self {
		Bin {
			min_corner:      Vec3::splat(f32::INFINITY),
			max_corner:      Vec3::splat(f32::NEG_INFINITY),
			primitive_count: 0,
		}
	}

	fn extend(&mut self, item_min: Vec3, item_max: Vec3) {
		self.min_corner      = self.min_corner.min(item_min);
		self.max_corner      = self.max_corner.max(item_max);
		self.primitive_count += 1;
	}
}

impl BVHNode {
	fn build_range<Index>(items: &mut [(Index, (Vec3, Vec3))], nodes: &mut Vec<BVHNode>, start: u16, end: u16) -> Self {
		assert!(end > start);
		let slice = &mut items[start as usize..end as usize];

		// Compute the bounding box of all primitives and of their centroids.
		let mut bounds_min   = slice[0].1.0;
		let mut bounds_max   = slice[0].1.1;
		let mut centroid_min = (slice[0].1.0 + slice[0].1.1) * 0.5;
		let mut centroid_max = centroid_min;

		for item in slice.iter() {
			bounds_min   = bounds_min.min(item.1.0);
			bounds_max   = bounds_max.max(item.1.1);
			let centroid = (item.1.0 + item.1.1) * 0.5;
			centroid_min = centroid_min.min(centroid);
			centroid_max = centroid_max.max(centroid);
		}

		let count = slice.len() as u16;

		// Leaf conditions:
		//   ≤ 4 primitives  -> forced leaf (each item is an expensive DDA call on the GPU;
		//                      small leaves mean we bail out after fewer wasted traversals)
		//   coincident centroids -> splitting cannot separate anything
		if count <= 4 || centroid_min == centroid_max {
			return Self {
				min_corner: bounds_min,
				max_corner: bounds_max,
				sub_nodes:  BVHInternal::Leaf { start, count },
			};
		}

		// SAH binning split
		//
		// For each axis, distribute primitives into BIN_COUNT uniform bins along
		// that axis by centroid position. Sweep left->right and right->left to
		// build prefix bounds and counts, then evaluate the SAH cost at every
		// bin boundary on every axis. The globally cheapest split wins.
		//
		// SAH cost = TRAVERSAL_COST
		//          + (left_count  * SA(left))  / SA(parent) * INTERSECT_COST
		//          + (right_count * SA(right)) / SA(parent) * INTERSECT_COST

		let parent_surface_area = surface_area(bounds_min, bounds_max);
		let no_split_cost       = count as f32 * INTERSECT_COST;

		let mut best_cost = f32::INFINITY;
		let mut best_axis = 0usize;
		let mut best_bin  = 0usize;

		for axis in 0..3 {
			let axis_extent = centroid_max[axis] - centroid_min[axis];
			if axis_extent <= 0.0 { continue; }

			let mut bins = [(); BIN_COUNT].map(|_| Bin::empty());

			for item in slice.iter() {
				let centroid   = (item.1.0 + item.1.1) * 0.5;
				let normalized = (centroid[axis] - centroid_min[axis]) / axis_extent;
				let bin_index  = ((normalized * BIN_COUNT as f32) as usize).min(BIN_COUNT - 1);
				bins[bin_index].extend(item.1.0, item.1.1);
			}

			// Left-to-right prefix
			let mut left_min    = [Vec3::splat(f32::INFINITY);     BIN_COUNT];
			let mut left_max    = [Vec3::splat(f32::NEG_INFINITY); BIN_COUNT];
			let mut left_counts = [0u32; BIN_COUNT];
			{
				let mut rmin = Vec3::splat(f32::INFINITY);
				let mut rmax = Vec3::splat(f32::NEG_INFINITY);
				let mut cnt  = 0u32;
				for b in 0..BIN_COUNT {
					let bin = &bins[b];
					if bin.primitive_count > 0 {
						rmin = rmin.min(bin.min_corner);
						rmax = rmax.max(bin.max_corner);
						cnt += bin.primitive_count;
					}
					left_min[b]    = rmin;
					left_max[b]    = rmax;
					left_counts[b] = cnt;
				}
			}

			// Right-to-left prefix
			let mut right_min    = [Vec3::splat(f32::INFINITY);     BIN_COUNT];
			let mut right_max    = [Vec3::splat(f32::NEG_INFINITY); BIN_COUNT];
			let mut right_counts = [0u32; BIN_COUNT];
			{
				let mut rmin = Vec3::splat(f32::INFINITY);
				let mut rmax = Vec3::splat(f32::NEG_INFINITY);
				let mut cnt  = 0u32;
				for b in (0..BIN_COUNT).rev() {
					let bin = &bins[b];
					if bin.primitive_count > 0 {
						rmin = rmin.min(bin.min_corner);
						rmax = rmax.max(bin.max_corner);
						cnt += bin.primitive_count;
					}
					right_min[b]    = rmin;
					right_max[b]    = rmax;
					right_counts[b] = cnt;
				}
			}

			// Evaluate split planes
			for split_bin in 0..(BIN_COUNT - 1) {
				let left_count  = left_counts[split_bin];
				let right_count = right_counts[split_bin + 1];
				if left_count == 0 || right_count == 0 { continue; }

				let left_sa  = surface_area(left_min[split_bin],      left_max[split_bin]);
				let right_sa = surface_area(right_min[split_bin + 1], right_max[split_bin + 1]);

				let cost = TRAVERSAL_COST
					+ (left_count  as f32 * left_sa  / parent_surface_area) * INTERSECT_COST
					+ (right_count as f32 * right_sa / parent_surface_area) * INTERSECT_COST;

				if cost < best_cost {
					best_cost = cost;
					best_axis = axis;
					best_bin  = split_bin;
				}
			}
		}

		// If no split beats a leaf, make a leaf.
		if best_cost >= no_split_cost {
			return Self {
				min_corner: bounds_min,
				max_corner: bounds_max,
				sub_nodes:  BVHInternal::Leaf { start, count },
			};
		}

		// In-place partition around the winning split plane.
		let axis_extent = centroid_max[best_axis] - centroid_min[best_axis];

		let split_index = {
			let mut left  = 0usize;
			let mut right = slice.len() - 1;

			loop {
				while left <= right {
					let centroid   = (slice[left].1.0 + slice[left].1.1) * 0.5;
					let normalized = (centroid[best_axis] - centroid_min[best_axis]) / axis_extent;
					let bin_index  = ((normalized * BIN_COUNT as f32) as usize).min(BIN_COUNT - 1);
					if bin_index <= best_bin { left += 1; } else { break; }
				}
				while right > left {
					let centroid   = (slice[right].1.0 + slice[right].1.1) * 0.5;
					let normalized = (centroid[best_axis] - centroid_min[best_axis]) / axis_extent;
					let bin_index  = ((normalized * BIN_COUNT as f32) as usize).min(BIN_COUNT - 1);
					if bin_index > best_bin { if right == 0 { break; } right -= 1; } else { break; }
				}
				if left >= right { break; }
				slice.swap(left, right);
				left += 1;
				if right > 0 { right -= 1; }
			}

			// Guard against degenerate cases where floating-point binning
			// pushes everything to one side.
			left.max(1).min(slice.len() - 1) as u16
		};

		nodes.push(BVHNode {
			min_corner: Vec3::ZERO,
			max_corner: Vec3::ZERO,
			sub_nodes:  BVHInternal::Leaf { start: 0, count: 0 },
		});
		let sub1_index = nodes.len() - 1;
		nodes[sub1_index] = Self::build_range(items, nodes, start, start + split_index);

		nodes.push(BVHNode {
			min_corner: Vec3::ZERO,
			max_corner: Vec3::ZERO,
			sub_nodes:  BVHInternal::Leaf { start: 0, count: 0 },
		});
		let sub2_index = nodes.len() - 1;
		nodes[sub2_index] = Self::build_range(items, nodes, start + split_index, end);

		Self {
			min_corner: bounds_min,
			max_corner: bounds_max,
			sub_nodes:  BVHInternal::SubNodes {
				sub1: sub1_index as u16,
				sub2: sub2_index as u16,
			},
		}
	}
}

#[derive(Debug)]
pub struct BVH<Index: Copy + Debug + PartialEq> {
	nodes: Vec<BVHNode>,
	items: Vec<(Index, (Vec3, Vec3))>,
}

impl<Index: Copy + Debug + PartialEq> BVH<Index> {
	pub fn new(mut items: Vec<(Index, (Vec3, Vec3))>) -> Self {
		let _zone = span!("BVH creation");
		let item_count = items.len() as u16;
		let mut nodes: Vec<BVHNode> = vec![];
		nodes.push(BVHNode {
			min_corner: Vec3::ZERO,
			max_corner: Vec3::ZERO,
			sub_nodes:  BVHInternal::Leaf { start: 0, count: 0 },
		});
		if item_count != 0 {
			nodes[0] = BVHNode::build_range(&mut items, &mut nodes, 0, item_count);
		}
		BVH { nodes, items }
	}

	fn intersects(aabb_a: &(Vec3, Vec3), aabb_b: &(Vec3, Vec3)) -> bool {
		aabb_a.0.cmple(aabb_b.1).all() && aabb_a.1.cmpge(aabb_b.0).all()
	}

	pub fn get_collisions(&self, bounds: &(Vec3, Vec3)) -> Vec<Index> {
		let _zone = span!("BVH get collisions");
		let mut out: Vec<Index> = vec![];
		let mut stack = vec![0u16];

		while let Some(node_index) = stack.pop() {
			let node = &self.nodes[node_index as usize];

			if !Self::intersects(bounds, &(node.min_corner, node.max_corner)) { continue; }

			match node.sub_nodes {
				BVHInternal::SubNodes { sub1, sub2 } => {
					stack.push(sub1);
					stack.push(sub2);
				}
				BVHInternal::Leaf { start, count } => {
					out.extend(
						self.items[start as usize..(start + count) as usize]
							.iter()
							.filter_map(|item|
								if Self::intersects(bounds, &item.1) { Some(item.0) } else { None }
							)
					);
				}
			}
		}

		out
	}

	fn ray_aabb_intersection(start: &Vec3, direction: &Vec3, aabb: &(Vec3, Vec3)) -> Option<f32> {
		let (min, max) = aabb;

		if start.cmpge(*min).all() && start.cmple(*max).all() {
			return Some(0.0);
		}

		let inv = Vec3::ONE / *direction;
		let t1  = (*min - *start) * inv;
		let t2  = (*max - *start) * inv;

		let tmin = t1.min(t2).max_element();
		let tmax = t1.max(t2).min_element();

		if tmax < 0.0 || tmin > tmax { return None; }

		Some(tmin)
	}

	pub fn raycast(&'_ self, pose: &Pose, max_length: Option<f32>) -> BVHRaycastIterator<'_, Index> {
		let start     = pose.translation;
		let direction = pose.rotation * Vec3::Z;
		let mut heap  = BinaryHeap::new();
		let root      = &self.nodes[0];
		if let Some(length) = Self::ray_aabb_intersection(&start, &direction, &(root.min_corner, root.max_corner)) {
			if max_length.is_none() || length <= max_length.unwrap() {
				heap.push(Candidate { length, entry: BVHEntry::Node(0) });
			}
		}
		BVHRaycastIterator { bvh: self, start, direction, max_length, heap }
	}

	pub fn render_debug(&self) {
		let mut stack = vec![0];

		while let Some(idx) = stack.pop() {
			let node = &self.nodes[idx as usize];
			match node.sub_nodes {
				BVHInternal::SubNodes { sub1, sub2 } => {
					debug_draw::aabb(node.min_corner, node.max_corner, &Vec4::ONE);
					stack.push(sub1);
					stack.push(sub2);
				}
				BVHInternal::Leaf { start, count } => {
					for item in self.items[start as usize..(start + count) as usize].iter() {
						debug_draw::aabb(item.1.0, item.1.1, &Vec4::W);
					}
				}
			}
		}
	}

	pub fn get_internals(&self) -> (&Vec<BVHNode>, &Vec<(Index, (Vec3, Vec3))>) {
		(&self.nodes, &self.items)
	}
}

#[derive(PartialEq)]
enum BVHEntry<Index> {
	Node(u16),
	Hit(Index),
}

#[derive(PartialEq)]
struct Candidate<Index> {
	length: f32,
	entry:  BVHEntry<Index>,
}

impl<Index: PartialEq> Eq for Candidate<Index> {}
impl<Index: PartialEq> PartialOrd for Candidate<Index> {
	fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) }
}
impl<Index: PartialEq> Ord for Candidate<Index> {
	fn cmp(&self, other: &Self) -> Ordering {
		other.length.partial_cmp(&self.length).unwrap_or(Ordering::Equal)
	}
}

pub struct BVHRaycastIterator<'a, Index: Copy + Debug + PartialEq> {
	bvh:        &'a BVH<Index>,
	start:      Vec3,
	direction:  Vec3,
	max_length: Option<f32>,
	heap:       BinaryHeap<Candidate<Index>>,
}

impl<'a, Index: Copy + Debug + PartialEq + PartialEq> Iterator for BVHRaycastIterator<'a, Index> {
	type Item = (Index, f32);

	fn next(&mut self) -> Option<Self::Item> {
		loop {
			let Candidate { length, entry } = self.heap.pop()?;
			assert!(self.max_length.is_none() || length <= self.max_length.unwrap());
			match entry {
				BVHEntry::Hit(index) => return Some((index, length)),
				BVHEntry::Node(idx) => {
					let node = &self.bvh.nodes[idx as usize];
					match node.sub_nodes {
						BVHInternal::SubNodes { sub1, sub2 } => {
							for child in [sub1, sub2] {
								let cn = &self.bvh.nodes[child as usize];
								if let Some(length) = BVH::<Index>::ray_aabb_intersection(&self.start, &self.direction, &(cn.min_corner, cn.max_corner)) {
									if self.max_length.is_none() || length <= self.max_length.unwrap() {
										self.heap.push(Candidate { length, entry: BVHEntry::Node(child) });
									}
								}
							}
						}
						BVHInternal::Leaf { start, count } => {
							for item in &self.bvh.items[start as usize..(start + count) as usize] {
								if let Some(length) = BVH::<Index>::ray_aabb_intersection(&self.start, &self.direction, &item.1) {
									if self.max_length.is_none() || length <= self.max_length.unwrap() {
										self.heap.push(Candidate { length, entry: BVHEntry::Hit(item.0) });
									}
								}
							}
						}
					}
				}
			}
		}
	}
}
