use std::{cmp::Ordering, collections::BinaryHeap, fmt::Debug, sync::Arc};

use bevy::math::Vec3;
use bevy::tasks::{ComputeTaskPool, TaskPoolBuilder};
use bevy::transform::components::Transform;

use tracy_client::span;

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

/// SAH binning constants
///
/// SAH (Surface Area Heuristic) produces significantly better trees than a
/// plain spatial median split. The heuristic estimates the expected cost of
/// a split by weighting each side's AABB surface area against the number of
/// primitives it contains.
///
/// BIN_COUNT=16 gives noticeably better split planes than 8 at modest extra
/// build cost. The improvement matters most when primitives are non-uniform
/// in size or distribution.
const BIN_COUNT: usize = 16;
const PARALLEL_BUILD_THRESHOLD: usize = 512;

/// Cost of traversing one BVH node relative to testing one primitive.
const TRAVERSAL_COST: f32 = 1.0;
const INTERSECT_COST: f32 = 1.0;

fn surface_area(min_corner: Vec3, max_corner: Vec3) -> f32 {
	let extent = (max_corner - min_corner).max(Vec3::ZERO);
	2.0 * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x)
}

#[derive(Clone, Copy)]
struct BuildPrimitive<Index> {
	index: Index,
	min_corner: Vec3,
	max_corner: Vec3,
}

struct BuildBuffer<Index> {
	nodes: Vec<BVHNode>,
	items: Vec<(Index, (Vec3, Vec3))>,
}

impl<Index> BuildBuffer<Index> {
	fn with_capacity(primitive_count: usize) -> Self {
		Self {
			nodes: Vec::with_capacity(primitive_count.saturating_mul(2).saturating_sub(1)),
			items: Vec::with_capacity(primitive_count),
		}
	}
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

fn rebase_subtree<Index>(subtree: &mut BuildBuffer<Index>, node_base: u16, item_base: u16) {
	for node in &mut subtree.nodes {
		match &mut node.sub_nodes {
			BVHInternal::SubNodes { sub1, sub2 } => {
				*sub1 += node_base;
				*sub2 += node_base;
			}
			BVHInternal::Leaf { start, .. } => {
				*start += item_base;
			}
		}
	}
}

impl BVHNode {
	fn build_subtree<Index: Copy + Send + Sync + 'static>(
		primitives: Arc<[BuildPrimitive<Index>]>,
		indices: &mut [u16],
		output: &mut BuildBuffer<Index>,
	) -> u16 {
		assert!(!indices.is_empty());

		let first = primitives[indices[0] as usize];
		let mut bounds_min = first.min_corner;
		let mut bounds_max = first.max_corner;
		let mut centroid_min = (first.min_corner + first.max_corner) * 0.5;
		let mut centroid_max = centroid_min;

		for &primitive_index in indices.iter() {
			let primitive = primitives[primitive_index as usize];
			bounds_min = bounds_min.min(primitive.min_corner);
			bounds_max = bounds_max.max(primitive.max_corner);
			let centroid = (primitive.min_corner + primitive.max_corner) * 0.5;
			centroid_min = centroid_min.min(centroid);
			centroid_max = centroid_max.max(centroid);
		}

		let count = indices.len() as u16;
		if count <= 4 || centroid_min == centroid_max {
			let start = output.items.len() as u16;
			output.items.extend(indices.iter().map(|&primitive_index| {
				let primitive = primitives[primitive_index as usize];
				(primitive.index, (primitive.min_corner, primitive.max_corner))
			}));
			let node_index = output.nodes.len() as u16;
			output.nodes.push(Self {
				min_corner: bounds_min,
				max_corner: bounds_max,
				sub_nodes: BVHInternal::Leaf { start, count },
			});
			return node_index;
		}

		let parent_surface_area_inv = surface_area(bounds_min, bounds_max).recip();
		let no_split_cost = count as f32 * INTERSECT_COST;
		let mut best_cost = f32::INFINITY;
		let mut best_axis = 0usize;
		let mut best_bin = 0usize;

		let mut axis_extent_inv = [0.0; 3];
		let mut axis_active = [false; 3];
		for axis in 0..3 {
			let axis_extent = centroid_max[axis] - centroid_min[axis];
			if axis_extent > 0.0 {
				axis_extent_inv[axis] = 1.0 / axis_extent;
				axis_active[axis] = true;
			}
		}

		let mut bins_by_axis: [[Bin; BIN_COUNT]; 3] = std::array::from_fn(|_| std::array::from_fn(|_| Bin::empty()));
		for &primitive_index in indices.iter() {
			let primitive = primitives[primitive_index as usize];
			let centroid = (primitive.min_corner + primitive.max_corner) * 0.5;
			for axis in 0..3 {
				if !axis_active[axis] {
					continue;
				}
				let normalized = (centroid[axis] - centroid_min[axis]) * axis_extent_inv[axis];
				let bin_index = ((normalized * BIN_COUNT as f32) as usize).min(BIN_COUNT - 1);
				bins_by_axis[axis][bin_index].extend(primitive.min_corner, primitive.max_corner);
			}
		}

		for axis in 0..3 {
			if !axis_active[axis] {
				continue;
			}
			let bins = &bins_by_axis[axis];
			let mut left_costs = [f32::INFINITY; BIN_COUNT - 1];
			{
				let mut rmin = Vec3::splat(f32::INFINITY);
				let mut rmax = Vec3::splat(f32::NEG_INFINITY);
				let mut cnt = 0u32;
				for split_bin in 0..(BIN_COUNT - 1) {
					let bin = &bins[split_bin];
					if bin.primitive_count > 0 {
						rmin = rmin.min(bin.min_corner);
						rmax = rmax.max(bin.max_corner);
						cnt += bin.primitive_count;
					}
					if cnt > 0 {
						left_costs[split_bin] = cnt as f32 * surface_area(rmin, rmax);
					}
				}
			}

			let mut right_costs = [f32::INFINITY; BIN_COUNT - 1];
			{
				let mut rmin = Vec3::splat(f32::INFINITY);
				let mut rmax = Vec3::splat(f32::NEG_INFINITY);
				let mut cnt = 0u32;
				for split_bin in (0..(BIN_COUNT - 1)).rev() {
					let bin = &bins[split_bin + 1];
					if bin.primitive_count > 0 {
						rmin = rmin.min(bin.min_corner);
						rmax = rmax.max(bin.max_corner);
						cnt += bin.primitive_count;
					}
					if cnt > 0 {
						right_costs[split_bin] = cnt as f32 * surface_area(rmin, rmax);
					}
				}
			}

			for split_bin in 0..(BIN_COUNT - 1) {
				let cost = TRAVERSAL_COST
					+ (left_costs[split_bin] + right_costs[split_bin]) * parent_surface_area_inv * INTERSECT_COST;
				if cost < best_cost {
					best_cost = cost;
					best_axis = axis;
					best_bin = split_bin;
				}
			}
		}

		if best_cost >= no_split_cost {
			let start = output.items.len() as u16;
			output.items.extend(indices.iter().map(|&primitive_index| {
				let primitive = primitives[primitive_index as usize];
				(primitive.index, (primitive.min_corner, primitive.max_corner))
			}));
			let node_index = output.nodes.len() as u16;
			output.nodes.push(Self {
				min_corner: bounds_min,
				max_corner: bounds_max,
				sub_nodes: BVHInternal::Leaf { start, count },
			});
			return node_index;
		}

		let axis_extent = centroid_max[best_axis] - centroid_min[best_axis];
		let split_position = centroid_min[best_axis] + axis_extent * ((best_bin + 1) as f32 / BIN_COUNT as f32);
		let split_index = {
			let mut left = 0usize;
			let mut right = indices.len() - 1;
			loop {
				while left <= right {
					let primitive = primitives[indices[left] as usize];
					let centroid = (primitive.min_corner[best_axis] + primitive.max_corner[best_axis]) * 0.5;
					if centroid < split_position {
						left += 1;
					} else {
						break;
					}
				}
				while right > left {
					let primitive = primitives[indices[right] as usize];
					let centroid = (primitive.min_corner[best_axis] + primitive.max_corner[best_axis]) * 0.5;
					if centroid >= split_position {
						if right == 0 {
							break;
						}
						right -= 1;
					} else {
						break;
					}
				}
				if left >= right {
					break;
				}
				indices.swap(left, right);
				left += 1;
				right = right.saturating_sub(1);
			}
			left.max(1).min(indices.len() - 1)
		};

		let node_index = output.nodes.len() as u16;
		output.nodes.push(Self {
			min_corner: bounds_min,
			max_corner: bounds_max,
			sub_nodes: BVHInternal::SubNodes { sub1: 0, sub2: 0 },
		});

		let should_parallelize = indices.len() >= PARALLEL_BUILD_THRESHOLD;
		let (left_indices, right_indices) = indices.split_at_mut(split_index);
		let (left_root, right_root) = if should_parallelize {
			let (spawned_indices, local_indices, spawned_is_left) = if left_indices.len() < right_indices.len() {
				(left_indices.to_vec(), right_indices, true)
			} else {
				(right_indices.to_vec(), left_indices, false)
			};
			let pool = ComputeTaskPool::get_or_init(|| TaskPoolBuilder::new().build());
			let mut local_root = 0u16;
			let mut results: Vec<(u16, BuildBuffer<Index>)> = pool.scope(|scope| {
				let spawned_primitives = primitives.clone();
				scope.spawn(async move {
					let mut indices = spawned_indices;
					let mut output = BuildBuffer::with_capacity(indices.len());
					let root = Self::build_subtree(spawned_primitives, &mut indices, &mut output);
					(root, output)
				});
				local_root = Self::build_subtree(primitives.clone(), local_indices, output);
			});
			let (spawned_local_root, mut spawned_tree) = results.remove(0);
			let spawned_node_base = output.nodes.len() as u16;
			let spawned_item_base = output.items.len() as u16;
			rebase_subtree(&mut spawned_tree, spawned_node_base, spawned_item_base);
			let spawned_root = spawned_local_root + spawned_node_base;
			output.nodes.extend(spawned_tree.nodes);
			output.items.extend(spawned_tree.items);

			if spawned_is_left {
				(spawned_root, local_root)
			} else {
				(local_root, spawned_root)
			}
		} else {
			let left_root = Self::build_subtree(primitives.clone(), left_indices, output);
			let right_root = Self::build_subtree(primitives.clone(), right_indices, output);
			(left_root, right_root)
		};

		output.nodes[node_index as usize].sub_nodes = BVHInternal::SubNodes { sub1: left_root, sub2: right_root };
		node_index
	}
}

#[derive(Debug)]
pub struct BVH<Index: Copy + Debug + PartialEq> {
	nodes: Vec<BVHNode>,
	items: Vec<(Index, (Vec3, Vec3))>,
}

impl<Index: Copy + Debug + PartialEq> BVH<Index> {
	pub fn new(items: Vec<(Index, (Vec3, Vec3))>) -> Self
	where
		Index: Send + Sync + 'static,
	{
		let _zone = span!("BVH creation");
		let item_count = items.len() as u16;
		let primitives: Arc<[BuildPrimitive<Index>]> = items
			.into_iter()
			.map(|(index, (min_corner, max_corner))| BuildPrimitive {
				index,
				min_corner,
				max_corner,
			})
			.collect::<Vec<_>>()
			.into();
		if item_count == 0 {
			return BVH { nodes: vec![BVHNode {
				min_corner: Vec3::ZERO,
				max_corner: Vec3::ZERO,
				sub_nodes: BVHInternal::Leaf { start: 0, count: 0 },
			}], items: Vec::new() };
		}

		let mut indices: Vec<u16> = (0..item_count).collect();
		let mut output = BuildBuffer::with_capacity(item_count as usize);
		BVHNode::build_subtree(primitives, &mut indices, &mut output);
		BVH { nodes: output.nodes, items: output.items }
	}

	fn intersects(aabb_a: &(Vec3, Vec3), aabb_b: &(Vec3, Vec3)) -> bool {
		aabb_a.0.cmple(aabb_b.1).all() && aabb_a.1.cmpge(aabb_b.0).all()
	}

	pub fn collisions(&self, bounds: &(Vec3, Vec3)) -> Vec<Index> {
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

	pub fn raycast(&'_ self, transform: &Transform, max_length: Option<f32>) -> BVHRaycastIterator<'_, Index> {
		let start     = transform.translation;
		let direction = transform.rotation * Vec3::Z;
		let mut heap  = BinaryHeap::new();
		let root      = &self.nodes[0];
		if let Some(length) = Self::ray_aabb_intersection(&start, &direction, &(root.min_corner, root.max_corner)) {
			if max_length.is_none() || length <= max_length.unwrap() {
				heap.push(Candidate { length, entry: BVHEntry::Node(0) });
			}
		}
		BVHRaycastIterator { bvh: self, start, direction, max_length, heap }
	}

	pub fn internals(&self) -> (&Vec<BVHNode>, &Vec<(Index, (Vec3, Vec3))>) {
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

impl<'a, Index: Copy + Debug + PartialEq> Iterator for BVHRaycastIterator<'a, Index> {
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
