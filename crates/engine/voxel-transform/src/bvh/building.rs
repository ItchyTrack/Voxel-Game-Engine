use std::sync::Arc;

use voxel_math::{Fixed, FixedVec3};
use bevy::tasks::{ComputeTaskPool, TaskPoolBuilder};
use tracy_client::span;

use super::{BVH, BVHInternal, BVHNode};

/// Surface-area heuristic bins for each split axis.
const BIN_COUNT: usize = 16;
const PARALLEL_BUILD_THRESHOLD: usize = 512;
const TRAVERSAL_COST: Fixed = Fixed::ONE;
const INTERSECT_COST: Fixed = Fixed::ONE;

fn midpoint(min: FixedVec3, max: FixedVec3) -> FixedVec3 {
	FixedVec3::from_array(std::array::from_fn(|axis| {
		let sum = min[axis].to_bits() as i128 + max[axis].to_bits() as i128;
		let half = sum / 2;
		let rounded = half + if sum % 2 != 0 && half & 1 != 0 { sum.signum() } else { 0 };
		Fixed::from_bits(i64::try_from(rounded).expect("BVH midpoint overflow"))
	}))
}

// Normalize before squaring so large world bounds do not overflow the heuristic.
fn surface_area(min: FixedVec3, max: FixedVec3, unit: Fixed) -> Fixed {
	let extent = (max - min).max(FixedVec3::ZERO) / unit;
	Fixed::from_num(2) * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x)
}

fn bin_index(centroid: Fixed, min: Fixed, extent: Fixed) -> usize {
	let normalized = (centroid - min) / extent;
	(normalized * Fixed::from_num(BIN_COUNT)).floor().to_num::<usize>().min(BIN_COUNT - 1)
}

fn index(value: usize) -> u16 { u16::try_from(value).expect("BVH exceeds u16 index capacity") }

#[derive(Clone, Copy)]
struct BuildPrimitive<Index> {
	index: Index,
	min_corner: FixedVec3,
	max_corner: FixedVec3,
}

struct BuildBuffer<Index> {
	nodes: Vec<BVHNode>,
	items: Vec<(Index, (FixedVec3, FixedVec3))>,
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
	min_corner: FixedVec3,
	max_corner: FixedVec3,
	primitive_count: u32,
}

impl Bin {
	fn empty() -> Self {
		Self { min_corner: FixedVec3::MAX, max_corner: FixedVec3::MIN, primitive_count: 0 }
	}

	fn extend(&mut self, item_min: FixedVec3, item_max: FixedVec3) {
		self.min_corner = self.min_corner.min(item_min);
		self.max_corner = self.max_corner.max(item_max);
		self.primitive_count += 1;
	}
}

fn rebase_subtree<Index>(subtree: &mut BuildBuffer<Index>, node_base: u16, item_base: u16) {
	for node in &mut subtree.nodes {
		match &mut node.sub_nodes {
			BVHInternal::SubNodes { sub1, sub2 } => {
				*sub1 = sub1.checked_add(node_base).expect("BVH node index overflow");
				*sub2 = sub2.checked_add(node_base).expect("BVH node index overflow");
			}
			BVHInternal::Leaf { start, .. } => {
				*start = start.checked_add(item_base).expect("BVH item index overflow");
			}
		}
	}
}

impl BVHNode {
	fn build_leaf<Index: Copy>(
		primitives: &[BuildPrimitive<Index>], indices: &[u16], output: &mut BuildBuffer<Index>, min: FixedVec3, max: FixedVec3,
	) -> u16 {
		let start = index(output.items.len());
		output.items.extend(indices.iter().map(|&primitive_index| {
			let primitive = primitives[primitive_index as usize];
			(primitive.index, (primitive.min_corner, primitive.max_corner))
		}));
		let node_index = index(output.nodes.len());
		output.nodes.push(Self {
			min_corner: min, max_corner: max,
			sub_nodes: BVHInternal::Leaf { start, count: index(indices.len()) },
		});
		node_index
	}

	fn build_subtree<Index: Copy + Send + Sync + 'static>(
		primitives: Arc<[BuildPrimitive<Index>]>, indices: &mut [u16], output: &mut BuildBuffer<Index>,
	) -> u16 {
		assert!(!indices.is_empty());
		let first = primitives[indices[0] as usize];
		let mut bounds_min = first.min_corner;
		let mut bounds_max = first.max_corner;
		let mut centroid_min = midpoint(first.min_corner, first.max_corner);
		let mut centroid_max = centroid_min;
		for &primitive_index in indices.iter() {
			let primitive = primitives[primitive_index as usize];
			bounds_min = bounds_min.min(primitive.min_corner);
			bounds_max = bounds_max.max(primitive.max_corner);
			let centroid = midpoint(primitive.min_corner, primitive.max_corner);
			centroid_min = centroid_min.min(centroid);
			centroid_max = centroid_max.max(centroid);
		}

		if indices.len() <= 4 || centroid_min == centroid_max {
			return Self::build_leaf(&primitives, indices, output, bounds_min, bounds_max);
		}

		let unit = (bounds_max - bounds_min).max_element();
		let parent_area = surface_area(bounds_min, bounds_max, unit);
		if parent_area.is_zero() {
			return Self::build_leaf(&primitives, indices, output, bounds_min, bounds_max);
		}
		let no_split_cost = Fixed::from_num(indices.len()) * INTERSECT_COST;
		let mut best_cost = no_split_cost;
		let mut best_axis = 0usize;
		let mut best_bin = 0usize;
		let axis_extent = centroid_max - centroid_min;

		let mut bins_by_axis: [[Bin; BIN_COUNT]; 3] = std::array::from_fn(|_| std::array::from_fn(|_| Bin::empty()));
		for &primitive_index in indices.iter() {
			let primitive = primitives[primitive_index as usize];
			let centroid = midpoint(primitive.min_corner, primitive.max_corner);
			for axis in 0..3 {
				if axis_extent[axis].is_zero() { continue; }
				let bin = bin_index(centroid[axis], centroid_min[axis], axis_extent[axis]);
				bins_by_axis[axis][bin].extend(primitive.min_corner, primitive.max_corner);
			}
		}

		for axis in 0..3 {
			if axis_extent[axis].is_zero() { continue; }
			let bins = &bins_by_axis[axis];
			let mut left_costs = [None; BIN_COUNT - 1];
			let mut rmin = FixedVec3::MAX;
			let mut rmax = FixedVec3::MIN;
			let mut cnt = 0u32;
			for split_bin in 0..(BIN_COUNT - 1) {
				let bin = &bins[split_bin];
				if bin.primitive_count > 0 {
					rmin = rmin.min(bin.min_corner);
					rmax = rmax.max(bin.max_corner);
					cnt += bin.primitive_count;
				}
				if cnt > 0 { left_costs[split_bin] = Some(Fixed::from_num(cnt) * surface_area(rmin, rmax, unit)); }
			}

			let mut right_costs = [None; BIN_COUNT - 1];
			let mut rmin = FixedVec3::MAX;
			let mut rmax = FixedVec3::MIN;
			let mut cnt = 0u32;
			for split_bin in (0..(BIN_COUNT - 1)).rev() {
				let bin = &bins[split_bin + 1];
				if bin.primitive_count > 0 {
					rmin = rmin.min(bin.min_corner);
					rmax = rmax.max(bin.max_corner);
					cnt += bin.primitive_count;
				}
				if cnt > 0 { right_costs[split_bin] = Some(Fixed::from_num(cnt) * surface_area(rmin, rmax, unit)); }
			}

			for split_bin in 0..(BIN_COUNT - 1) {
				let (Some(left), Some(right)) = (left_costs[split_bin], right_costs[split_bin]) else { continue };
				let cost = TRAVERSAL_COST + (left + right) / parent_area * INTERSECT_COST;
				if cost < best_cost {
					best_cost = cost;
					best_axis = axis;
					best_bin = split_bin;
				}
			}
		}

		if best_cost >= no_split_cost {
			return Self::build_leaf(&primitives, indices, output, bounds_min, bounds_max);
		}

		let mut split_index = 0;
		for cursor in 0..indices.len() {
			let primitive = primitives[indices[cursor] as usize];
			let centroid = midpoint(primitive.min_corner, primitive.max_corner);
			if bin_index(centroid[best_axis], centroid_min[best_axis], axis_extent[best_axis]) <= best_bin {
				indices.swap(split_index, cursor);
				split_index += 1;
			}
		}
		assert!(split_index > 0 && split_index < indices.len());

		let node_index = index(output.nodes.len());
		output.nodes.push(Self {
			min_corner: bounds_min, max_corner: bounds_max,
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
			let spawned_node_base = index(output.nodes.len());
			let spawned_item_base = index(output.items.len());
			rebase_subtree(&mut spawned_tree, spawned_node_base, spawned_item_base);
			let spawned_root = spawned_local_root.checked_add(spawned_node_base).expect("BVH node index overflow");
			assert!(output.nodes.len() + spawned_tree.nodes.len() <= u16::MAX as usize + 1, "BVH exceeds u16 node capacity");
			output.nodes.extend(spawned_tree.nodes);
			output.items.extend(spawned_tree.items);
			if spawned_is_left { (spawned_root, local_root) } else { (local_root, spawned_root) }
		} else {
			let left_root = Self::build_subtree(primitives.clone(), left_indices, output);
			let right_root = Self::build_subtree(primitives.clone(), right_indices, output);
			(left_root, right_root)
		};

		output.nodes[node_index as usize].sub_nodes = BVHInternal::SubNodes { sub1: left_root, sub2: right_root };
		node_index
	}
}

impl<Index: Copy + std::fmt::Debug + PartialEq> BVH<Index> {
	pub fn new(items: Vec<(Index, (FixedVec3, FixedVec3))>) -> Self
	where Index: Send + Sync + 'static,
	{
		let _zone = span!("BVH creation");
		let item_count = index(items.len());
		let primitives: Arc<[BuildPrimitive<Index>]> = items.into_iter().map(|(index, (min_corner, max_corner))| {
			assert!(min_corner.cmple(max_corner).all(), "invalid BVH bounds");
			BuildPrimitive { index, min_corner, max_corner }
		}).collect::<Vec<_>>().into();
		if item_count == 0 {
			return BVH { nodes: vec![BVHNode {
				min_corner: FixedVec3::ZERO, max_corner: FixedVec3::ZERO,
				sub_nodes: BVHInternal::Leaf { start: 0, count: 0 },
			}], items: Vec::new() };
		}

		let mut indices: Vec<u16> = (0..item_count).collect();
		let mut output = BuildBuffer::with_capacity(item_count as usize);
		BVHNode::build_subtree(primitives, &mut indices, &mut output);
		BVH { nodes: output.nodes, items: output.items }
	}
}
