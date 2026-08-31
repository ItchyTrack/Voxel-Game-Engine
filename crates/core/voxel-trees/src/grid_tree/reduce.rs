use bevy::math::{IVec3, UVec3};

use super::*;
use crate::views::{GridTreeView, NodeRef};
use view::GridTreeViewImpl;

pub struct SourceTree<'a, G: GridType> {
	pub tree: &'a GridTree<G>,

	/// Source-local coordinates are divided by `1 << scale_down`.
	pub scale_down: u8,

	/// Added after scale-down to place source data in output space.
	pub output_offset: IVec3,
}

pub struct SourceOverlap<'a, G: GridType> {
	pub source_index: usize,

	/// Source-local region contributing to the current output region.
	pub source_region: NonZeroVoxelRegion,

	/// Projected/clipped region in output coordinates.
	pub output_region: NonZeroVoxelRegion,

	pub data: G::Data<'a>,
}

impl<'a, G: GridType> Copy for SourceOverlap<'a, G> {}
impl<'a, G: GridType> Clone for SourceOverlap<'a, G> {
	fn clone(&self) -> Self { *self }
}

#[derive(Clone, Copy)]
struct SourceCursor<'a, G: GridType> {
	source_index: usize,
	node: NodeRef<<GridTreeViewImpl<'a, G> as GridTreeView<'a>>::NodeHandle>,
	query: NonZeroVoxelRegion,
}

pub struct SourceOverlaps<'overlaps, 'a, G: GridType> {
	collected: &'overlaps [SourceOverlap<'a, G>],
	index: usize,
}

impl<'overlaps, 'a, G: GridType> SourceOverlaps<'overlaps, 'a, G> {
	fn from_collected(collected: &'overlaps [SourceOverlap<'a, G>]) -> Self {
		Self { collected, index: 0 }
	}
}

impl<'overlaps, 'a, G> Iterator for SourceOverlaps<'overlaps, 'a, G>
where
	G: GridType,
{
	type Item = SourceOverlap<'a, G>;

	fn next(&mut self) -> Option<Self::Item> {
		let overlap = self.collected.get(self.index).copied();
		self.index += overlap.is_some() as usize;
		overlap
	}
}

pub trait GridReducer<G: GridType> {
	type Output;

	fn output_grid_type(&self) -> G;

	fn reduce<'overlaps, 'a>(
		&mut self,
		region: NonZeroVoxelRegion,
		overlaps: SourceOverlaps<'overlaps, 'a, G>,
	) -> Option<Self::Output>;
}

pub fn reduce_grid_trees<'a, G, R>(
	output_region: NonZeroVoxelRegion,
	sources: &[SourceTree<'a, G>],
	mut reducer: R,
) -> Option<GridTree<G>>
where
	G: GridType,
	R: GridReducer<G>,
	for<'d> &'d R::Output: Into<G::Data<'d>>,
{
	if !region_fits_tree(output_region) {
		return None;
	}

	let mut out = GridTree::new_with_type(reducer.output_grid_type());
	if !out.make_sure_root_covers_area(output_region.min().as_uvec3(), output_region.max().as_uvec3()) || !out.has_node_budget() {
		return None;
	}
	let root_depth = out.raw.root_depth();
	let root_origin = out.raw.root_pos();
	let source_bounds = tree_region();
	let active_sources: Vec<_> = sources
		.iter()
		.enumerate()
		.filter_map(|(source_index, source)| root_source_cursor(source_index, source, output_region, source_bounds))
		.collect();
	let mut overlaps_scratch = Vec::new();
	if !reduce_output_region(output_region, sources, &active_sources, &mut overlaps_scratch, &mut reducer, &mut out, 0, root_depth, root_origin) {
		return None;
	}
	if out.is_empty() {
		return None;
	}
	Some(out)
}

fn reduce_output_region<'a, G, R>(
	region: NonZeroVoxelRegion,
	sources: &[SourceTree<'a, G>],
	active_sources: &[SourceCursor<'a, G>],
	overlaps_scratch: &mut Vec<SourceOverlap<'a, G>>,
	reducer: &mut R,
	out: &mut GridTree<G>,
	node_index: u32,
	node_depth: u8,
	node_origin: UVec3,
) -> bool
where
	G: GridType,
	R: GridReducer<G>,
	for<'d> &'d R::Output: Into<G::Data<'d>>,
{
	let node_region = NonZeroVoxelRegion::from_min_size(node_origin.as_ivec3(), UVec3::splat(size(node_depth) as u32)).unwrap();
	let Some(region) = node_region.intersection(region) else { return true };
	let cell_size = child_size(node_depth);
	let child_min = (region.min().as_uvec3() - node_origin) / UVec3::splat(cell_size);
	let child_max = (region.end().as_uvec3() - node_origin - UVec3::ONE) / UVec3::splat(cell_size);
	let mut child_active_sources: [smallvec::SmallVec<[SourceCursor<G>; 2]>; SIZE_USIZE_CUBED] = std::array::from_fn(|_| smallvec::SmallVec::new());
	partition_source_cursors_by_child(sources, active_sources, region, node_origin, cell_size, &mut child_active_sources);

	for z in child_min.z..=child_max.z {
		for y in child_min.y..=child_max.y {
			for x in child_min.x..=child_max.x {
				let child_index = (x + y * SIZE as u32 + z * SIZE as u32 * SIZE as u32) as u8;
				let child_origin = node_origin + UVec3::new(x as u32, y as u32, z as u32) * cell_size;
				let child_region = NonZeroVoxelRegion::from_min_size(child_origin.as_ivec3(), UVec3::splat(cell_size)).unwrap();
				let Some(work_region) = child_region.intersection(region) else { continue };
				let child_active_sources = &child_active_sources[child_index as usize];

				if work_region == child_region {
					if !reduce_output_child(child_region, sources, child_active_sources, overlaps_scratch, reducer, out, node_index, node_depth, node_origin, child_index) {
						return false;
					}
					continue;
				}

				if node_depth == 0 {
					if !reduce_output_child(child_region, sources, child_active_sources, overlaps_scratch, reducer, out, node_index, node_depth, node_origin, child_index) {
						return false;
					}
					continue;
				}

				let Some(child_node_index) = out.child_node_for_partial_area(node_index, child_index) else { return false };
				if !reduce_output_region(work_region, sources, child_active_sources, overlaps_scratch, reducer, out, child_node_index, node_depth - 1, child_origin) {
					return false;
				}
				out.collapse_child_node_if_possible(node_index, child_index);
			}
		}
	}
	true
}

fn reduce_output_child<'a, G, R>(
	region: NonZeroVoxelRegion,
	sources: &[SourceTree<'a, G>],
	active_sources: &[SourceCursor<'a, G>],
	overlaps_scratch: &mut Vec<SourceOverlap<'a, G>>,
	reducer: &mut R,
	out: &mut GridTree<G>,
	node_index: u32,
	node_depth: u8,
	node_origin: UVec3,
	child_index: u8,
) -> bool
where
	G: GridType,
	R: GridReducer<G>,
	for<'d> &'d R::Output: Into<G::Data<'d>>,
{
	match classify_region(sources, region, active_sources, overlaps_scratch) {
		RegionAction::Empty => true,
		RegionAction::Reduce => {
			if let Some(value) = reducer.reduce(region, SourceOverlaps::<G>::from_collected(overlaps_scratch)) {
				out.set_child_area_to_data(node_index, node_depth, child_index, (&value).into());
			}
			true
		}
		RegionAction::Split { a, b } => {
			if node_depth == 0 {
				return true;
			}
			let child_origin = node_origin + get_child_contents_pos(child_index).as_uvec3() * child_size(node_depth);
			let Some(child_node_index) = out.child_node_for_partial_area(node_index, child_index) else { return false };
			let a_active_sources = child_source_cursors(sources, active_sources, a);
			if !reduce_output_region(a, sources, &a_active_sources, overlaps_scratch, reducer, out, child_node_index, node_depth - 1, child_origin) {
				return false;
			}
			let b_active_sources = child_source_cursors(sources, active_sources, b);
			if !reduce_output_region(b, sources, &b_active_sources, overlaps_scratch, reducer, out, child_node_index, node_depth - 1, child_origin) {
				return false;
			}
			out.collapse_child_node_if_possible(node_index, child_index);
			true
		}
	}
}

#[derive(Clone, Copy)]
enum RegionAction {
	Empty,
	Reduce,
	Split { a: NonZeroVoxelRegion, b: NonZeroVoxelRegion },
}

fn classify_region<'a, G>(
	sources: &[SourceTree<'a, G>],
	region: NonZeroVoxelRegion,
	active_sources: &[SourceCursor<'a, G>],
	overlaps: &mut Vec<SourceOverlap<'a, G>>,
) -> RegionAction
where
	G: GridType,
{
	overlaps.clear();
	for &cursor in active_sources {
		if let SourceRegionInspection::Split(a, b) = inspect_source_region(&sources[cursor.source_index], region, cursor, overlaps) {
			return RegionAction::Split { a, b };
		}
	}

	if overlaps.is_empty() { RegionAction::Empty } else { RegionAction::Reduce }
}

#[derive(Clone, Copy)]
enum SourceRegionInspection {
	Empty,
	Overlap,
	Split(NonZeroVoxelRegion, NonZeroVoxelRegion),
}

fn inspect_source_region<'a, G>(
	source: &SourceTree<'a, G>,
	region: NonZeroVoxelRegion,
	cursor: SourceCursor<'a, G>,
	overlaps: &mut Vec<SourceOverlap<'a, G>>,
) -> SourceRegionInspection
where
	G: GridType,
{
	fn recurse<'tree, G>(
		source: &SourceTree<'tree, G>,
		region: NonZeroVoxelRegion,
		source_index: usize,
		node: NodeRef<<GridTreeViewImpl<'tree, G> as GridTreeView<'tree>>::NodeHandle>,
		query: NonZeroVoxelRegion,
		overlaps: &mut Vec<SourceOverlap<'tree, G>>,
	) -> Option<(NonZeroVoxelRegion, NonZeroVoxelRegion)>
	where
		G: GridType,
	{
		let view = source.tree.view();
		for (child, source_region) in view.occupied_children_in_region(node, query) {
			let Some(output_region) = project_source_region(source_region, source).and_then(|projected| projected.intersection(region)) else { continue };
			if !region_is_unit(region) {
				if let Some(split) = split_from_output_region(region, output_region) {
					return Some(split);
				}
			}

			match child.kind {
				CellKind::Empty => {}
				CellKind::Data => overlaps.push(SourceOverlap { source_index, source_region, output_region, data: view.cell_data(child.child_handle.unwrap(), child.child_index)}),
				CellKind::Node => {
					let child_node = child.node_ref().expect("Node kind implies child_node");
					if let Some(split) = recurse(source, region, source_index, child_node, source_region, overlaps) {
						return Some(split);
					}
				}
			}
		}
		None
	}

	let overlap_start = overlaps.len();
	if let Some((a, b)) = recurse(source, region, cursor.source_index, cursor.node, cursor.query, overlaps) {
		return SourceRegionInspection::Split(a, b);
	}
	if overlaps.len() == overlap_start { SourceRegionInspection::Empty } else { SourceRegionInspection::Overlap }
}

fn root_source_cursor<'tree, G>(source_index: usize, source: &SourceTree<'tree, G>, output_region: NonZeroVoxelRegion, source_bounds: NonZeroVoxelRegion) -> Option<SourceCursor<'tree, G>>
where
	G: GridType,
{
	let query = source_preimage(output_region, source).and_then(|region| region.intersection(source_bounds))?;
	let view = source.tree.view();
	let root = view.root();
	let root_region = NonZeroVoxelRegion::from_min_size(root.origin.as_ivec3(), UVec3::splat(size(root.depth))).unwrap();
	let query = query.intersection(root_region)?;
	refine_cursor(source, SourceCursor::<'tree, G> { source_index, node: root, query })
}

fn partition_source_cursors_by_child<'a, G>(
	sources: &[SourceTree<'a, G>],
	active_sources: &[SourceCursor<'a, G>],
	region: NonZeroVoxelRegion,
	node_origin: UVec3,
	cell_size: u32,
	child_active_sources: &mut [smallvec::SmallVec<[SourceCursor<'a, G>; 2]>; SIZE_USIZE_CUBED],
) where
	G: GridType,
{
	for &cursor in active_sources {
		let source = &sources[cursor.source_index];
		let Some(output_region) = project_source_region(cursor.query, source).and_then(|output_region| output_region.intersection(region)) else { continue };
		let child_min = (output_region.min().as_uvec3() - node_origin) / UVec3::splat(cell_size);
		let child_max = (output_region.end().as_uvec3() - node_origin - UVec3::ONE) / UVec3::splat(cell_size);
		for z in child_min.z..=child_max.z {
			for y in child_min.y..=child_max.y {
				for x in child_min.x..=child_max.x {
					let child_index = (x + y * SIZE as u32 + z * SIZE as u32 * SIZE as u32) as u8;
					let child_origin = node_origin + UVec3::new(x, y, z) * cell_size;
					let child_region = NonZeroVoxelRegion::from_min_size(child_origin.as_ivec3(), UVec3::splat(cell_size)).unwrap();
					let Some(work_region) = child_region.intersection(region) else { continue };
					if let Some(child_ref) = child_ref(source, cursor, work_region) {
						child_active_sources[child_index as usize].push(child_ref);
					}
				}
			}
		}
	}
}

fn child_source_cursors<'tree, G>(sources: &[SourceTree<'tree, G>], active_sources: &[SourceCursor<'tree, G>], region: NonZeroVoxelRegion) -> Vec<SourceCursor<'tree, G>>
where
	G: GridType,
{
	let mut child_sources = Vec::new();
	for &cursor in active_sources {
		if let Some(child_ref) = child_ref(&sources[cursor.source_index], cursor, region) {
			child_sources.push(child_ref);
		}
	}
	child_sources
}

fn child_ref<'tree, G>(source: &SourceTree<'tree, G>, cursor: SourceCursor<'tree, G>, child_region: NonZeroVoxelRegion) -> Option<SourceCursor<'tree, G>>
where
	G: GridType,
{
	let query = source_preimage(child_region, source)?.intersection(cursor.query)?;
	refine_cursor(source, SourceCursor::<G> { query, ..cursor })
}

fn refine_cursor<'tree, G>(source: &SourceTree<'tree, G>, mut cursor: SourceCursor<'tree, G>) -> Option<SourceCursor<'tree, G>>
where
	G: GridType,
{
	let view = source.tree.view();
	loop {
		let mut intersecting_node_child = None;
		let mut intersecting_children = 0u8;

		for (child, source_region) in view.occupied_children_in_region(cursor.node, cursor.query) {
			intersecting_children = intersecting_children.saturating_add(1);
			if child.kind == CellKind::Node {
				intersecting_node_child = Some((view.child(cursor.node, child.child_index).node_ref().expect("Node kind implies child_node"), source_region));
			}
		}

		if intersecting_children == 0 {
			return None;
		}
		if intersecting_children != 1 {
			return Some(cursor);
		}
		let Some((node, query)) = intersecting_node_child else {
			return Some(cursor);
		};
		cursor.node = node;
		cursor.query = query;
	}
}

fn split_from_output_region(region: NonZeroVoxelRegion, output_region: NonZeroVoxelRegion) -> Option<(NonZeroVoxelRegion, NonZeroVoxelRegion)> {
	for axis in 0..3 {
		let min = axis_value(output_region.min(), axis);
		let end = axis_value(output_region.end(), axis);
		let region_min = axis_value(region.min(), axis);
		let region_end = axis_value(region.end(), axis);
		if region_min < min && min < region_end {
			return split_at(region, axis, min);
		}
		if region_min < end && end < region_end {
			return split_at(region, axis, end);
		}
	}
	None
}

fn split_at(region: NonZeroVoxelRegion, axis: usize, cut: i32) -> Option<(NonZeroVoxelRegion, NonZeroVoxelRegion)> {
	let mut a_end = region.end();
	set_axis(&mut a_end, axis, cut);
	let mut b_min = region.min();
	set_axis(&mut b_min, axis, cut);
	Some((NonZeroVoxelRegion::from_min_end(region.min(), a_end)?, NonZeroVoxelRegion::from_min_end(b_min, region.end())?))
}

fn source_preimage<G: GridType>(region: NonZeroVoxelRegion, source: &SourceTree<'_, G>) -> Option<NonZeroVoxelRegion> {
	let factor = scale_factor(source.scale_down)?;
	let min = checked_mul_ivec3(checked_sub_ivec3(region.min(), source.output_offset)?, factor)?;
	let end = checked_mul_ivec3(checked_sub_ivec3(region.end(), source.output_offset)?, factor)?;
	NonZeroVoxelRegion::from_min_end(min, end)
}

fn project_source_region<G: GridType>(region: NonZeroVoxelRegion, source: &SourceTree<'_, G>) -> Option<NonZeroVoxelRegion> {
	let factor = scale_factor(source.scale_down)?;
	let min = checked_add_ivec3(div_floor_ivec3(region.min(), factor), source.output_offset)?;
	let end = checked_add_ivec3(div_ceil_ivec3(region.end(), factor), source.output_offset)?;
	NonZeroVoxelRegion::from_min_end(min, end)
}

fn scale_factor(scale_down: u8) -> Option<i32> {
	i32::try_from(1u32.checked_shl(scale_down as u32)?).ok()
}

fn checked_add_ivec3(a: IVec3, b: IVec3) -> Option<IVec3> {
	Some(IVec3::new(a.x.checked_add(b.x)?, a.y.checked_add(b.y)?, a.z.checked_add(b.z)?))
}

fn checked_sub_ivec3(a: IVec3, b: IVec3) -> Option<IVec3> {
	Some(IVec3::new(a.x.checked_sub(b.x)?, a.y.checked_sub(b.y)?, a.z.checked_sub(b.z)?))
}

fn checked_mul_ivec3(v: IVec3, factor: i32) -> Option<IVec3> {
	Some(IVec3::new(v.x.checked_mul(factor)?, v.y.checked_mul(factor)?, v.z.checked_mul(factor)?))
}

fn div_floor_ivec3(v: IVec3, divisor: i32) -> IVec3 {
	IVec3::new(v.x.div_euclid(divisor), v.y.div_euclid(divisor), v.z.div_euclid(divisor))
}

fn div_ceil_ivec3(v: IVec3, divisor: i32) -> IVec3 {
	IVec3::new(div_ceil(v.x, divisor), div_ceil(v.y, divisor), div_ceil(v.z, divisor))
}

fn div_ceil(value: i32, divisor: i32) -> i32 {
	let floor = value.div_euclid(divisor);
	if value.rem_euclid(divisor) == 0 { floor } else { floor + 1 }
}

fn region_is_unit(region: NonZeroVoxelRegion) -> bool {
	region.size() == UVec3::ONE
}

fn region_fits_tree(region: NonZeroVoxelRegion) -> bool {
	tree_region().contains_region(region)
}

fn tree_region() -> NonZeroVoxelRegion {
	NonZeroVoxelRegion::from_min_size(IVec3::ZERO, UVec3::splat(size(13/*Co::MAX_ROOT_DEPTH*/))).unwrap()
}

fn axis_value(v: IVec3, axis: usize) -> i32 {
	match axis {
		0 => v.x,
		1 => v.y,
		2 => v.z,
		_ => unreachable!("invalid axis"),
	}
}

fn set_axis(v: &mut IVec3, axis: usize, value: i32) {
	match axis {
		0 => v.x = value,
		1 => v.y = value,
		2 => v.z = value,
		_ => unreachable!("invalid axis"),
	}
}

#[cfg(test)]
mod tests {
	use voxel_trees::grid_tree::U16Cell;

	use super::*;

	#[derive(Clone, Copy, Debug)]
	struct SumReducer;

	impl GridReducer<U16Cell> for SumReducer {
		type Output = u16;

		fn output_grid_type(&self) -> U16Cell {
			U16Cell
		}

		fn reduce<'overlaps, 'a>(
			&mut self,
			_region: NonZeroVoxelRegion,
			overlaps: SourceOverlaps<'overlaps, 'a, U16Cell>,
		) -> Option<Self::Output> {
			let mut seen = false;
			let mut sum = 0u16;
			for overlap in overlaps {
				seen = true;
				sum += overlap.data;
			}
			seen.then_some(sum)
		}
	}

	#[test]
	fn reduce_unit_region_sees_all_downsampled_cells_from_one_source() {
		let mut source = GridTree::<U16Cell>::new();
		source.insert(&UVec3::new(0, 0, 0), 10);
		source.insert(&UVec3::new(1, 0, 0), 20);

		let sources = [SourceTree { tree: &source, scale_down: 1, output_offset: IVec3::ZERO }];
		let output_region = NonZeroVoxelRegion::from_min_size(IVec3::ZERO, UVec3::ONE).expect("unit output region");
		let output = reduce_grid_trees(output_region, &sources, SumReducer).expect("reduced output");

		assert_eq!(output.get(&UVec3::ZERO), Some(30));
	}

	#[test]
	fn reduce_unit_region_sees_all_active_sources() {
		let mut first = GridTree::<U16Cell>::new();
		first.insert(&UVec3::new(0, 0, 0), 11);
		let mut second = GridTree::<U16Cell>::new();
		second.insert(&UVec3::new(1, 0, 0), 22);

		let sources = [
			SourceTree { tree: &first, scale_down: 1, output_offset: IVec3::ZERO },
			SourceTree { tree: &second, scale_down: 1, output_offset: IVec3::ZERO },
		];
		let output_region = NonZeroVoxelRegion::from_min_size(IVec3::ZERO, UVec3::ONE).expect("unit output region");
		let output = reduce_grid_trees(output_region, &sources, SumReducer).expect("reduced output");

		assert_eq!(output.get(&UVec3::ZERO), Some(33));
	}

	#[test]
	fn reduce_preserves_leaf_boundaries_across_a_larger_region() {
		let mut source = GridTree::<U16Cell>::new();
		source.add_area(&UVec3::ZERO, UVec3::new(4, 8, 8), 7);
		source.add_area(&UVec3::new(4, 0, 0), UVec3::new(4, 8, 8), 9);

		let sources = [SourceTree { tree: &source, scale_down: 0, output_offset: IVec3::ZERO }];
		let output_region = NonZeroVoxelRegion::from_min_size(IVec3::ZERO, UVec3::splat(8)).expect("output region");
		let output = reduce_grid_trees(output_region, &sources, SumReducer).expect("reduced output");

		assert_eq!(output.get(&UVec3::new(3, 6, 6)), Some(7));
		assert_eq!(output.get(&UVec3::new(4, 6, 6)), Some(9));
	}

	#[derive(Clone, Copy, Debug)]
	struct SourceVolumeReducer;

	impl GridReducer<U16Cell> for SourceVolumeReducer {
		type Output = u16;

		fn output_grid_type(&self) -> U16Cell { U16Cell }

		fn reduce<'overlaps, 'a>(
			&mut self,
			_region: NonZeroVoxelRegion,
			overlaps: SourceOverlaps<'overlaps, 'a, U16Cell>,
		) -> Option<Self::Output>
		{
			let mut volume = 0u16;
			for overlap in overlaps {
				let size = overlap.source_region.size();
				volume += (size.x * size.y * size.z) as u16;
			}
			(volume != 0).then_some(volume)
		}
	}

	#[test]
	fn reduce_keeps_source_region_clipping_when_reusing_overlaps() {
		let mut source = GridTree::<U16Cell>::new();
		source.add_area(&UVec3::ZERO, UVec3::splat(8), 1);

		let sources = [SourceTree { tree: &source, scale_down: 1, output_offset: IVec3::ZERO }];
		let output_region = NonZeroVoxelRegion::from_min_size(IVec3::ZERO, UVec3::splat(4)).expect("output region");
		let output = reduce_grid_trees(output_region, &sources, SourceVolumeReducer).expect("reduced output");

		assert_eq!(output.get(&UVec3::new(0, 0, 0)), Some(8));
		assert_eq!(output.get(&UVec3::new(3, 3, 3)), Some(8));
	}
}
