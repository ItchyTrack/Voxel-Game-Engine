use bevy::math::IVec3;

use super::*;

pub struct SourceTree<'a, G: GridType, Co: GridCoord> {
	pub tree: &'a GridTree<G, Co>,

	/// Source-local coordinates are divided by `1 << scale_down`.
	pub scale_down: u8,

	/// Added after scale-down to place source data in output space.
	pub output_offset: IVec3,
}

pub struct SourceOverlap<'a, G: GridType> {
	pub source_index: usize,

	/// Source-local region contributing to the current output region.
	pub source_region: GridRegion,

	/// Projected/clipped region in output coordinates.
	pub output_region: GridRegion,

	pub data: G::Data<'a>,
}

impl<'a, G: GridType> Copy for SourceOverlap<'a, G> {}
impl<'a, G: GridType> Clone for SourceOverlap<'a, G> {
	fn clone(&self) -> Self { *self }
}

struct OverlapFrame<'a, G: GridType, Co: GridCoord> {
	children: ChildCellsInRegion<'a, G, Co>,
}

impl<'a, G: GridType, Co: GridCoord> Copy for OverlapFrame<'a, G, Co> {}
impl<'a, G: GridType, Co: GridCoord> Clone for OverlapFrame<'a, G, Co> {
	fn clone(&self) -> Self { *self }
}

#[derive(Clone, Copy)]
struct SourceCursor {
	source_index: usize,
	node: NodeRef,
	query: GridRegion,
}

pub struct SourceOverlaps<'overlaps, 'a, G: GridType, Co: GridCoord> {
	sources: &'overlaps [SourceTree<'a, G, Co>],
	output_region: GridRegion,
	source_bounds: GridRegion,
	active: Option<&'overlaps [SourceCursor]>,
	first: Option<SourceOverlap<'a, G>>,
	source_index: usize,
	active_index: usize,
	current_source_index: usize,
	stack: [OverlapFrame<'a, G, Co>; MAX_TREE_DEPTH_USIZE + 1],
	stack_len: usize,
}

impl<'overlaps, 'a, G, Co> SourceOverlaps<'overlaps, 'a, G, Co>
where
	G: GridType,
	Co: GridCoord,
{
	fn single(sources: &'overlaps [SourceTree<'a, G, Co>], output_region: GridRegion, overlap: SourceOverlap<'a, G>) -> Self {
		Self::new(sources, output_region, Some(&[]), Some(overlap))
	}

	fn new(
		sources: &'overlaps [SourceTree<'a, G, Co>],
		output_region: GridRegion,
		active: Option<&'overlaps [SourceCursor]>,
		first: Option<SourceOverlap<'a, G>>,
	) -> Self {
		let source_bounds = tree_region::<Co>();
		let dummy_view = sources[0].tree.view();
		let dummy_root = dummy_view.root();
		let dummy = OverlapFrame { children: dummy_view.occupied_children_in_region(dummy_root, GridRegion { min: IVec3::ZERO, end: IVec3::ZERO }) };
		let mut overlaps = Self {
			sources,
			output_region,
			source_bounds,
			active,
			first,
			source_index: 0,
			active_index: 0,
			current_source_index: 0,
			stack: [dummy; MAX_TREE_DEPTH_USIZE + 1],
			stack_len: 0,
		};
		overlaps.advance_to_next_source();
		overlaps
	}

	fn advance_to_next_source(&mut self) {
		self.stack_len = 0;
		if let Some(active) = self.active {
			while self.active_index < active.len() {
				let cursor = active[self.active_index];
				self.active_index += 1;
				let source = &self.sources[cursor.source_index];
				self.current_source_index = cursor.source_index;
				self.stack[0] = OverlapFrame { children: source.tree.view().occupied_children_in_region(cursor.node, cursor.query) };
				self.stack_len = 1;
				return;
			}
			return;
		}

		while self.source_index < self.sources.len() {
			let source_index = self.source_index;
			self.source_index += 1;
			let source = &self.sources[source_index];
			let Some(cursor) = root_source_cursor(source_index, source, self.output_region, self.source_bounds) else { continue };
			self.current_source_index = cursor.source_index;
			self.stack[0] = OverlapFrame { children: source.tree.view().occupied_children_in_region(cursor.node, cursor.query) };
			self.stack_len = 1;
			return;
		}
	}
}

impl<'overlaps, 'a, G, Co> Iterator for SourceOverlaps<'overlaps, 'a, G, Co>
where
	G: GridType,
	Co: GridCoord,
{
	type Item = SourceOverlap<'a, G>;

	fn next(&mut self) -> Option<Self::Item> {
		if let Some(overlap) = self.first.take() {
			return Some(overlap);
		}
		loop {
			if self.stack_len == 0 {
				self.advance_to_next_source();
				if self.stack_len == 0 {
					return None;
				}
				continue;
			}

			let frame_index = self.stack_len - 1;
			let frame = &mut self.stack[frame_index];
			let Some((child, source_region)) = frame.children.next() else {
				self.stack_len -= 1;
				continue;
			};
			let source = &self.sources[self.current_source_index];
			let view = source.tree.view();

			match child.kind() {
				CellKind::Empty => {}
				CellKind::Data => {
					let Some(projected) = project_source_region(source_region, source) else { continue };
					let Some(output_region) = projected.intersection(self.output_region) else { continue };
					return Some(SourceOverlap { source_index: self.current_source_index, source_region, output_region, data: child.data_value() });
				}
				CellKind::Node => {
					let child_node = view.child_node(child).expect("Node kind implies child_node");
					debug_assert!(self.stack_len < self.stack.len());
					self.stack[self.stack_len] = OverlapFrame { children: view.occupied_children_in_region(child_node, source_region) };
					self.stack_len += 1;
				}
			}
		}
	}
}

pub fn reduce_grid_trees<'a, G, Co, F>(
	output_region: GridRegion,
	sources: &[SourceTree<'a, G, Co>],
	mut reduce: F,
) -> Option<GridTree<G, Co>>
where
	G: GridType,
	Co: GridCoord,
	F: for<'overlaps> FnMut(GridRegion, SourceOverlaps<'overlaps, 'a, G, Co>) -> Option<G::Data<'a>>,
{
	let output_region = GridRegion::new(output_region.min, output_region.end)?;
	if !region_fits_tree::<Co>(output_region) {
		return None;
	}

	let mut out = GridTree::new_with_type(sources.first()?.tree.grid_type().clone());
	if !out.make_sure_root_covers_area(output_region.min, output_region.max_inclusive()) || !out.has_node_budget() {
		return None;
	}
	let root_depth = out.raw.root_depth();
	let root_origin = out.raw.root_pos();
	let source_bounds = tree_region::<Co>();
	let active_sources: Vec<_> = sources
		.iter()
		.enumerate()
		.filter_map(|(source_index, source)| root_source_cursor(source_index, source, output_region, source_bounds))
		.collect();
	if !reduce_output_region(output_region, sources, &active_sources, &mut reduce, &mut out, 0, root_depth, root_origin) {
		return None;
	}
	if out.is_empty() {
		return None;
	}
	Some(out)
}

fn reduce_output_region<'a, G, Co, F>(
	region: GridRegion,
	sources: &[SourceTree<'a, G, Co>],
	active_sources: &[SourceCursor],
	reduce: &mut F,
	out: &mut GridTree<G, Co>,
	node_index: u32,
	node_depth: u8,
	node_origin: IVec3,
) -> bool
where
	G: GridType,
	Co: GridCoord,
	F: for<'overlaps> FnMut(GridRegion, SourceOverlaps<'overlaps, 'a, G, Co>) -> Option<G::Data<'a>>,
{
	let node_region = GridRegion { min: node_origin, end: node_origin + IVec3::splat(size(node_depth) as i32) };
	let Some(region) = node_region.intersection(region) else { return true };
	let cell_size = child_size(node_depth) as i32;
	let child_min = (region.min - node_origin).div_euclid(IVec3::splat(cell_size));
	let child_max = (region.end - node_origin - IVec3::ONE).div_euclid(IVec3::splat(cell_size));
	let mut child_active_sources: [Vec<SourceCursor>; SIZE_USIZE_CUBED] = std::array::from_fn(|_| Vec::new());
	partition_source_cursors_by_child(sources, active_sources, region, node_origin, cell_size, &mut child_active_sources);

	for z in child_min.z..=child_max.z {
		for y in child_min.y..=child_max.y {
			for x in child_min.x..=child_max.x {
				let child_index = (x + y * SIZE as i32 + z * SIZE as i32 * SIZE as i32) as u8;
				let child_origin = node_origin + IVec3::new(x, y, z) * cell_size;
				let child_region = GridRegion { min: child_origin, end: child_origin + IVec3::splat(cell_size) };
				let Some(work_region) = child_region.intersection(region) else { continue };
				let child_active_sources = &child_active_sources[child_index as usize];

				if work_region == child_region {
					if !reduce_output_child(child_region, sources, child_active_sources, reduce, out, node_index, node_depth, node_origin, child_index) {
						return false;
					}
					continue;
				}

				if node_depth == 0 {
					if !reduce_output_child(child_region, sources, child_active_sources, reduce, out, node_index, node_depth, node_origin, child_index) {
						return false;
					}
					continue;
				}

				let Some(child_node_index) = out.child_node_for_partial_area(node_index, child_index) else { return false };
				if !reduce_output_region(work_region, sources, child_active_sources, reduce, out, child_node_index, node_depth - 1, child_origin) {
					return false;
				}
				out.collapse_child_node_if_possible(node_index, child_index);
			}
		}
	}
	true
}

fn reduce_output_child<'a, G, Co, F>(
	region: GridRegion,
	sources: &[SourceTree<'a, G, Co>],
	active_sources: &[SourceCursor],
	reduce: &mut F,
	out: &mut GridTree<G, Co>,
	node_index: u32,
	node_depth: u8,
	node_origin: IVec3,
	child_index: u8,
) -> bool
where
	G: GridType,
	Co: GridCoord,
	F: for<'overlaps> FnMut(GridRegion, SourceOverlaps<'overlaps, 'a, G, Co>) -> Option<G::Data<'a>>,
{
	match classify_region(sources, region, active_sources) {
		RegionAction::Empty => true,
		RegionAction::Reduce(overlap) => {
			if let Some(value) = reduce(region, SourceOverlaps::single(sources, region, overlap)) {
				out.set_child_area_to_data(node_index, node_depth, child_index, value);
			}
			true
		}
		RegionAction::Split { a, b } => {
			if node_depth == 0 {
				return true;
			}
			let child_origin = node_origin + (get_child_contents_pos(child_index).as_uvec3() * child_size(node_depth)).as_ivec3();
			let Some(child_node_index) = out.child_node_for_partial_area(node_index, child_index) else { return false };
			let a_active_sources = child_source_cursors(sources, active_sources, a);
			if !reduce_output_region(a, sources, &a_active_sources, reduce, out, child_node_index, node_depth - 1, child_origin) {
				return false;
			}
			let b_active_sources = child_source_cursors(sources, active_sources, b);
			if !reduce_output_region(b, sources, &b_active_sources, reduce, out, child_node_index, node_depth - 1, child_origin) {
				return false;
			}
			out.collapse_child_node_if_possible(node_index, child_index);
			true
		}
	}
}

#[derive(Clone, Copy)]
enum RegionAction<'a, G: GridType> {
	Empty,
	Reduce(SourceOverlap<'a, G>),
	Split { a: GridRegion, b: GridRegion },
}

fn classify_region<'a, G, Co>(sources: &[SourceTree<'a, G, Co>], region: GridRegion, active_sources: &[SourceCursor]) -> RegionAction<'a, G>
where
	G: GridType,
	Co: GridCoord,
{
	let mut first_overlap = None;
	for &cursor in active_sources {
		match inspect_source_region(&sources[cursor.source_index], region, cursor) {
			SourceRegionInspection::Empty => {}
			SourceRegionInspection::Split(a, b) => return RegionAction::Split { a, b },
			SourceRegionInspection::Overlap(overlap) => {
				let covers = source_preimage(region, &sources[overlap.source_index]).is_some_and(|source_region| {
					overlap.output_region.contains_region(region) && overlap.source_region.contains_region(source_region)
				});
				if covers || region_is_unit(region) {
					// Source data regions are disjoint in output space, so a single full-covering
					// overlap proves no other source can contribute inside this region.
					return RegionAction::Reduce(overlap);
				}
				if first_overlap.is_none() {
					first_overlap = Some(overlap);
				}
			}
		}
	}

	let Some(overlap) = first_overlap else {
		return RegionAction::Empty;
	};

	if let Some((a, b)) = split_from_output_region(region, overlap.output_region) {
		RegionAction::Split { a, b }
	} else if let Some((a, b)) = split_longest_axis(region) {
		RegionAction::Split { a, b }
	} else {
		RegionAction::Reduce(overlap)
	}
}

#[derive(Clone, Copy)]
enum SourceRegionInspection<'a, G: GridType> {
	Empty,
	Overlap(SourceOverlap<'a, G>),
	Split(GridRegion, GridRegion),
}

fn inspect_source_region<'a, G, Co>(source: &SourceTree<'a, G, Co>, region: GridRegion, cursor: SourceCursor) -> SourceRegionInspection<'a, G>
where
	G: GridType,
	Co: GridCoord,
{
	let view = source.tree.view();
	let mut stack = [OverlapFrame { children: view.occupied_children_in_region(cursor.node, cursor.query) }; MAX_TREE_DEPTH_USIZE + 1];
	let mut stack_len = 1usize;
	let mut first_overlap = None;

	while stack_len > 0 {
		let frame_index = stack_len - 1;
		let Some((child, source_region)) = stack[frame_index].children.next() else {
			stack_len -= 1;
			continue;
		};

		let Some(output_region) = project_source_region(source_region, source).and_then(|projected| projected.intersection(region)) else { continue };
		if !region_is_unit(region) {
			if let Some((a, b)) = split_from_output_region(region, output_region) {
				return SourceRegionInspection::Split(a, b);
			}
		}

		match child.kind() {
			CellKind::Empty => {}
			CellKind::Data => {
				let overlap = SourceOverlap { source_index: cursor.source_index, source_region, output_region, data: child.data_value() };
				if first_overlap.is_none() {
					first_overlap = Some(overlap);
				}
			}
			CellKind::Node => {
				let child_node = view.child_node(child).expect("Node kind implies child_node");
				debug_assert!(stack_len < stack.len());
				stack[stack_len] = OverlapFrame { children: view.occupied_children_in_region(child_node, source_region) };
				stack_len += 1;
			}
		}
	}

	first_overlap.map_or(SourceRegionInspection::Empty, SourceRegionInspection::Overlap)
}

fn root_source_cursor<G, Co>(source_index: usize, source: &SourceTree<'_, G, Co>, output_region: GridRegion, source_bounds: GridRegion) -> Option<SourceCursor>
where
	G: GridType,
	Co: GridCoord,
{
	let query = source_preimage(output_region, source).and_then(|region| region.intersection(source_bounds))?;
	let view = source.tree.view();
	let root = view.root();
	let root_region = GridRegion { min: root.origin, end: root.origin + IVec3::splat(size(root.depth) as i32) };
	let query = query.intersection(root_region)?;
	refine_cursor(source, SourceCursor { source_index, node: root, query })
}

fn partition_source_cursors_by_child<G, Co>(
	sources: &[SourceTree<'_, G, Co>],
	active_sources: &[SourceCursor],
	region: GridRegion,
	node_origin: IVec3,
	cell_size: i32,
	child_active_sources: &mut [Vec<SourceCursor>; SIZE_USIZE_CUBED],
) where
	G: GridType,
	Co: GridCoord,
{
	for &cursor in active_sources {
		let source = &sources[cursor.source_index];
		let Some(output_region) = project_source_region(cursor.query, source).and_then(|output_region| output_region.intersection(region)) else { continue };
		let child_min = (output_region.min - node_origin).div_euclid(IVec3::splat(cell_size));
		let child_max = (output_region.end - node_origin - IVec3::ONE).div_euclid(IVec3::splat(cell_size));
		for z in child_min.z..=child_max.z {
			for y in child_min.y..=child_max.y {
				for x in child_min.x..=child_max.x {
					let child_index = (x + y * SIZE as i32 + z * SIZE as i32 * SIZE as i32) as u8;
					let child_origin = node_origin + IVec3::new(x, y, z) * cell_size;
					let child_region = GridRegion { min: child_origin, end: child_origin + IVec3::splat(cell_size) };
					let Some(work_region) = child_region.intersection(region) else { continue };
					if let Some(child_cursor) = child_cursor(source, cursor, work_region) {
						child_active_sources[child_index as usize].push(child_cursor);
					}
				}
			}
		}
	}
}

fn child_source_cursors<G, Co>(sources: &[SourceTree<'_, G, Co>], active_sources: &[SourceCursor], region: GridRegion) -> Vec<SourceCursor>
where
	G: GridType,
	Co: GridCoord,
{
	let mut child_sources = Vec::new();
	for &cursor in active_sources {
		if let Some(child_cursor) = child_cursor(&sources[cursor.source_index], cursor, region) {
			child_sources.push(child_cursor);
		}
	}
	child_sources
}

fn child_cursor<G, Co>(source: &SourceTree<'_, G, Co>, cursor: SourceCursor, child_region: GridRegion) -> Option<SourceCursor>
where
	G: GridType,
	Co: GridCoord,
{
	let query = source_preimage(child_region, source)?.intersection(cursor.query)?;
	refine_cursor(source, SourceCursor { query, ..cursor })
}

fn refine_cursor<G, Co>(source: &SourceTree<'_, G, Co>, mut cursor: SourceCursor) -> Option<SourceCursor>
where
	G: GridType,
	Co: GridCoord,
{
	let view = source.tree.view();
	loop {
		let mut intersecting_node_child = None;
		let mut intersecting_children = 0u8;

		for (child, source_region) in view.occupied_children_in_region(cursor.node, cursor.query) {
			intersecting_children = intersecting_children.saturating_add(1);
			if child.kind() == CellKind::Node {
				intersecting_node_child = Some((view.child_node(child).expect("Node kind implies child_node"), source_region));
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

fn split_from_output_region(region: GridRegion, output_region: GridRegion) -> Option<(GridRegion, GridRegion)> {
	for axis in 0..3 {
		let min = axis_value(output_region.min, axis);
		let end = axis_value(output_region.end, axis);
		let region_min = axis_value(region.min, axis);
		let region_end = axis_value(region.end, axis);
		if region_min < min && min < region_end {
			return split_at(region, axis, min);
		}
		if region_min < end && end < region_end {
			return split_at(region, axis, end);
		}
	}
	None
}

fn split_longest_axis(region: GridRegion) -> Option<(GridRegion, GridRegion)> {
	let size = region.size();
	let axis = if size.x >= size.y && size.x >= size.z {
		0
	} else if size.y >= size.z {
		1
	} else {
		2
	};
	let len = axis_value(size, axis);
	if len <= 1 {
		return None;
	}
	split_at(region, axis, axis_value(region.min, axis) + len / 2)
}

fn split_at(region: GridRegion, axis: usize, cut: i32) -> Option<(GridRegion, GridRegion)> {
	let mut a_end = region.end;
	set_axis(&mut a_end, axis, cut);
	let mut b_min = region.min;
	set_axis(&mut b_min, axis, cut);
	Some((GridRegion::new(region.min, a_end)?, GridRegion::new(b_min, region.end)?))
}

fn source_preimage<G: GridType, Co: GridCoord>(region: GridRegion, source: &SourceTree<'_, G, Co>) -> Option<GridRegion> {
	let factor = scale_factor(source.scale_down)?;
	let min = checked_mul_ivec3(checked_sub_ivec3(region.min, source.output_offset)?, factor)?;
	let end = checked_mul_ivec3(checked_sub_ivec3(region.end, source.output_offset)?, factor)?;
	GridRegion::new(min, end)
}

fn project_source_region<G: GridType, Co: GridCoord>(region: GridRegion, source: &SourceTree<'_, G, Co>) -> Option<GridRegion> {
	let factor = scale_factor(source.scale_down)?;
	let min = checked_add_ivec3(div_floor_ivec3(region.min, factor), source.output_offset)?;
	let end = checked_add_ivec3(div_ceil_ivec3(region.end, factor), source.output_offset)?;
	GridRegion::new(min, end)
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

fn region_is_unit(region: GridRegion) -> bool {
	region.size() == IVec3::ONE
}

fn region_fits_tree<Co: GridCoord>(region: GridRegion) -> bool {
	tree_region::<Co>().contains_region(region)
}

fn tree_region<Co: GridCoord>() -> GridRegion {
	GridRegion { min: IVec3::ZERO, end: IVec3::splat(size(Co::MAX_ROOT_DEPTH) as i32) }
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
