use std::collections::{HashMap, HashSet};

use bevy::math::{I16Vec3, IVec3};
use bevy::tasks::ComputeTaskPool;
use tracy_client::span;

use crate::grid::{Grid, SubGridSlot};
use crate::voxels::Voxels;

pub struct GridSplat<'a> {
	pub grid: usize,
	pub base: IVec3,
	pub voxels: &'a Voxels,
}

#[derive(Clone, Copy)]
struct SourceSlice<'a> {
	base: IVec3,
	voxels: &'a Voxels,
	source_min: I16Vec3,
	source_size: I16Vec3,
}

struct SubGridSplatJob<'a> {
	grid: usize,
	sub_origin: IVec3,
	existing: Option<Voxels>,
	slices: Vec<SourceSlice<'a>>,
}

struct SubGridSliceGroup<'a> {
	grid: usize,
	sub_origin: IVec3,
	slices: Vec<SourceSlice<'a>>,
}

impl<'a> SubGridSliceGroup<'a> {
	fn push(groups: &mut Vec<Self>, grid: usize, sub_origin: IVec3, slice: SourceSlice<'a>) {
		if let Some(group) = groups.iter_mut().find(|group| group.grid == grid && group.sub_origin == sub_origin) {
			group.slices.push(slice);
		} else {
			groups.push(Self { grid, sub_origin, slices: vec![slice] });
		}
	}
}

#[derive(Debug)]
struct SubGridSplatResult {
	grid: usize,
	sub_origin: IVec3,
	voxels: Voxels,
}

/// Apply many voxel splats to many grids, blocking until all per-sub-grid build
/// work has completed. Inputs targeting the same `(grid, sub-grid)` are grouped
/// into one job, so overlapping chunk loads serialize deterministically within
/// that sub-grid without locks.
///
/// Grouping is intentionally per source `Voxels`/sub-grid intersection, not per
/// source leaf. The expensive source-tree iteration happens inside per-sub-grid
/// worker tasks instead of serially on the caller thread.
pub fn splat_voxels_blocking(grids: &mut [Grid], splats: &[GridSplat<'_>]) -> HashMap<usize, HashSet<IVec3>> {
	let _zone = span!("splat_voxels_blocking");

	let _group_zone = span!("group splats by subgrid");
	let mut grouped: Vec<SubGridSliceGroup<'_>> = Vec::new();
	for splat in splats {
		if splat.grid >= grids.len() { continue; }
		let Some((bounds_min, bounds_max)) = splat.voxels.bounding_box() else { continue };
		let world_min = splat.base + bounds_min.as_ivec3();
		let world_end = splat.base + bounds_max.as_ivec3() + IVec3::ONE;

		for (sub_origin, cell_lo, cell_hi) in grids[splat.grid].write_regions(world_min, world_end) {
			let source_min = (cell_lo - splat.base).as_i16vec3();
			let source_size = (cell_hi - cell_lo).as_i16vec3();
			SubGridSliceGroup::push(
				&mut grouped,
				splat.grid,
				sub_origin,
				SourceSlice { base: splat.base, voxels: splat.voxels, source_min, source_size },
			);
		}
	}
	drop(_group_zone);

	let _job_zone = span!("prepare subgrid splat jobs");
	let jobs: Vec<_> = grouped
		.into_iter()
		.map(|group| SubGridSplatJob {
			grid: group.grid,
			sub_origin: group.sub_origin,
			existing: grids[group.grid].subgrids.get(&group.sub_origin).map(|slot| slot.voxels.clone()),
			slices: group.slices,
		})
		.collect();
	drop(_job_zone);

	let _scope_zone = span!("compute subgrid splat jobs");
	let results: Vec<SubGridSplatResult> = ComputeTaskPool::get().scope(|scope| {
		for job in jobs {
			scope.spawn(async move {
				let _zone = span!("build one subgrid splat");
				let mut destination = job.existing.unwrap_or_else(Voxels::new);
				for slice in job.slices {
					let source_region = crate::grid_tree::GridRegion::from_min_size(slice.source_min.as_ivec3(), slice.source_size.as_ivec3()).unwrap();
					destination.merge_region_from(slice.voxels, Some(source_region), slice.base - job.sub_origin);
				}
				SubGridSplatResult { grid: job.grid, sub_origin: job.sub_origin, voxels: destination }
			});
		}
	});
	drop(_scope_zone);

	let _apply_zone = span!("apply subgrid splat results");
	let mut touched: HashMap<usize, HashSet<IVec3>> = HashMap::new();
	for result in results {
		let Some(grid) = grids.get_mut(result.grid) else { continue };
		if let Some(slot) = grid.subgrids.get_mut(&result.sub_origin) {
			slot.voxels = result.voxels;
		} else {
			let mut slot = SubGridSlot::new_default(result.sub_origin);
			slot.voxels = result.voxels;
			grid.subgrids.insert(result.sub_origin, slot);
		}
		touched.entry(result.grid).or_default().insert(result.sub_origin);
	}
	touched
}
