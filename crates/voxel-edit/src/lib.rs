use std::{collections::HashSet, sync::Arc};

use bevy::math::{IVec2, IVec3, Vec3};
use bevy::prelude::*;

use voxel_data::grid::Grid;
use voxel_data::region::{NonZeroVoxelRegion, VoxelRegion};
use voxel_data::sdf::{Sdf, shrink_aabb_with_sdf, voxel_center, voxel_region_from_bounds};
use voxel_data::voxels::Voxel;

#[derive(Clone)]
pub enum GridEdit {
	Add { voxel_pos: IVec3, voxel: Voxel },
	Remove { voxel_pos: IVec3 },
	AddArea { region: NonZeroVoxelRegion, voxel: Voxel },
	RemoveArea { region: NonZeroVoxelRegion },
	ApplySdf {
		bounds_min: Vec3,
		bounds_max: Vec3,
		face_resolution: IVec2,
		iterations: usize,
		voxel: Voxel,
		sdf: Arc<dyn Sdf>,
	},
	ClearSdf {
		bounds_min: Vec3,
		bounds_max: Vec3,
		face_resolution: IVec2,
		iterations: usize,
		sdf: Arc<dyn Sdf>,
	},
}

impl GridEdit {
	pub fn voxel_bounds(&self) -> VoxelRegion {
		match self {
			GridEdit::Add { voxel_pos, .. } | GridEdit::Remove { voxel_pos } => VoxelRegion::new(*voxel_pos, UVec3::ONE),
			GridEdit::AddArea { region, .. } | GridEdit::RemoveArea { region } => (*region).into(),
			GridEdit::ApplySdf { bounds_min, bounds_max, .. } | GridEdit::ClearSdf { bounds_min, bounds_max, .. } => {
				let min = bounds_min.floor().as_ivec3();
				VoxelRegion::new(min, (bounds_max.ceil().as_ivec3() - min).as_uvec3())
			}
		}
	}

	/// Resolve edits containing process-local SDFs into replayable area commands.
	pub fn resolved_commands(&self) -> Vec<Self> {
		match self {
			GridEdit::ApplySdf { bounds_min, bounds_max, face_resolution, iterations, voxel, sdf } => {
				sdf_area_edits(*bounds_min, *bounds_max, *face_resolution, *iterations, &**sdf, Some(voxel.clone()))
			}
			GridEdit::ClearSdf { bounds_min, bounds_max, face_resolution, iterations, sdf } => {
				sdf_area_edits(*bounds_min, *bounds_max, *face_resolution, *iterations, &**sdf, None)
			}
			_ => vec![self.clone()],
		}
	}

	/// Restrict this edit to `limit`, preserving world-space SDF coordinates.
	pub fn clipped_to(&self, limit: NonZeroVoxelRegion) -> Option<Self> {
		match self {
			GridEdit::Add { voxel_pos, voxel } => limit.contains(*voxel_pos).then(|| GridEdit::Add { voxel_pos: *voxel_pos, voxel: voxel.clone() }),
			GridEdit::Remove { voxel_pos } => limit.contains(*voxel_pos).then_some(GridEdit::Remove { voxel_pos: *voxel_pos }),
			GridEdit::AddArea { region, voxel } => region.intersection(limit).map(|region| GridEdit::AddArea { region, voxel: voxel.clone() }),
			GridEdit::RemoveArea { region } => region.intersection(limit).map(|region| GridEdit::RemoveArea { region }),
			GridEdit::ApplySdf { bounds_min, bounds_max, face_resolution, iterations, voxel, sdf } => {
				let min = bounds_min.max(limit.min().as_vec3());
				let max = bounds_max.min(limit.end().as_vec3());
				min.cmplt(max).all().then(|| GridEdit::ApplySdf {
					bounds_min: min,
					bounds_max: max,
					face_resolution: *face_resolution,
					iterations: *iterations,
					voxel: voxel.clone(),
					sdf: sdf.clone(),
				})
			}
			GridEdit::ClearSdf { bounds_min, bounds_max, face_resolution, iterations, sdf } => {
				let min = bounds_min.max(limit.min().as_vec3());
				let max = bounds_max.min(limit.end().as_vec3());
				min.cmplt(max).all().then(|| GridEdit::ClearSdf {
					bounds_min: min,
					bounds_max: max,
					face_resolution: *face_resolution,
					iterations: *iterations,
					sdf: sdf.clone(),
				})
			}
		}
	}
}

fn sdf_area_edits(
	bounds_min: Vec3,
	bounds_max: Vec3,
	face_resolution: IVec2,
	iterations: usize,
	sdf: &(impl Sdf + ?Sized),
	voxel: Option<Voxel>,
) -> Vec<GridEdit> {
	let (min, max) = shrink_aabb_with_sdf(bounds_min, bounds_max, sdf, face_resolution, iterations);
	let Some(region) = voxel_region_from_bounds(min, max) else { return Vec::new() };
	let mut edits = Vec::new();
	for z in region.min().z..region.end().z {
		for y in region.min().y..region.end().y {
			let mut x = region.min().x;
			while x < region.end().x {
				if sdf.sample(voxel_center(IVec3::new(x, y, z))) > 0.0 {
					x += 1;
					continue;
				}
				let run_start = x;
				x += 1;
				while x < region.end().x && sdf.sample(voxel_center(IVec3::new(x, y, z))) <= 0.0 {
					x += 1;
				}
				let min = IVec3::new(run_start, y, z);
				let region = NonZeroVoxelRegion::new(min, UVec3::new((x - run_start) as u32, 1, 1)).unwrap();
				edits.push(match &voxel {
					Some(voxel) => GridEdit::AddArea { region, voxel: voxel.clone() },
					None => GridEdit::RemoveArea { region },
				});
			}
		}
	}
	edits
}

/// Apply one edit directly and return the touched sub-grid positions.
pub fn apply_grid_edit(grid: &mut Grid, edit: &GridEdit) -> HashSet<IVec3> {
	match edit {
		GridEdit::Add { voxel_pos, voxel } => HashSet::from([grid.set_voxel(*voxel_pos, Some(voxel.get_ref()))]),
		GridEdit::Remove { voxel_pos } => HashSet::from([grid.set_voxel(*voxel_pos, None)]),
		GridEdit::AddArea { region, voxel } => grid.set_area(region.min(), region.size().as_ivec3(), Some(voxel.get_ref())),
		GridEdit::RemoveArea { region } => grid.set_area(region.min(), region.size().as_ivec3(), None),
		GridEdit::ApplySdf { bounds_min, bounds_max, face_resolution, iterations, voxel, sdf } => {
			grid.apply_sdf(*bounds_min, *bounds_max, &**sdf, *face_resolution, *iterations, voxel.get_ref())
		}
		GridEdit::ClearSdf { bounds_min, bounds_max, face_resolution, iterations, sdf } => {
			grid.clear_sdf(*bounds_min, *bounds_max, &**sdf, *face_resolution, *iterations)
		}
	}
}
