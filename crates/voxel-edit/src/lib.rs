use std::sync::Arc;

use bevy::math::{IVec2, IVec3, Vec3};
use bevy::prelude::*;

use voxel_data::region::{NonZeroVoxelRegion, VoxelRegion};
use voxel_data::sdf::{Sdf, shrink_aabb_with_sdf, voxel_center, voxel_region_from_bounds};
use voxel_data::voxels::Voxel;

#[derive(Clone)]
pub enum ResolvedGridEdit {
	AddArea { region: NonZeroVoxelRegion, voxel: Voxel },
	RemoveArea { region: NonZeroVoxelRegion },
}

impl ResolvedGridEdit {
	pub fn voxel_bounds(&self) -> NonZeroVoxelRegion {
		match self {
			ResolvedGridEdit::AddArea { region, .. } | ResolvedGridEdit::RemoveArea { region } => *region,
		}
	}

	/// Restrict this edit to `limit`
	pub fn clipped_to(&self, limit: NonZeroVoxelRegion) -> Option<Self> {
		match self {
			ResolvedGridEdit::AddArea { region, voxel } => region.intersection(limit).map(|region| ResolvedGridEdit::AddArea { region, voxel: voxel.clone() }),
			ResolvedGridEdit::RemoveArea { region } => region.intersection(limit).map(|region| ResolvedGridEdit::RemoveArea { region }),
		}
	}
}

#[derive(Clone)]
pub enum GridEdit {
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
			GridEdit::AddArea { region, .. } | GridEdit::RemoveArea { region } => (*region).into(),
			GridEdit::ApplySdf { bounds_min, bounds_max, .. } | GridEdit::ClearSdf { bounds_min, bounds_max, .. } => {
				let min = bounds_min.floor().as_ivec3();
				VoxelRegion::new(min, (bounds_max.ceil().as_ivec3() - min).as_uvec3())
			}
		}
	}

	/// Restrict this edit to `limit`
	pub fn clipped_to(&self, limit: NonZeroVoxelRegion) -> Option<Self> {
		match self {
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

	pub fn into_resolved(self) -> Vec<ResolvedGridEdit> {
		match self {
			GridEdit::AddArea { region, voxel } => {
				vec![ResolvedGridEdit::AddArea { region, voxel }]
			}
			GridEdit::RemoveArea { region } => {
				vec![ResolvedGridEdit::RemoveArea { region }]
			}
			GridEdit::ApplySdf { bounds_min, bounds_max, face_resolution, iterations, voxel, sdf } => {
				sdf_area_edits(bounds_min, bounds_max, face_resolution, iterations, &*sdf, Some(voxel.clone()))
			}
			GridEdit::ClearSdf { bounds_min, bounds_max, face_resolution, iterations, sdf } => {
				sdf_area_edits(bounds_min, bounds_max, face_resolution, iterations, &*sdf, None)
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
) -> Vec<ResolvedGridEdit> {
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
					Some(voxel) => ResolvedGridEdit::AddArea { region, voxel: voxel.clone() },
					None => ResolvedGridEdit::RemoveArea { region },
				});
			}
		}
	}
	edits
}
