use bevy::prelude::*;
use voxel_math::{Fixed, FixedVec3};
use voxel_transform::Transform;
use super::fixed_math::ShapeMath;

use voxel_data::grid::Grid;
use voxel_trees::sdf::Sdf;
use voxel_data::voxels::{VoxelRef, VoxelType};
use voxel_sources::SourceManager;
use voxel_sources::edit::GridEditIdManager;
use voxel_lightyear::ReplicateVoxels;
use voxel_streaming::{GridStreaming, RequestChunkPresence};

use basic_voxel::{BasicVoxel, LodVoxel};
use voxel_content::{SdfSource, SdfSourceOptions, VoxelSdf};

const POWER: Fixed = Fixed::from_bits(8 << 24);
const ITERATIONS: u32 = 8;
const BAILOUT: Fixed = Fixed::from_bits(8 << 24);
const SCALE: Fixed = Fixed::from_bits(480 << 24);
/// Grid-local presence radius in voxels. Keep this tied to SCALE so making the
/// Mandelbulb larger also expands the claimed chunk-presence area.
const BOUNDS_RADIUS: Fixed = Fixed::from_bits(SCALE.to_bits() * 7 / 4);
const COST: u32 = 20;

#[derive(Clone, Debug)]
struct MandelbulbSdf {
	voxel: BasicVoxel,
	lod_voxel: LodVoxel,
}

impl Default for MandelbulbSdf {
	fn default() -> Self {
		Self {
			voxel: BasicVoxel { color: [220, 128, 128, 255], mass: 0 },
			lod_voxel: LodVoxel::solid([220, 128, 128, 255]),
		}
	}
}

impl MandelbulbSdf {
	fn local(pos: FixedVec3) -> FixedVec3 {
		pos / SCALE
	}

	fn estimate(pos: FixedVec3) -> (Fixed, u32) {
		let c = Self::local(pos);
		let mut z = c;
		// Store the reciprocal derivative to avoid overflow near the surface.
		let mut inverse_dr = Fixed::ONE;
		let mut r = Fixed::ZERO;
		let mut escaped_at = ITERATIONS;

		for i in 0..ITERATIONS {
			r = z.length();
			if r > BAILOUT {
				escaped_at = i;
				break;
			}

			let safe_r = r.max(Fixed::from_num(1.0e-6));
			let theta = (z.z / safe_r).clamp(Fixed::NEG_ONE, Fixed::ONE).acos() * POWER;
			let phi = z.y.atan2(z.x) * POWER;
			let zr = safe_r.powi(8);
			let derivative_factor = POWER * safe_r.powi(7);
			if inverse_dr != Fixed::ZERO {
				inverse_dr /= derivative_factor + inverse_dr;
			}

			let sin_theta = theta.sin();
			z = zr * FixedVec3::new(sin_theta * phi.cos(), sin_theta * phi.sin(), theta.cos()) + c;
		}

		if escaped_at == ITERATIONS {
			// The classic Mandelbulb distance estimator is unsigned for interior
			// points. Return a small negative distance so the lazy voxel source's
			// `sample <= 0` rule treats non-escaped samples as solid.
			(Fixed::from_num(-0.5), escaped_at)
		} else {
			let distance = Fixed::from_num(0.5) * r.ln() * r * inverse_dr;
			((distance * SCALE).max(Fixed::EPSILON), escaped_at)
		}
	}
}

impl Sdf for MandelbulbSdf {
	fn sample(&self, pos: FixedVec3) -> Fixed {
		Self::estimate(pos).0
	}
}

impl VoxelSdf for MandelbulbSdf {
	fn voxel(&self) -> VoxelRef<'_> {
		self.voxel.get_ref()
	}

	fn lod_voxel(&self) -> VoxelRef<'_> {
		self.lod_voxel.get_ref()
	}

	fn bounds(&self) -> Option<(FixedVec3, FixedVec3)> {
		Some((FixedVec3::splat(-BOUNDS_RADIUS), FixedVec3::splat(BOUNDS_RADIUS)))
	}
}

pub fn spawn_mandelbulb_grid(mut commands: Commands, mut source: ResMut<SourceManager>) {
	let entity = commands
		.spawn((
			Transform::from_xyz(0, 0, -1000),
			Grid::new::<BasicVoxel>(),
			GridEditIdManager::default(),
			GridStreaming::default(),
			RequestChunkPresence,
			ReplicateVoxels,
		))
		.id();
	source.get_source_mut::<SdfSource>().unwrap().set_grid_sdf_with_options(entity, MandelbulbSdf::default(), SdfSourceOptions {
		cost: COST,
		sample_radius_scale: Fixed::ONE,
	});
}
