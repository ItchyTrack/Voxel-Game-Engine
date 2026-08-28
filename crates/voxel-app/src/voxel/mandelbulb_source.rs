use bevy::math::Vec3;
use bevy::prelude::*;

use voxel_data::grid::Grid;
use voxel_trees::sdf::Sdf;
use voxel_data::voxels::{VoxelRef, VoxelType};
use voxel_sources::SourceManager;
use voxel_sources::edit::GridEditIdManager;
use voxel_lightyear::ReplicateVoxels;
use voxel_streaming::{GridStreaming, RequestChunkPresence};

use basic_voxel::{BasicVoxel, LodVoxel};
use voxel_content::{SdfSource, SdfSourceOptions, VoxelSdf};

const POWER: f32 = 8.0;
const ITERATIONS: u32 = 8;
const BAILOUT: f32 = 8.0;
const SCALE: f32 = 480.0;
/// Grid-local presence radius in voxels. Keep this tied to SCALE so making the
/// Mandelbulb larger also expands the claimed chunk-presence area.
const BOUNDS_RADIUS: f32 = SCALE * 1.75;
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
	fn local(pos: Vec3) -> Vec3 {
		pos / SCALE
	}

	fn estimate(pos: Vec3) -> (f32, u32) {
		let c = Self::local(pos);
		let mut z = c;
		let mut dr = 1.0f32;
		let mut r = 0.0f32;
		let mut escaped_at = ITERATIONS;

		for i in 0..ITERATIONS {
			r = z.length();
			if r > BAILOUT {
				escaped_at = i;
				break;
			}

			let safe_r = r.max(1.0e-6);
			let theta = (z.z / safe_r).clamp(-1.0, 1.0).acos() * POWER;
			let phi = z.y.atan2(z.x) * POWER;
			let zr = safe_r.powf(POWER);
			dr = POWER * safe_r.powf(POWER - 1.0) * dr + 1.0;

			let sin_theta = theta.sin();
			z = zr * Vec3::new(sin_theta * phi.cos(), sin_theta * phi.sin(), theta.cos()) + c;
		}

		if escaped_at == ITERATIONS {
			// The classic Mandelbulb distance estimator is unsigned for interior
			// points. Return a small negative distance so the lazy voxel source's
			// `sample <= 0` rule treats non-escaped samples as solid.
			(-0.5, escaped_at)
		} else {
			let distance = 0.5 * r.ln() * r / dr;
			(distance * SCALE, escaped_at)
		}
	}
}

impl Sdf for MandelbulbSdf {
	fn sample(&self, pos: Vec3) -> f32 {
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

	fn bounds(&self) -> Option<(Vec3, Vec3)> {
		Some((Vec3::splat(-BOUNDS_RADIUS), Vec3::splat(BOUNDS_RADIUS)))
	}
}

pub fn spawn_mandelbulb_grid(mut commands: Commands, mut source: ResMut<SourceManager>) {
	let entity = commands
		.spawn((
			Transform::from_translation(Vec3::new(0.0, 0.0, -1000.0)),
			Grid::new::<BasicVoxel>(),
			GridEditIdManager::default(),
			GridStreaming::default(),
			RequestChunkPresence,
			ReplicateVoxels,
		))
		.id();
	source.get_source_mut::<SdfSource>().unwrap().set_grid_sdf_with_options(entity, MandelbulbSdf::default(), SdfSourceOptions {
		cost: COST,
		sample_radius_scale: 1.0,
	});
}
