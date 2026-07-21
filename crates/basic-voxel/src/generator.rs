use bevy::math::IVec3;
use voxel_data::grid_tree::GridRegion;
use voxel_data::voxels::{SourceOverlap, SourceTree, VoxelRef, VoxelType, VoxelTypeId, Voxels};
use voxel_sources::VoxelLodGenerator;
use voxel_streaming::CHUNK_SIZE;

use crate::BasicVoxel;

#[derive(Clone, Copy, Debug, Default)]
pub struct BasicVoxelLodGenerator;

impl VoxelLodGenerator for BasicVoxelLodGenerator {
	fn input_type_id(&self) -> VoxelTypeId {
		BasicVoxel::TYPE_INFO.id
	}

	fn generate(&self, min: IVec3, size: IVec3, lod: f32, fetch: &dyn Fn(IVec3) -> Option<Voxels>) -> Option<Voxels> {
		downsample_region(min, size, lod, fetch)
	}
}

#[derive(Clone, Copy, Default)]
struct Accum {
	color: [u64; 3],
	weight: u64,
}

impl Accum {
	fn add(&mut self, other: Accum) {
		for c in 0..3 { self.color[c] += other.color[c]; }
		self.weight += other.weight;
	}
	fn from_leaf(voxel: &BasicVoxel, volume: u64) -> Self {
		Self {
			color: [voxel.color[0] as u64 * volume, voxel.color[1] as u64 * volume, voxel.color[2] as u64 * volume],
			weight: volume,
		}
	}
	fn average(&self) -> [u8; 3] {
		std::array::from_fn(|c| round_channel(self.color[c], self.weight))
	}
}

pub fn downsample_region(min: IVec3, size: IVec3, lod: f32, fetch: impl Fn(IVec3) -> Option<Voxels>) -> Option<Voxels> {
	let scale_down = lod.max(0.0).floor() as u8;
	let step = 1i32 << scale_down as u32;

	let mut loaded = Vec::new();
	for chunk_z in 0..size.z {
		for chunk_y in 0..size.y {
			for chunk_x in 0..size.x {
				let local = IVec3::new(chunk_x, chunk_y, chunk_z);
				if let Some(voxels) = fetch(min + local) {
					loaded.push((local, voxels));
				}
			}
		}
	}
	if loaded.is_empty() {
		return None;
	}

	let sources: Vec<_> = loaded
		.iter()
		.map(|(local, voxels)| SourceTree {
			voxels,
			scale_down,
			output_offset: (*local * CHUNK_SIZE).div_euclid(IVec3::splat(step)),
		})
		.collect();
	let output_size = div_ceil_ivec3(size * CHUNK_SIZE, step);
	let output_region = GridRegion::from_min_size(IVec3::ZERO, output_size)?;
	let mut generated = Vec::new();
	Voxels::reduce_voxels(output_region, &sources, |_, overlaps| reduce_basic_voxel(overlaps, &mut generated))
}

fn reduce_basic_voxel<'a>(overlaps: impl Iterator<Item = SourceOverlap<'a>>, generated: &mut Vec<BasicVoxel>) -> Option<VoxelRef<'a>> {
	let mut accum = Accum::default();
	for overlap in overlaps {
		let voxel = BasicVoxel::from_voxel_ref(&overlap.data);
		accum.add(Accum::from_leaf(&voxel, region_volume(overlap.source_region)));
	}
	if accum.weight == 0 {
		return None;
	}
	let color = accum.average();
	generated.push(BasicVoxel { color: [color[0], color[1], color[2], 255], mass: 0 });
	let voxel = generated.last().expect("just pushed generated voxel").get_ref();
	Some(unsafe { std::mem::transmute::<VoxelRef<'_>, VoxelRef<'a>>(voxel) })
}

fn region_volume(region: GridRegion) -> u64 {
	let size = region.size().as_uvec3();
	size.x as u64 * size.y as u64 * size.z as u64
}

fn div_ceil_ivec3(value: IVec3, divisor: i32) -> IVec3 {
	IVec3::new(div_ceil(value.x, divisor), div_ceil(value.y, divisor), div_ceil(value.z, divisor))
}

fn div_ceil(value: i32, divisor: i32) -> i32 {
	let floor = value.div_euclid(divisor);
	if value.rem_euclid(divisor) == 0 { floor } else { floor + 1 }
}

fn round_channel(sum: u64, weight: u64) -> u8 {
	((sum + weight / 2) / weight).min(255) as u8
}

