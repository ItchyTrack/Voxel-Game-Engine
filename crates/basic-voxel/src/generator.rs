use bevy::math::IVec3;
use voxel_data::grid_tree::{GridCoord, NonZeroVoxelRegion, SourceOverlaps as GridSourceOverlaps};
use voxel_data::voxel_grid_tree::VoxelGridType;
use voxel_data::voxels::{SourceOverlap, SourceTree, VoxelReducer, VoxelType, VoxelTypeId, Voxels};
use voxel_sources::VoxelLodGenerator;
use tile_data::CHUNK_SIZE;

use crate::{BasicVoxel, LodVoxel, MarchingVoxel};

#[derive(Clone, Copy, Debug, Default)]
pub struct BasicVoxelLodGenerator;

#[derive(Clone, Copy, Debug, Default)]
pub struct LodVoxelLodGenerator;

#[derive(Clone, Copy, Debug, Default)]
pub struct MarchingVoxelLodGenerator;

impl VoxelLodGenerator for BasicVoxelLodGenerator {
	fn input_type_id(&self) -> VoxelTypeId {
		BasicVoxel::TYPE_INFO.id
	}

	fn output_type_id(&self) -> VoxelTypeId {
		LodVoxel::TYPE_INFO.id
	}

	fn generate(&self, min: IVec3, size: IVec3, lod: f32, fetch: &dyn Fn(IVec3) -> Option<Voxels>) -> Option<Voxels> {
		downsample_region(min, size, lod, fetch)
	}
}

impl VoxelLodGenerator for LodVoxelLodGenerator {
	fn input_type_id(&self) -> VoxelTypeId { LodVoxel::TYPE_INFO.id }
	fn output_type_id(&self) -> VoxelTypeId { LodVoxel::TYPE_INFO.id }
	fn generate(&self, min: IVec3, size: IVec3, lod: f32, fetch: &dyn Fn(IVec3) -> Option<Voxels>) -> Option<Voxels> {
		downsample_region(min, size, lod, fetch)
	}
}

impl VoxelLodGenerator for MarchingVoxelLodGenerator {
	fn input_type_id(&self) -> VoxelTypeId { MarchingVoxel::TYPE_INFO.id }
	fn output_type_id(&self) -> VoxelTypeId { MarchingVoxel::TYPE_INFO.id }
	fn generate(&self, min: IVec3, size: IVec3, lod: f32, fetch: &dyn Fn(IVec3) -> Option<Voxels>) -> Option<Voxels> {
		downsample_marching_region(min, size, lod, fetch)
	}
}

pub fn downsample_marching_region(min: IVec3, size: IVec3, lod: f32, fetch: impl Fn(IVec3) -> Option<Voxels>) -> Option<Voxels> {
	let scale_down = lod.max(0.0).floor() as u8;
	let step = 1i32 << scale_down as u32;
	let mut loaded = Vec::new();
	for z in 0..size.z {
		for y in 0..size.y {
			for x in 0..size.x {
				let local = IVec3::new(x, y, z);
				if let Some(voxels) = fetch(min + local) { loaded.push((local, voxels)); }
			}
		}
	}
	if loaded.is_empty() { return None; }
	let sources: Vec<_> = loaded.iter().map(|(local, voxels)| SourceTree {
		voxels,
		scale_down,
		output_offset: (*local * CHUNK_SIZE).div_euclid(IVec3::splat(step)),
	}).collect();
	let output_size = div_ceil_ivec3(size * CHUNK_SIZE, step);
	let output_region = NonZeroVoxelRegion::from_min_size(IVec3::ZERO, output_size)?;
	Voxels::reduce_voxels(output_region, &sources, MarchingVoxelReducer)
}

struct MarchingVoxelReducer;

impl VoxelReducer for MarchingVoxelReducer {
	type Output = MarchingVoxel;

	fn reduce<'overlaps, 'a, Co>(
		&mut self,
		_region: NonZeroVoxelRegion,
		overlaps: GridSourceOverlaps<'overlaps, 'a, VoxelGridType, Co>,
	) -> Option<Self::Output>
	where
		Co: GridCoord,
	{
		let mut color = [0u64; 4];
		let mut mass = 0u64;
		let mut weight = 0u64;
		for overlap in overlaps {
			let voxel = MarchingVoxel::from_voxel_ref(&overlap.data);
			let volume = region_volume(overlap.source_region);
			for (sum, channel) in color.iter_mut().zip(voxel.0.color) {
				*sum += u64::from(channel) * volume;
			}
			mass += u64::from(voxel.0.mass) * volume;
			weight += volume;
		}
		if weight == 0 { return None; }
		Some(MarchingVoxel(BasicVoxel {
			color: color.map(|sum| ((sum + weight / 2) / weight).min(255) as u8),
			mass: ((mass + weight / 2) / weight).min(u64::from(u32::MAX)) as u32,
		}))
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
	fn from_color(color: [u8; 4], volume: u64) -> Self {
		Self {
			color: [color[0] as u64 * volume, color[1] as u64 * volume, color[2] as u64 * volume],
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
	let source_infos = sources
		.iter()
		.map(|source| SourceInfo { scale_down: source.scale_down, output_offset: source.output_offset })
		.collect();
	let output_size = div_ceil_ivec3(size * CHUNK_SIZE, step);
	let output_region = NonZeroVoxelRegion::from_min_size(IVec3::ZERO, output_size)?;
	Voxels::reduce_voxels(output_region, &sources, BasicVoxelReducer { sources: source_infos })
}

#[derive(Clone, Copy)]
struct SourceInfo {
	scale_down: u8,
	output_offset: IVec3,
}

struct BasicVoxelReducer {
	sources: Vec<SourceInfo>,
}

impl VoxelReducer for BasicVoxelReducer {
	type Output = LodVoxel;

	fn reduce<'overlaps, 'a, Co>(
		&mut self,
		region: NonZeroVoxelRegion,
		overlaps: GridSourceOverlaps<'overlaps, 'a, VoxelGridType, Co>,
	) -> Option<Self::Output>
	where
		Co: GridCoord,
	{
		reduce_basic_voxel(region, &self.sources, overlaps.map(|overlap| SourceOverlap {
			source_index: overlap.source_index,
			source_region: overlap.source_region,
			output_region: overlap.output_region,
			data: overlap.data,
		}))
	}
}

fn reduce_basic_voxel<'a>(region: NonZeroVoxelRegion, sources: &[SourceInfo], overlaps: impl Iterator<Item = SourceOverlap<'a>>) -> Option<LodVoxel> {
	let mut octants = [Accum::default(); 8];
	let mut total = Accum::default();
	for overlap in overlaps {
		let Some(source) = sources.get(overlap.source_index).copied() else { continue };
		let volume = region_volume(overlap.source_region);
		if overlap.data.type_id() == BasicVoxel::TYPE_ID {
			let voxel = BasicVoxel::from_voxel_ref(&overlap.data);
			let accum = Accum::from_color(voxel.color, volume * 8);
			octants[octant_for_region(region, source, overlap.source_region)]
				.add(accum);
			total.add(accum);
		} else if overlap.data.type_id() == LodVoxel::TYPE_ID {
			let voxel = LodVoxel::from_voxel_ref(&overlap.data);
			let child_volume = volume;
			for child in 0..8 {
				let accum = Accum::from_color(voxel.colors[child], child_volume);
				octants[octant_for_child_region(region, source, overlap.source_region, child)]
					.add(accum);
				total.add(accum);
			}
		}
	}
	if total.weight == 0 {
		return None;
	}
	let colors = std::array::from_fn(|i| {
		if octants[i].weight == 0 {
			[0, 0, 0, 0]
		} else {
			let color = octants[i].average();
			[color[0], color[1], color[2], 255]
		}
	});
	Some(LodVoxel { colors })
}

fn octant_for_region(output_region: NonZeroVoxelRegion, source: SourceInfo, source_region: NonZeroVoxelRegion) -> usize {
	let center = [
		(source_region.min().x + source_region.end().x) as f32 * 0.5,
		(source_region.min().y + source_region.end().y) as f32 * 0.5,
		(source_region.min().z + source_region.end().z) as f32 * 0.5,
	];
	octant_for_source_point(output_region, source, center)
}

fn octant_for_child_region(output_region: NonZeroVoxelRegion, source: SourceInfo, source_region: NonZeroVoxelRegion, child: usize) -> usize {
	let size = source_region.size();
	let center = [
		source_region.min().x as f32 + size.x as f32 * if (child & 1) != 0 { 0.75 } else { 0.25 },
		source_region.min().y as f32 + size.y as f32 * if (child & 2) != 0 { 0.75 } else { 0.25 },
		source_region.min().z as f32 + size.z as f32 * if (child & 4) != 0 { 0.75 } else { 0.25 },
	];
	octant_for_source_point(output_region, source, center)
}

fn octant_for_source_point(output_region: NonZeroVoxelRegion, source: SourceInfo, source_point: [f32; 3]) -> usize {
	let step = 1i32 << source.scale_down as u32;
	let preimage_min = (output_region.min() - source.output_offset) * step;
	let preimage_size = output_region.size() * step as u32;
	let threshold = [
		preimage_min.x as f32 + preimage_size.x as f32 * 0.5,
		preimage_min.y as f32 + preimage_size.y as f32 * 0.5,
		preimage_min.z as f32 + preimage_size.z as f32 * 0.5,
	];
	((source_point[0] >= threshold[0]) as usize)
		| (((source_point[1] >= threshold[1]) as usize) << 1)
		| (((source_point[2] >= threshold[2]) as usize) << 2)
}

fn region_volume(region: NonZeroVoxelRegion) -> u64 {
	let size = region.size();
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

