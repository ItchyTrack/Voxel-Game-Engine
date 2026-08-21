use bevy::math::{IVec3, UVec3};
use tile_data::{CHUNK_SIZE, NonZeroChunkRegion, TileVoxelReducer, VoxelRegionResult};
use voxel_data::grid_tree::{GridCoord, NonZeroVoxelRegion, SourceOverlaps as GridSourceOverlaps};
use voxel_data::voxel_grid_tree::VoxelGridType;
use voxel_data::voxels::{SourceOverlap, SourceTree, VoxelReducer, VoxelType, VoxelTypeId, Voxels};

use crate::{BasicVoxel, LodVoxel, MarchingVoxel};

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct BasicToLodVoxelReducer;

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct LodToLodVoxelReducer;

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct BasicToMarchingVoxelReducer;

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct MarchingToMarchingVoxelReducer;

impl TileVoxelReducer for BasicToLodVoxelReducer {
	fn input_type_id(&self) -> VoxelTypeId { BasicVoxel::TYPE_ID }
	fn output_type_id(&self) -> VoxelTypeId { LodVoxel::TYPE_ID }
	fn reduce(&self, area: NonZeroChunkRegion, output_lod: u8, inputs: &[&VoxelRegionResult]) -> Option<Voxels> {
		reduce_lod_voxels(area, output_lod, inputs)
	}
}

impl TileVoxelReducer for LodToLodVoxelReducer {
	fn input_type_id(&self) -> VoxelTypeId { LodVoxel::TYPE_ID }
	fn output_type_id(&self) -> VoxelTypeId { LodVoxel::TYPE_ID }
	fn reduce(&self, area: NonZeroChunkRegion, output_lod: u8, inputs: &[&VoxelRegionResult]) -> Option<Voxels> {
		reduce_lod_voxels(area, output_lod, inputs)
	}
}

impl TileVoxelReducer for BasicToMarchingVoxelReducer {
	fn input_type_id(&self) -> VoxelTypeId { BasicVoxel::TYPE_ID }
	fn output_type_id(&self) -> VoxelTypeId { MarchingVoxel::TYPE_ID }
	fn reduce(&self, area: NonZeroChunkRegion, output_lod: u8, inputs: &[&VoxelRegionResult]) -> Option<Voxels> {
		reduce_marching_voxels(area, output_lod, inputs)
	}
}

impl TileVoxelReducer for MarchingToMarchingVoxelReducer {
	fn input_type_id(&self) -> VoxelTypeId { MarchingVoxel::TYPE_ID }
	fn output_type_id(&self) -> VoxelTypeId { MarchingVoxel::TYPE_ID }
	fn reduce(&self, area: NonZeroChunkRegion, output_lod: u8, inputs: &[&VoxelRegionResult]) -> Option<Voxels> {
		reduce_marching_voxels(area, output_lod, inputs)
	}
}

pub fn downsample_region(region: NonZeroChunkRegion, lod: f32, fetch: impl Fn(IVec3) -> Option<Voxels>) -> Option<Voxels> {
	let min = region.min();
	let size = region.size();
	let scale_down = lod.max(0.0).floor() as u8;
	let step = 1i32 << scale_down as u32;

	let mut loaded = Vec::new();
	for chunk_z in 0..size.z {
		for chunk_y in 0..size.y {
			for chunk_x in 0..size.x {
				let local = IVec3::new(chunk_x as i32, chunk_y as i32, chunk_z as i32);
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
			output_offset: (*local * CHUNK_SIZE as i32).div_euclid(IVec3::splat(step)),
		})
		.collect();
	let source_infos = sources
		.iter()
		.map(|source| SourceInfo { scale_down: source.scale_down, output_offset: source.output_offset })
		.collect();
	let output_size = div_ceil_uvec3(size * CHUNK_SIZE, step as u32);
	let output_region = NonZeroVoxelRegion::from_min_size(IVec3::ZERO, output_size)?;
	Voxels::reduce_voxels(output_region, &sources, BasicVoxelReducer { sources: source_infos })
}

fn result_sources<'a>(
	area: NonZeroChunkRegion,
	output_lod: u8,
	inputs: &[&'a VoxelRegionResult],
) -> Option<(NonZeroVoxelRegion, Vec<SourceTree<'a>>)> {
	let output_step = 1i32.checked_shl(output_lod as u32)?;
	let output_size = div_ceil_uvec3(area.size() * CHUNK_SIZE, output_step as u32);
	let output_region = NonZeroVoxelRegion::from_min_size(IVec3::ZERO, output_size)?;
	let sources = inputs
		.iter()
		.map(|input| Some(SourceTree {
			voxels: &input.voxels,
			scale_down: output_lod.checked_sub(input.lod)?,
			output_offset: ((input.area.min() - area.min()) * CHUNK_SIZE as i32).div_euclid(IVec3::splat(output_step)),
		}))
		.collect::<Option<Vec<_>>>()?;
	Some((output_region, sources))
}

fn reduce_lod_voxels(area: NonZeroChunkRegion, output_lod: u8, inputs: &[&VoxelRegionResult]) -> Option<Voxels> {
	let (output_region, sources) = result_sources(area, output_lod, inputs)?;
	let source_infos = sources
		.iter()
		.map(|source| SourceInfo { scale_down: source.scale_down, output_offset: source.output_offset })
		.collect();
	Voxels::reduce_voxels(output_region, &sources, BasicVoxelReducer { sources: source_infos })
}

fn reduce_marching_voxels(area: NonZeroChunkRegion, output_lod: u8, inputs: &[&VoxelRegionResult]) -> Option<Voxels> {
	let (output_region, sources) = result_sources(area, output_lod, inputs)?;
	Voxels::reduce_voxels(output_region, &sources, MarchingVoxelReducer)
}

#[derive(Clone, Copy)]
struct SourceInfo {
	scale_down: u8,
	output_offset: IVec3,
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
			let voxel = if overlap.data.type_id() == BasicVoxel::TYPE_ID {
				BasicVoxel::from_voxel_ref(&overlap.data)
			} else if overlap.data.type_id() == MarchingVoxel::TYPE_ID {
				MarchingVoxel::from_voxel_ref(&overlap.data).0
			} else {
				continue;
			};
			let volume = region_volume(overlap.source_region);
			for (sum, channel) in color.iter_mut().zip(voxel.color) {
				*sum += u64::from(channel) * volume;
			}
			mass += u64::from(voxel.mass) * volume;
			weight += volume;
		}
		if weight == 0 { return None; }
		Some(MarchingVoxel(BasicVoxel {
			color: color.map(|sum| ((sum + weight / 2) / weight).min(255) as u8),
			mass: ((mass + weight / 2) / weight).min(u64::from(u32::MAX)) as u32,
		}))
	}
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

fn div_ceil_uvec3(value: UVec3, divisor: u32) -> UVec3 {
	UVec3::new(div_ceil(value.x, divisor), div_ceil(value.y, divisor), div_ceil(value.z, divisor))
}

fn div_ceil(value: u32, divisor: u32) -> u32 {
	let floor = value.div_euclid(divisor);
	if value.rem_euclid(divisor) == 0 { floor } else { floor + 1 }
}

fn round_channel(sum: u64, weight: u64) -> u8 {
	((sum + weight / 2) / weight).min(255) as u8
}

#[cfg(test)]
mod tests {
	use bevy::math::{IVec3, U16Vec3};
	use tile_data::{NonZeroChunkRegion, TileVoxelReducerRegistry, VoxelRegionResult};
	use voxel_data::voxels::{VoxelType, Voxels};

	use super::{BasicToLodVoxelReducer, BasicVoxel, LodVoxel};

	fn basic_result(chunk: IVec3, color: [u8; 4]) -> VoxelRegionResult {
		let mut voxels = Voxels::new::<BasicVoxel>();
		voxels.add_voxel(U16Vec3::ZERO, BasicVoxel { color, mass: 1 }.get_ref());
		VoxelRegionResult { area: NonZeroChunkRegion::from_single(chunk), lod: 0, voxels }
	}

	#[test]
	fn stored_chunks_are_downsampled_at_offsets_relative_to_the_requested_area() {
		let area = NonZeroChunkRegion::new(IVec3::new(4, 0, 0), bevy::math::UVec3::new(2, 1, 1)).unwrap();
		let first = basic_result(IVec3::new(4, 0, 0), [255, 0, 0, 255]);
		let second = basic_result(IVec3::new(5, 0, 0), [0, 255, 0, 255]);
		let inputs = [&first, &second];
		let output = tile_data::TileVoxelReducer::reduce(&BasicToLodVoxelReducer, area, 1, &inputs).unwrap();

		let first = LodVoxel::from_voxel_ref(&output.voxel(&U16Vec3::ZERO).unwrap());
		let second = LodVoxel::from_voxel_ref(&output.voxel(&U16Vec3::new(32, 0, 0)).unwrap());
		assert!(first.colors.contains(&[255, 0, 0, 255]));
		assert!(second.colors.contains(&[0, 255, 0, 255]));
	}

	#[test]
	fn registry_combines_finer_fallback_data_with_exact_requested_lod_data() {
		let area = NonZeroChunkRegion::new(IVec3::new(-2, 0, 0), bevy::math::UVec3::new(2, 1, 1)).unwrap();
		let raw = basic_result(IVec3::new(-2, 0, 0), [255, 0, 0, 255]);
		let mut exact_voxels = Voxels::new::<LodVoxel>();
		exact_voxels.add_voxel(U16Vec3::ZERO, LodVoxel::solid([0, 0, 255, 255]).get_ref());
		let exact = VoxelRegionResult {
			area: NonZeroChunkRegion::from_single(IVec3::new(-1, 0, 0)),
			lod: 1,
			voxels: exact_voxels,
		};
		let registry = TileVoxelReducerRegistry::default();
		registry.insert(BasicToLodVoxelReducer);
		let output = registry.reduce(area, 1, LodVoxel::TYPE_ID, &[raw, exact]).unwrap();

		let raw = LodVoxel::from_voxel_ref(&output.voxel(&U16Vec3::ZERO).unwrap());
		let exact = LodVoxel::from_voxel_ref(&output.voxel(&U16Vec3::new(32, 0, 0)).unwrap());
		assert!(raw.colors.contains(&[255, 0, 0, 255]));
		assert_eq!(exact, LodVoxel::solid([0, 0, 255, 255]));
	}
}
