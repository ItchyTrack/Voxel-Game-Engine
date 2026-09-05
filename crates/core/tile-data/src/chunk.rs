use bevy::math::{IVec3, UVec3};
use voxel_trees::{
	define_region_types,
	region::{NonZeroVoxelRegion, VoxelRegion},
};

pub const CHUNK_SIZE: u32 = 64;

define_region_types!(ChunkRegion, NonZeroChunkRegion);

pub fn chunk_of(voxel_position: IVec3) -> IVec3 {
	voxel_position.div_euclid(IVec3::splat(CHUNK_SIZE as i32))
}

pub fn chunk_origin(chunk_position: IVec3) -> IVec3 {
	chunk_position * CHUNK_SIZE as i32
}

pub fn voxel_region_from_chunks(region: ChunkRegion) -> VoxelRegion {
	VoxelRegion::new(chunk_origin(region.min()), region.size() * CHUNK_SIZE)
}

pub fn nonzero_voxel_region_from_chunks(region: NonZeroChunkRegion) -> NonZeroVoxelRegion {
	NonZeroVoxelRegion::new(chunk_origin(region.min()), region.size() * CHUNK_SIZE).unwrap()
}

pub fn chunks_covering_voxel_region(region: VoxelRegion) -> ChunkRegion {
	let Ok(nonzero_region) = region.try_into() else {
		return ChunkRegion::new(chunk_of(region.min()), UVec3::ZERO);
	};
	chunks_covering_nonzero_voxel_region(nonzero_region).into()
}

pub fn chunks_covering_nonzero_voxel_region(region: NonZeroVoxelRegion) -> NonZeroChunkRegion {
	NonZeroChunkRegion::from_min_max(chunk_of(region.min()), chunk_of(region.max())).unwrap()
}
