use bevy::math::{IVec3, UVec3};
use bile_math::NonZeroRegion;

pub const CHUNK_SIZE: u32 = 64;

pub fn chunk_of(voxel_position: IVec3) -> IVec3 {
	voxel_position.div_euclid(IVec3::splat(CHUNK_SIZE as i32))
}

pub fn chunk_origin(chunk_position: IVec3) -> IVec3 {
	chunk_position * CHUNK_SIZE as i32
}

pub fn chunk_region(chunk_position: IVec3) -> NonZeroRegion {
	NonZeroRegion::new(chunk_origin(chunk_position), UVec3::splat(CHUNK_SIZE)).unwrap()
}

pub fn chunk_aligned(region: NonZeroRegion) -> NonZeroRegion {
	NonZeroRegion::from_min_end(chunk_origin(chunk_of(region.min())), chunk_origin(chunk_of(region.max()) + IVec3::ONE)).unwrap()
}

pub fn chunks_in(region: NonZeroRegion) -> impl Iterator<Item = IVec3> {
	let min = chunk_of(region.min());
	let max = chunk_of(region.max());
	(min.x..=max.x).flat_map(move |x| (min.y..=max.y).flat_map(move |y| (min.z..=max.z).map(move |z| IVec3::new(x, y, z))))
}
