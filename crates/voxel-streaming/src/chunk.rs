use bevy::math::IVec3;

pub const CHUNK_SIZE: i32 = 64;

pub fn chunk_of(voxel_pos: IVec3) -> IVec3 {
	voxel_pos.div_euclid(IVec3::splat(CHUNK_SIZE))
}

pub fn chunk_origin(chunk: IVec3) -> IVec3 {
	chunk * CHUNK_SIZE
}
