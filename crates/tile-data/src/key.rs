use bevy::math::IVec3;

use crate::TileClassId;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TileKey {
	pub min: IVec3,
	pub size: IVec3,
	pub lod: u8,
	pub class: TileClassId,
}
