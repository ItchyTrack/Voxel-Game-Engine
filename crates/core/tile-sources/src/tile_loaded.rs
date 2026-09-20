use bevy::prelude::*;

#[derive(Debug, Message)]
pub struct TileLoaded {
	entity: Entity,
}
