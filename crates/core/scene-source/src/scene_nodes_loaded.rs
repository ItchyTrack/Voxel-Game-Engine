use bevy::prelude::*;

#[derive(Debug, Message)]
pub struct scene_nodesLoaded {
	entity: Entity,
}
