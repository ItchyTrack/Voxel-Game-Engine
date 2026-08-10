use bevy::ecs::{component::Component, entity::Entity};

#[derive(Component, Debug, Clone, Default)]
pub struct VoxelMarchingCamera {
	pub tiles_to_render: Vec<Entity>,
}
