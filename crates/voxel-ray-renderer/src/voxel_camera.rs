use bevy::ecs::{component::Component, entity::Entity};

#[derive(Component, Debug, Clone, Default)]
pub struct VoxelCamera {
	pub subgrids_to_render: Vec<Entity>,
	pub lods_to_render: Vec<Entity>,
}
