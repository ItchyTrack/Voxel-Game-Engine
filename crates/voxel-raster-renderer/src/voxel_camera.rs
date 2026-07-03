use bevy::ecs::{component::Component, entity::Entity};

#[derive(Component, Debug, Clone, Default)]
pub struct VoxelRasterCamera {
	pub subgrids_to_render: Vec<Entity>,
	pub lods_to_render: Vec<Entity>,
}
