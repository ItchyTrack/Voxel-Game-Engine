use bevy::ecs::resource::Resource;

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct FreezeRenderRequests(pub bool);
