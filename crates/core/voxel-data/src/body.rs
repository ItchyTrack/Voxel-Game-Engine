use bevy::prelude::*;

/// A [`Body`] entity.
pub type BodyId = Entity;

/// A collection of child grids sharing one transform. Each grid's local transform
/// places it within the body; physics participation is defined separately.
#[derive(Component, Default, Debug, Clone, Copy)]
#[require(Transform)]
pub struct Body;
