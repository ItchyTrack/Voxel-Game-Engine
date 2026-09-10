use std::collections::HashSet;
use bevy::{ecs::system::SystemParam, prelude::{ChildOf, Entity, Query}};
use voxel_data::grid::GridLocalTransform;
use crate::Transform;

#[derive(SystemParam)]
pub struct TransformQuery<'w, 's> {
	transforms: Query<'w, 's, (Option<&'static Transform>, Option<&'static GridLocalTransform>, Option<&'static ChildOf>)>,
}

impl TransformQuery<'_, '_> {
	/// Recompose the current parent chain, without a cache or propagation pass.
	/// Missing entities, cycles, and unrepresentable scales return None.
	/// Entities without a local transform inherit their parent's transform.
	pub fn get_world(&self, entity: Entity) -> Option<Transform> {
		let mut chain = Vec::new();
		let mut seen = HashSet::new();
		let mut current = entity;
		let mut has_transform = false;
		loop {
			if !seen.insert(current) { return None; }
			let (transform, grid_local, parent) = self.transforms.get(current).ok()?;
			has_transform |= transform.is_some() || grid_local.is_some();
			let mut local = transform.copied().unwrap_or_default();
			// If both are present, the grid offset is in the transform's local axes.
			if let Some(offset) = grid_local {
				local = local.checked_mul(Transform::from_translation(offset.0.into()))?;
			}
			chain.push(local);
			let Some(parent) = parent else { break };
			current = parent.parent();
		}
		if !has_transform { return None; }
		let mut world = Transform::IDENTITY;
		for local in chain.into_iter().rev() { world = world.checked_mul(local)?; }
		Some(world)
	}
}
