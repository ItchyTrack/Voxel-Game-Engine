use bevy::math::IVec3;
use bevy::transform::components::Transform;

use crate::grid_tree::GridTree;

pub struct ChunkPresence {
	tree: GridTree,
}

impl Default for ChunkPresence {
	fn default() -> Self {
		Self { tree: GridTree::new() }
	}
}

impl ChunkPresence {
	pub fn mark_present(&mut self, chunk: IVec3) {
		self.tree.insert(&chunk.as_i16vec3(), 0);
	}

	pub fn clear_present(&mut self, chunk: IVec3) {
		self.tree.remove(&chunk.as_i16vec3());
	}

	pub fn is_present(&self, chunk: IVec3) -> bool {
		self.tree.get(&chunk.as_i16vec3()).is_some()
	}

	/// `transform` rotation maps +Z onto the ray direction (matches the voxel tree).
	pub fn raycast(&self, transform: &Transform, max_length: Option<f32>) -> Option<(IVec3, f32)> {
		self.tree.raycast(transform, max_length).map(|(pos, _, dist)| (pos.as_ivec3(), dist))
	}

	pub fn iter_present(&self) -> impl Iterator<Item = (IVec3, u16)> + '_ {
		self.tree.iter().map(|(origin, size, _)| (origin.as_ivec3(), size))
	}
}
