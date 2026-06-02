use bevy::math::IVec3;
use bevy::transform::components::Transform;

use crate::grid_tree::GridTree;

/// Lifecycle of a chunk, stored directly as the presence tree's cell value.
/// A chunk absent from the tree is either unknown or confirmed empty.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ChunkState {
	/// Present in the source data, not yet requested.
	Available,
	/// Requested, awaiting load.
	InFlight,
	/// Loaded into the grid.
	Loaded,
}

impl ChunkState {
	fn encode(self) -> u16 {
		match self {
			ChunkState::Available => 0,
			ChunkState::InFlight => 1,
			ChunkState::Loaded => 2,
		}
	}

	fn decode(value: u16) -> Self {
		match value {
			1 => ChunkState::InFlight,
			2 => ChunkState::Loaded,
			_ => ChunkState::Available,
		}
	}
}

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
		self.tree.insert(&chunk.as_i16vec3(), ChunkState::Available.encode());
	}

	pub fn set_state(&mut self, chunk: IVec3, state: ChunkState) {
		self.tree.insert(&chunk.as_i16vec3(), state.encode());
	}

	pub fn state(&self, chunk: IVec3) -> Option<ChunkState> {
		self.tree.get(&chunk.as_i16vec3()).map(ChunkState::decode)
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

	/// Visit every present chunk inside the inclusive chunk region `[min, max]`,
	/// descending only into subtrees that overlap it. Used for collision
	/// broad-phase so cost scales with the region, not the whole footprint.
	pub fn for_each_in_region(&self, min: IVec3, max: IVec3, mut f: impl FnMut(IVec3)) {
		self.tree.for_each_in_region(min.as_i16vec3(), max.as_i16vec3(), |origin, size, _| {
			let origin = origin.as_ivec3();
			let lo = origin.max(min);
			let hi = (origin + IVec3::splat(size as i32) - IVec3::ONE).min(max);
			for x in lo.x..=hi.x {
				for y in lo.y..=hi.y {
					for z in lo.z..=hi.z {
						f(IVec3::new(x, y, z));
					}
				}
			}
		});
	}
}
