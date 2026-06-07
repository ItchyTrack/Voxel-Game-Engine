use bevy::math::IVec3;
use bevy::transform::components::Transform;

use voxel_data::grid_tree::{GridTree, I32Coord};
use voxel_data::voxel_grid_tree::PackedCell;

type ChunkGridTree = GridTree<PackedCell, I32Coord>;

/// Lifecycle of a chunk. A chunk absent from the tree is either unknown or
/// confirmed empty.
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
	fn code(self) -> u16 {
		match self {
			ChunkState::Available => 0,
			ChunkState::InFlight => 1,
			ChunkState::Loaded => 2,
		}
	}

	fn from_code(code: u16) -> Self {
		match code {
			1 => ChunkState::InFlight,
			2 => ChunkState::Loaded,
			_ => ChunkState::Available,
		}
	}
}

// A present chunk's cell value packs its state in the low 2 bits and the count
// of objects requesting it in the remaining bits.
const STATE_BITS: u16 = 2;
const STATE_MASK: u16 = (1 << STATE_BITS) - 1;

fn encode(state: ChunkState, count: u16) -> u16 {
	(count << STATE_BITS) | state.code()
}

fn decode(value: u16) -> (ChunkState, u16) {
	(ChunkState::from_code(value & STATE_MASK), value >> STATE_BITS)
}

pub struct ChunkPresence {
	tree: ChunkGridTree,
}

impl Default for ChunkPresence {
	fn default() -> Self {
		Self { tree: ChunkGridTree::new() }
	}
}

impl ChunkPresence {
	pub fn mark_present(&mut self, chunk: IVec3) {
		self.tree.insert(&chunk, encode(ChunkState::Available, 0));
	}

	pub fn mark_present_area(&mut self, min: IVec3, size: IVec3) {
		self.tree.add_area(&min, size, encode(ChunkState::Available, 0));
	}

	pub fn clear_present_area(&mut self, min: IVec3, size: IVec3) {
		self.tree.remove_area(&min, size);
	}

	pub fn set_state(&mut self, chunk: IVec3, state: ChunkState) {
		let count = self.request_count(chunk);
		self.tree.insert(&chunk, encode(state, count));
	}

	pub fn state(&self, chunk: IVec3) -> Option<ChunkState> {
		self.tree.get(&chunk).map(|v| decode(v).0)
	}

	/// Number of objects currently requesting `chunk` (0 if absent).
	pub fn request_count(&self, chunk: IVec3) -> u16 {
		self.tree.get(&chunk).map_or(0, |v| decode(v).1)
	}

	/// Record one more object requesting `chunk`. No-op if the chunk is absent.
	pub fn add_request(&mut self, chunk: IVec3) {
		if let Some((state, count)) = self.tree.get(&chunk).map(decode) {
			self.tree.insert(&chunk, encode(state, count + 1));
		}
	}

	/// Drop one object's request for `chunk`, returning the remaining count.
	pub fn remove_request(&mut self, chunk: IVec3) -> u16 {
		let Some((state, count)) = self.tree.get(&chunk).map(decode) else { return 0; };
		let count = count.saturating_sub(1);
		self.tree.insert(&chunk, encode(state, count));
		count
	}

	pub fn clear_present(&mut self, chunk: IVec3) {
		self.tree.remove(&chunk);
	}

	pub fn is_present(&self, chunk: IVec3) -> bool {
		self.tree.get(&chunk).is_some()
	}

	/// `transform` rotation maps +Z onto the ray direction (matches the voxel tree).
	pub fn raycast(&self, transform: &Transform, max_length: Option<f32>) -> Option<(IVec3, f32)> {
		self.tree.raycast(transform, max_length).map(|(pos, _, dist)| (pos, dist))
	}

	pub fn iter_present(&self) -> impl Iterator<Item = (IVec3, u32)> + '_ {
		self.tree.iter().map(|(origin, size, _)| (origin, size))
	}

	/// Visit every present chunk inside the inclusive chunk region `[min, max]`,
	/// descending only into subtrees that overlap it. Used for collision
	/// broad-phase so cost scales with the region, not the whole footprint.
	pub fn for_each_in_region(&self, min: IVec3, max: IVec3, mut f: impl FnMut(IVec3)) {
		self.tree.for_each_in_region(min, max, |origin, size, _| {
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
