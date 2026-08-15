use bevy::math::IVec3;
use tile_data::ChunkRegion;
use voxel_data::grid_tree::{GridType, NonZeroVoxelRegion};
use voxel_data::signed_grid_tree::SignedGridTree;

#[derive(Clone, Copy, Debug, Default)]
struct GenerationCell;

impl GridType for GenerationCell {
	type Data<'a> = u64;
	const MAX_NODE_OFFSET: u32 = u32::MAX;

	fn data_size_bytes(&self) -> usize { std::mem::size_of::<u64>() }

	fn read_data<'a>(&self, bytes: &'a [u8]) -> Self::Data<'a> {
		u64::from_le_bytes(bytes[..8].try_into().expect("generation bytes"))
	}

	fn write_data(&self, data: Self::Data<'_>, bytes: &mut [u8]) {
		bytes[..8].copy_from_slice(&data.to_le_bytes());
	}

	fn data_eq_bytes(&self, data: Self::Data<'_>, bytes: &[u8]) -> bool {
		bytes[..8] == data.to_le_bytes()
	}
}

/// Spatial last-change generations for one grid. Large commands remain compact
/// because homogeneous chunk regions share tree cells.
#[derive(Clone, Debug, Default)]
pub struct ChunkGenerationIndex {
	tree: SignedGridTree<GenerationCell>,
}

impl ChunkGenerationIndex {
	pub fn chunk_generation(&self, chunk: IVec3) -> u64 {
		self.tree.get(&chunk).unwrap_or(0)
	}

	pub fn last_changed(&self, region: ChunkRegion) -> u64 {
		let Some(region) = NonZeroVoxelRegion::from_min_size(region.min(), region.size().as_ivec3()) else { return 0 };
		let mut latest = 0;
		self.tree.for_each_in_region(region, |_, _, generation| latest = latest.max(generation));
		latest
	}

	/// Record that all chunks in `region` were changed at `generation`. Older
	/// out-of-order observations never overwrite a newer known generation.
	pub fn set_region(&mut self, region: ChunkRegion, generation: u64) {
		if region.is_empty() || self.last_changed(region) > generation { return; }
		self.tree.add_area(&region.min(), region.size().as_ivec3(), generation);
	}

	pub fn set_chunk(&mut self, chunk: IVec3, generation: u64) {
		if self.chunk_generation(chunk) <= generation { self.tree.insert(&chunk, generation); }
	}

	pub fn unchanged_since(&self, region: ChunkRegion, generation: u64) -> bool {
		self.last_changed(region) <= generation
	}
}
