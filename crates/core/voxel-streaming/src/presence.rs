use std::collections::{HashMap, HashSet};

use bevy::prelude::*;
use bile_data::DataConsumingNode;
use bile_math::NonZeroRegion;
use bile_region_loader::RegionUpdated;
use bile_scene::NodeBounds;
use tile_data::{chunk_aligned, chunk_origin};
use voxel_data::grid::GridId;
use voxel_sources::{PresenceRegions, PresenceRequester, VoxelPresence, edit::GridEditMessage};

#[derive(Component, Debug, Default)]
pub struct RequestChunkPresence;

/// Requests [`VoxelPresence`] for grids with [`RequestChunkPresence`] and grows their [`NodeBounds`].
#[derive(Component, Default)]
pub struct ChunkPresenceNode {
	requested: HashSet<GridId>,
	received: Vec<(GridId, PresenceRegions)>,
}

impl DataConsumingNode<VoxelPresence> for ChunkPresenceNode {
	fn pass_data(&mut self, grid: &GridId, _tick: u64, regions: PresenceRegions) {
		self.received.push((*grid, regions));
	}

	fn dirty_data(&mut self, _grid: &GridId, _last_clean_tick: u64) {}
}

pub fn bounds_of_chunks(chunks: impl IntoIterator<Item = IVec3>) -> Option<NodeBounds> {
	let mut chunks = chunks.into_iter();
	let first = chunks.next()?;
	let (min, max) = chunks.fold((first, first), |(min, max), chunk| (min.min(chunk), max.max(chunk)));
	Some(NodeBounds::new(NonZeroRegion::from_min_end(chunk_origin(min), chunk_origin(max + IVec3::ONE)).unwrap()))
}

fn union(a: NonZeroRegion, b: NonZeroRegion) -> NonZeroRegion {
	NonZeroRegion::from_min_end(a.min().min(b.min()), a.end().max(b.end())).unwrap()
}

fn grow_bounds(commands: &mut Commands, bounds: &mut Query<&mut NodeBounds>, added: HashMap<GridId, NonZeroRegion>) {
	for (grid, region) in added {
		match bounds.get_mut(grid) {
			Ok(mut bounds) => {
				if !bounds.region().contains_region(region) {
					let region = union(bounds.region(), region);
					bounds.set_region(region);
				}
			}
			Err(_) => { commands.entity(grid).try_insert(NodeBounds::new(region)); }
		}
	}
}

pub(crate) fn update_presence(
	mut commands: Commands,
	node: Single<(Entity, &mut ChunkPresenceNode)>,
	grids: Query<GridId, With<RequestChunkPresence>>,
	mut requester: PresenceRequester,
	mut edits: MessageReader<GridEditMessage>,
	mut bounds: Query<&mut NodeBounds>,
) {
	let (entity, node) = node.into_inner();
	let node = node.into_inner();
	for grid in &grids {
		if !node.requested.insert(grid) { continue; }
		if let Some(regions) = requester.request(grid, entity) {
			node.received.push((grid, regions));
		}
	}

	let mut added: HashMap<GridId, NonZeroRegion> = HashMap::new();
	let mut add = |grid: GridId, region: NonZeroRegion| {
		added.entry(grid).and_modify(|bounds| *bounds = union(*bounds, region)).or_insert(region);
	};
	for (grid, regions) in std::mem::take(&mut node.received) {
		for &region in regions.iter() {
			add(grid, region);
		}
		if node.requested.remove(&grid) {
			requester.cancel(grid, entity);
			commands.entity(grid).try_remove::<RequestChunkPresence>();
		}
	}
	for edit in edits.read() {
		add(edit.grid_id(), chunk_aligned(edit.edit().affected_region()));
	}
	grow_bounds(&mut commands, &mut bounds, added);
}

pub(crate) fn publish_bounds_updates(
	grids: Query<(Entity, &NodeBounds), Changed<NodeBounds>>,
	mut updates: MessageWriter<RegionUpdated>,
) {
	for (entity, bounds) in &grids {
		updates.write(RegionUpdated { entity, region: bounds.region() });
	}
}
