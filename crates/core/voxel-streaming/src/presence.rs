use std::collections::HashMap;

use bevy::prelude::*;
use bile_math::NonZeroRegion;
use bile_region_loader::RegionUpdated;
use bile_scene::NodeBounds;
use tile_data::{chunk_aligned, chunk_origin};
use voxel_data::grid::GridId;
use voxel_sources::{SourceManager, SourceResult, SourceResultData, edit::GridEditMessage};

use crate::VoxelTileSource;

#[derive(Component, Debug, Default)]
pub struct RequestChunkPresence;

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

pub(crate) fn request_presence(
	mut source: ResMut<VoxelTileSource>,
	mut sources: ResMut<SourceManager>,
	grids: Query<GridId, With<RequestChunkPresence>>,
) {
	for grid in &grids {
		let tiles = source.grids.entry(grid).or_default();
		if tiles.presence_request.is_none() {
			tiles.presence_request = Some(sources.request_presence(grid));
		}
	}
}

pub(crate) fn receive_presence(
	mut commands: Commands,
	mut source: ResMut<VoxelTileSource>,
	mut results: MessageReader<SourceResult>,
	mut edits: MessageReader<GridEditMessage>,
	mut bounds: Query<&mut NodeBounds>,
) {
	let mut added: HashMap<GridId, NonZeroRegion> = HashMap::new();
	let mut add = |grid: GridId, region: NonZeroRegion| {
		added.entry(grid).and_modify(|bounds| *bounds = union(*bounds, region)).or_insert(region);
	};
	for result in results.read() {
		match &result.data {
			SourceResultData::Presence { grid, region } => add(*grid, *region),
			SourceResultData::PresenceLoaded => {
				for (&grid, tiles) in &mut source.grids {
					if tiles.presence_request != Some(result.request_id) { continue; }
					tiles.presence_request = None;
					commands.entity(grid).try_remove::<RequestChunkPresence>();
				}
			}
			_ => {}
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
